/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file voted_sensors_update.cpp
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include "voted_sensors_update.h"

#include <systemlib/mavlink_log.h>

using namespace temperature_compensation;
using namespace time_literals;

VotedSensorsUpdate::VotedSensorsUpdate() :
	ModuleParams(nullptr),
	ScheduledWorkItem(px4::wq_configurations::lp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, "temperature_compensation"))
{
	// initialise the publication variables
	for (unsigned i = 0; i < 3; i++) {
		_corrections.gyro_scale_0[i] = 1.0f;
		_corrections.accel_scale_0[i] = 1.0f;
		_corrections.gyro_scale_1[i] = 1.0f;
		_corrections.accel_scale_1[i] = 1.0f;
		_corrections.gyro_scale_2[i] = 1.0f;
		_corrections.accel_scale_2[i] = 1.0f;
	}

	_corrections.baro_scale_0 = 1.0f;
	_corrections.baro_scale_1 = 1.0f;
	_corrections.baro_scale_2 = 1.0f;
}

VotedSensorsUpdate::~VotedSensorsUpdate()
{
	perf_free(_loop_perf);
}

bool
VotedSensorsUpdate::init()
{
	ScheduleOnInterval(1_s);

	return true;
}

void VotedSensorsUpdate::parameters_update()
{
	/* temperature compensation */
	_temperature_compensation.parameters_update();

	// gyro
	for (uint8_t uorb_index = 0; uorb_index < GYRO_COUNT_MAX; uorb_index++) {
		sensor_gyro_s report;

		if (_gyro_subs[uorb_index].copy(&report)) {
			int temp = _temperature_compensation.set_sensor_id_gyro(report.device_id, uorb_index);

			if (temp < 0) {
				PX4_ERR("%s temp compensation init: failed to find device ID %u for instance %i", "gyro", report.device_id, uorb_index);
				_corrections.gyro_mapping[uorb_index] = 0;

			} else {
				_corrections.gyro_mapping[uorb_index] = temp;
			}
		}
	}

	// accel
	for (uint8_t uorb_index = 0; uorb_index < ACCEL_COUNT_MAX; uorb_index++) {
		sensor_accel_s report;

		if (_accel_subs[uorb_index].copy(&report)) {
			int temp = _temperature_compensation.set_sensor_id_accel(report.device_id, uorb_index);

			if (temp < 0) {
				PX4_ERR("%s temp compensation init: failed to find device ID %u for instance %i", "accel", report.device_id,
					uorb_index);
				_corrections.accel_mapping[uorb_index] = 0;

			} else {
				_corrections.accel_mapping[uorb_index] = temp;
			}
		}
	}

	// baro
	for (uint8_t uorb_index = 0; uorb_index < BARO_COUNT_MAX; uorb_index++) {
		sensor_baro_s report;

		if (_baro_subs[uorb_index].copy(&report)) {
			int temp = _temperature_compensation.set_sensor_id_baro(report.device_id, uorb_index);

			if (temp < 0) {
				PX4_ERR("%s temp compensation init: failed to find device ID %u for instance %i", "baro", report.device_id, uorb_index);
				_corrections.baro_mapping[uorb_index] = 0;

			} else {
				_corrections.baro_mapping[uorb_index] = temp;

			}
		}
	}
}

void VotedSensorsUpdate::accel_poll()
{
	float *offsets[] = {_corrections.accel_offset_0, _corrections.accel_offset_1, _corrections.accel_offset_2 };
	float *scales[] = {_corrections.accel_scale_0, _corrections.accel_scale_1, _corrections.accel_scale_2 };

	for (uint8_t uorb_index = 0; uorb_index < ACCEL_COUNT_MAX; uorb_index++) {
		sensor_accel_s report;

		if (_accel_subs[uorb_index].update(&report)) {

			matrix::Vector3f accel_data = matrix::Vector3f(report.x, report.y, report.z);

			// handle temperature compensation
			if (_temperature_compensation.apply_corrections_accel(uorb_index, accel_data, report.temperature, offsets[uorb_index],
					scales[uorb_index]) == 2) {
				_corrections_changed = true;
			}
		}
	}
}

void VotedSensorsUpdate::gyro_poll()
{
	float *offsets[] = {_corrections.gyro_offset_0, _corrections.gyro_offset_1, _corrections.gyro_offset_2 };
	float *scales[] = {_corrections.gyro_scale_0, _corrections.gyro_scale_1, _corrections.gyro_scale_2 };

	for (uint8_t uorb_index = 0; uorb_index < GYRO_COUNT_MAX; uorb_index++) {
		sensor_gyro_s report;

		if (_gyro_subs[uorb_index].update(&report)) {

			matrix::Vector3f gyro_rate = matrix::Vector3f(report.x, report.y, report.z);

			// handle temperature compensation
			if (_temperature_compensation.apply_corrections_gyro(uorb_index, gyro_rate, report.temperature, offsets[uorb_index],
					scales[uorb_index]) == 2) {
				_corrections_changed = true;
			}
		}
	}
}

void VotedSensorsUpdate::baro_poll()
{
	float *offsets[] = {&_corrections.baro_offset_0, &_corrections.baro_offset_1, &_corrections.baro_offset_2 };
	float *scales[] = {&_corrections.baro_scale_0, &_corrections.baro_scale_1, &_corrections.baro_scale_2 };

	for (uint8_t uorb_index = 0; uorb_index < BARO_COUNT_MAX; uorb_index++) {
		sensor_baro_s report;

		if (_baro_subs[uorb_index].update(&report)) {

			// Convert from millibar to Pa
			float corrected_pressure = 100.0f * report.pressure;

			// handle temperature compensation
			if (_temperature_compensation.apply_corrections_baro(uorb_index, corrected_pressure, report.temperature,
					offsets[uorb_index], scales[uorb_index]) == 2) {
				_corrections_changed = true;
			}
		}
	}
}

void VotedSensorsUpdate::Run()
{
	perf_begin(_loop_perf);

	/* Check if any parameter has changed */
	parameter_update_s update;

	if (_params_sub.update(&update)) {
		/* read from param to clear updated flag */

		parameters_update();
		//updateParams();
	}

	accel_poll();
	gyro_poll();
	baro_poll();

	// publish sensor corrections if necessary
	if (_corrections_changed) {
		_corrections.timestamp = hrt_absolute_time();

		if (_sensor_correction_pub == nullptr) {
			_sensor_correction_pub = orb_advertise(ORB_ID(sensor_correction), &_corrections);

		} else {
			orb_publish(ORB_ID(sensor_correction), _sensor_correction_pub, &_corrections);
		}

		_corrections_changed = false;
	}

	perf_end(_loop_perf);
}

int VotedSensorsUpdate::task_spawn(int argc, char *argv[])
{
	VotedSensorsUpdate *instance = new VotedSensorsUpdate();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int VotedSensorsUpdate::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VotedSensorsUpdate::print_status()
{
	_temperature_compensation.print_status();

	return PX4_OK;
}

int VotedSensorsUpdate::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The temperature compensation module is central to the whole system. It takes low-level output from drivers, turns
it into a more usable form, and publishes it for the rest of the system.

The provided functionality includes:
- Read the output from the sensor drivers (`sensor_gyro`, etc.).
  If there are multiple of the same type, do voting and failover handling.
  Then apply the board rotation and temperature calibration (if enabled). And finally publish the data; one of the
  topics is `sensor_combined`, used by many parts of the system.
- Do RC channel mapping: read the raw input channels (`input_rc`), then apply the calibration, map the RC channels
  to the configured channels & mode switches, low-pass filter, and then publish as `rc_channels` and
  `manual_control_setpoint`.
- Read the output from the ADC driver (via ioctl interface) and publish `battery_status`.
- Make sure the sensor drivers get the updated calibration parameters (scale & offset) when the parameters change or
  on startup. The sensor drivers use the ioctl interface for parameter updates. For this to work properly, the
  sensor drivers must already be running when `sensors` is started.
- Do preflight sensor consistency checks and publish the `sensor_preflight` topic.

### Implementation
It runs in its own thread and polls on the currently selected gyro topic.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("temperature_compensation", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int temperature_compensation_main(int argc, char *argv[]);

int temperature_compensation_main(int argc, char *argv[])
{
	return VotedSensorsUpdate::main(argc, argv);
}
