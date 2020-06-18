#include "mgm_hbc.hpp"
#include <drivers/drv_hrt.h>

#define MOTOR_BIT(x) (1<<(x))

mgm_hbc::mgm_hbc(uavcan::INode &node) :
	_node(node),
	_orb_timer(node),
	_tx_deadline(uavcan::MonotonicTime::fromMSec(100))
{
	_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
	_esc_status.esc_count = 8;
	for (size_t i = 0; i < _esc_status.esc_count; i++) {
		// mgm compro
		_esc_status.esc[i].esc_vendor = esc_status_s::ESC_VENDOR_GENERIC;
		// TMM xxxx - 3 HBC-series HighVoltage V7 400400-3 programmable brushless ESC
		_esc_status.esc[i].esc_version = 7;
		// P15 communication address  // 240, 224, 208, 192, 176, 160, 144, 128 olabilir.
		_esc_status.esc[i].esc_address = 240 - i * 16;
		// ????????????
		_esc_status.esc[i].esc_state = 1234;
	}
}

mgm_hbc::~mgm_hbc()
{

}

int mgm_hbc::init()
{
	// ESC status will be relayed from UAVCAN bus into ORB at this rate
	_orb_timer.setCallback(TimerCbBinder(this, &mgm_hbc::orb_pub_timer_cb));
	_orb_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / ESC_STATUS_UPDATE_RATE_HZ));

	return 0;
}

void mgm_hbc::handle_message(const uavcan::CanRxFrame *frame)
{
	// abritration field'in least significant byte'i,
	//	hem node id(esc comm addresss) hem msg id bilgisi iceriyor.
	//	msb 4bit = node id, lsb 4 bit = msg id
	uint8_t escAdd = frame->id & 0xF0;
	uint8_t escInd = (240 - escAdd) / 16;

	uint8_t msgId = frame->id & 0x0F;

	switch (msgId) {
	case PACKET1_ID:
		handle_message_packet1(frame->data, escInd);
		break;

	case PACKET2_ID:
		handle_message_packet2(frame->data, escInd);
		break;

	case PACKET3_ID:
		handle_message_packet3(frame->data, escInd);
		break;

	case PACKET4_ID:
		handle_message_packet4(frame->data, escInd);
		break;

	default:
		// TODO: error handling
		break;
	}
}

void mgm_hbc::handle_message_packet1(const uint8_t *data, const int escInd)
{
	float bat_v = (data[0] + 256 * data[1]) / 57.45f;
	float bat_i = (data[2] + 256 * data[3]) / 10.0f;
	uint32_t mtr_rpm = (data[4] + 256 * data[5] + 65536 * data[6]) * 10;

	_esc_status.esc[escInd].esc_voltage = bat_v;
	_esc_status.esc[escInd].esc_current = bat_i;
	_esc_status.esc[escInd].esc_rpm = mtr_rpm;
	// failsafe'te kullanılmalı

	uint64_t ts = hrt_absolute_time();
	_esc_status.esc[escInd].timestamp = ts;
	_esc_status.timestamp = ts;
}

void mgm_hbc::handle_message_packet2(const uint8_t *data,  const int escInd)
{
	// uint32_t odometer = data[0] + 256 * data[1] + 65536 * data[2] + 16777216 * data[3];
	uint8_t esc_temp = data[4];
	// uint8_t mtr_temp = data[5];
	// uint8_t bat_temp = data[6];
	// TODO: fill _esc_status
	_esc_status.esc[escInd].esc_temperature = static_cast<float>(esc_temp);;
		// TODO: esc_report uorb mesaji mtr temp ve bat temp icerecek sekilde modifiye edilmeli.
		// TODO: esc_report uorb mesaji odometer icerecek sekilde modifiye edilmeli.???

	// failsafe'te kullanilmali

	uint64_t ts = hrt_absolute_time();
	_esc_status.esc[escInd].timestamp = ts;
	_esc_status.timestamp = ts;
}

void mgm_hbc::handle_message_packet3(const uint8_t *data,  const int escInd)
{
	float req_pwm = (data[0] + 256 * data[1]) / 10.0f;
	// float real_pwm = (data[2] + 256 * data[3]) / 10.0f;
	uint32_t warnings = data[4] + 256 * data[5];
	uint32_t failures = data[6] + 256 * data[7];
	// TODO: fill _esc_status
	_esc_status.esc[escInd].esc_setpoint = req_pwm;
	// _esc_status.esc[escInd].esc_setpoint_raw = real_pwm;
	_esc_status.esc[escInd].esc_errorcount = (failures << 16) & warnings;

	// commander'da, navigator'da, failsafe'te kesin kullanılmalı

	uint64_t ts = hrt_absolute_time();
	_esc_status.esc[escInd].timestamp = ts;
	_esc_status.timestamp = ts;
}

void mgm_hbc::handle_message_packet4(const uint8_t *data,  const int escInd)
{
	// uint16_t bat_lvl = data[0] + 256 * data[1];
	// TODO: fill _esc_status
		// TODO: esc_report uorb mesajı battery level icerecek sekilde modifiye edilmeli.

	uint64_t ts = hrt_absolute_time();
	_esc_status.esc[escInd].timestamp = ts;
	_esc_status.timestamp = ts;
}

void mgm_hbc::orb_pub_timer_cb(const uavcan::TimerEvent &)
{
	_esc_status.counter += 1;

	if (_esc_status_pub != nullptr) {
		(void)orb_publish(ORB_ID(esc_status), _esc_status_pub, &_esc_status);

	} else {
		_esc_status_pub = orb_advertise(ORB_ID(esc_status), &_esc_status);
	}
}

void mgm_hbc::update_outputs(float *outputs, unsigned num_outputs)
{
	/*
	 * Rate limiting - we don't want to congest the bus
	 */
	const auto timestamp = _node.getMonotonicTime();

	if ((timestamp - _prev_cmd_pub).toUSec() < (1000000 / MAX_RATE_HZ)) {
		return;
	}

	_prev_cmd_pub = timestamp;

	/*
	 * Send commands.
	 * If unarmed, we publish an empty message anyway
	 */
	actuator_outputs_s actuator_outputs = {};
	actuator_outputs.noutputs = num_outputs;
	actuator_outputs.timestamp = hrt_absolute_time();

	// const float cmd_min = _run_at_idle_throttle_when_armed ? 1.0F : 0.0F;

	for (unsigned escIdx = 0; escIdx < num_outputs; escIdx++) {
		if (_armed_mask & MOTOR_BIT(escIdx)) {
			actuator_outputs.output[escIdx] = outputs[escIdx] * 1023;
			int16_t thrl_cmd = static_cast<int16_t>(actuator_outputs.output[escIdx]);
			// _esc_status.esc[i].esc_setpoint_raw = thrl_cmd
			// TODO: esc_setpoint_raw int 16 olacak sekilde modifiye edilmeli
			send_thrl_cmd(thrl_cmd, escIdx);
			// msg.cmd.push_back(static_cast<int>(scaled));


		} else {
			// msg.cmd.push_back(static_cast<unsigned>(0));
			send_thrl_cmd(-1024, escIdx);
			actuator_outputs.output[escIdx] = -1.0f;
		}
	}

	// Publish actuator outputs
	if (_actuator_outputs_pub != nullptr) {
		orb_publish(ORB_ID(actuator_outputs), _actuator_outputs_pub, &actuator_outputs);

	} else {
		int instance;
		_actuator_outputs_pub = orb_advertise_multi(ORB_ID(actuator_outputs), &actuator_outputs,
					&instance, ORB_PRIO_DEFAULT);
	}
}

void mgm_hbc::send_thrl_cmd(int16_t thrl_cmd, const int escInd)
{
	thrl_cmd = thrl_cmd > 1023 ? 1023 : thrl_cmd;
	thrl_cmd = thrl_cmd < -1024 ? -1024 : thrl_cmd;

	uint32_t frameId = BS + _esc_status.esc[escInd].esc_address + CMD_ID;
	uint8_t data[] = {static_cast<uint8_t>(thrl_cmd & 0xff), static_cast<uint8_t>(thrl_cmd >> 8) };

	uavcan::CanFrame myCanFrame{frameId, data, 2};
	/*int result =*/ _node.injectTxFrame(myCanFrame, _tx_deadline, 3U);
}

void mgm_hbc::arm_all_escs(bool arm)
{
	// TODO
	// yeni state arming, eger bundaysa _armed_mask'e bakarak tek tek arm edecek?!
}

void mgm_hbc::arm_single_esc(int num, bool arm)
{
	// TODO
}
