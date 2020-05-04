/****************************************************************************
 *
 ****************************************************************************/

/**
 * @file RedundancyManager.h
 * redundancy manager helper functions definitions
 *
 * @author M.Yusuf Korkut <mkorkut@thy.com>
 */

#ifndef REDUNDANCY_MANAGER_H_
#define REDUNDANCY_MANAGER_H_

#include <drivers/drv_hrt.h>
#include <px4_log.h>

#include "px4_custom_mode.h"

// UORB Topic'leri buraya eklencek
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>

// uORB::Publication<vehicle_command_s>	_vehCmd_pub{ORB_ID(vehicle_command)};
orb_advert_t _vehCmd_pub{nullptr};

int publishCmd(orb_advert_t *handle, vehicle_command_s *cmd)
{
	if (!*handle) {
		*handle = orb_advertise_queue(ORB_ID(vehicle_command), cmd, 2);

		if (*handle) {
			return 0;
		}

	} else {
		return orb_publish(ORB_ID(vehicle_command), *handle, cmd);
	}

	return -1;
}

void checkRedStatus(vehicle_status_s *redStatus, vehicle_status_s *vehStatus, hrt_abstime last_update,
		    bool everPublished)
{
	hrt_abstime absTimeSinceLastUpdate = hrt_elapsed_time(&last_update);

	if (vehStatus->red_state == 1) {
		if (redStatus->red_state == 1 && vehStatus->component_id != 1 && absTimeSinceLastUpdate < 100*1E3) {

			vehStatus->red_state = 0;
			PX4_WARN("RED_STATE 0 OLDU !!!!!!!!!!!!!!");
		}

	} else {
		if (everPublished) {

			// PX4_INFO("absTimeSinceLastUpdate : %lu", absTimeSinceLastUpdate);
			if (redStatus->arming_state != vehStatus->arming_state) {
				vehicle_command_s vehCmd;
				vehCmd.target_system = vehStatus->system_id;
				vehCmd.target_component = vehStatus->component_id;
				vehCmd.source_system = vehStatus->system_id;
				vehCmd.source_component = vehStatus->component_id;

				switch (redStatus->arming_state) {
				case vehicle_status_s::ARMING_STATE_STANDBY:
					PX4_WARN("red arming status: standby");
					vehCmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
					vehCmd.param1 = 0;
					// vehCmd.param2 = 21196;
					publishCmd(&_vehCmd_pub, &vehCmd);
					break;

				case vehicle_status_s::ARMING_STATE_ARMED:
					PX4_WARN("red arming status: armed");
					vehCmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
					vehCmd.param1 = 1;
					publishCmd(&_vehCmd_pub, &vehCmd);
					// _vehCmd_pub.publish(vehCmd);

					// vehCmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
					// vehCmd.param1 = 0b00000001;
					// vehCmd.param2 = 3;
					// publishCmd(&_vehCmd_pub, &vehCmd);
					// _vehCmd_pub.publish(vehCmd);
					break;

				default:
					break;
				}
			}

			if (redStatus->nav_state != vehStatus->nav_state)
			{
				vehicle_command_s vehCmd;
				vehCmd.target_system = vehStatus->system_id;
				vehCmd.target_component = vehStatus->component_id;
				vehCmd.source_system = vehStatus->system_id;
				vehCmd.source_component = vehStatus->component_id;
				vehCmd.param1 = 0b00000001;
				vehCmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;

				if (redStatus->nav_state <= 3)
				{
					vehCmd.param2 = redStatus->nav_state + 1;
					vehCmd.param3 = 4;
				}
				else if (redStatus->nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER)
				{
					vehCmd.param2 = PX4_CUSTOM_MAIN_MODE_AUTO;
					vehCmd.param3 = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
				}
				else if(redStatus->nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF)
				{
					vehCmd.param2 = PX4_CUSTOM_MAIN_MODE_AUTO;
					vehCmd.param3 = PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF;
				}
				else if(redStatus->nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL)
				{
					vehCmd.param2 = PX4_CUSTOM_MAIN_MODE_AUTO;
					vehCmd.param3 = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
				}

					publishCmd(&_vehCmd_pub, &vehCmd);
			}



			if (absTimeSinceLastUpdate > 150 * 1E3) {
				// PX4_INFO("SHOULD TAKE CONTROL !!!");
				vehStatus->red_state = 1;
				PX4_WARN("RED_STATE 1 OLDU !!!!!!!!!!!!!!");
			}

		}
	}


}




#endif /* REDUNDANCY_MANAGER_H_ */
