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

// UORB Topic'leri buraya eklencek
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>

// uORB::Publication<vehicle_command_s>	_vehCmd_pub{ORB_ID(vehicle_command)};
orb_advert_t _vehCmd_pub{nullptr};

int publishCmd(orb_advert_t* handle, vehicle_command_s* cmd)
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

void checkRedStatus(vehicle_status_s* redStatus, vehicle_status_s* vehStatus, hrt_abstime last_update, bool everPublished)
{
	if (vehStatus->red_state == 1)
	{
		/* code */
	}
	else
	{
		if (everPublished)
		{
			hrt_abstime absTimeSinceLastUpdate = hrt_elapsed_time(&last_update);
			// PX4_INFO("absTimeSinceLastUpdate : %lu", absTimeSinceLastUpdate);
			if (redStatus->arming_state != vehStatus->arming_state)
			{
				vehicle_command_s vehCmd;
				vehCmd.target_system = vehStatus->system_id;
				vehCmd.target_component = vehStatus->component_id;
				vehCmd.source_system = vehStatus->system_id;
				vehCmd.source_component = vehStatus->component_id;

				switch (redStatus->arming_state)
				{
				case vehicle_status_s::ARMING_STATE_STANDBY:
					PX4_WARN("red arming status: standby");
					vehCmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
					vehCmd.param1 = 0;
					vehCmd.param2 = 21196;
					publishCmd(&_vehCmd_pub, &vehCmd);
					break;

				case vehicle_status_s::ARMING_STATE_ARMED:
					PX4_WARN("red arming status: armed");
					vehCmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
					vehCmd.param1 = 1;
					publishCmd(&_vehCmd_pub, &vehCmd);
					// _vehCmd_pub.publish(vehCmd);

					vehCmd.command = vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
					vehCmd.param1 = 0;
					publishCmd(&_vehCmd_pub, &vehCmd);
					// _vehCmd_pub.publish(vehCmd);
					break;
				default:
					break;
				}
			}


			if ( absTimeSinceLastUpdate > 150 * 1E3)
			{
				// PX4_INFO("SHOULD TAKE CONTROL !!!");
				vehStatus->red_state = 1;
				PX4_WARN("RED_STATE 1 OLDU !!!!!!!!!!!!!!");
			}

		}
	}


}




#endif /* REDUNDANCY_MANAGER_H_ */
