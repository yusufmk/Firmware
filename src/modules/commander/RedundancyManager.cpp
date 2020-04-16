/****************************************************************************
*
****************************************************************************/

/**
* @file RedundancyManager.cpp
*
* redundancy manager helper functions
*
* @author M.Yusuf Korkut <mkorkut@thy.com>
*/

#include "modules/commander/RedundancyManager.hpp"

// #include <math.h>
// #include <parameters/param.h>
// #include <systemlib/mavlink_log.h>
// using namespace time_literals;

// #include <uORB/Subscription.hpp>


bool anyArmed(arming_state_t armingState)
{
	if (armingState == vehicle_status_s::ARMING_STATE_MON_ARMED || armingState == vehicle_status_s::ARMING_STATE_OP_ARMED)
	{
		return true;
	}
	return false;
}
bool anyStandby(arming_state_t armingState)
{
	if (armingState == vehicle_status_s::ARMING_STATE_MON_STANDBY || armingState == vehicle_status_s::ARMING_STATE_OP_STANDBY)
	{
		return true;
	}
	return false;
}
bool anyStandbyError(arming_state_t armingState)
{
	if (armingState == vehicle_status_s::ARMING_STATE_MON_STANDBY_ERROR || armingState == vehicle_status_s::ARMING_STATE_OP_STANDBY_ERROR)
	{
		return true;
	}
	return false;
}

bool anyOp(arming_state_t armingState)
{
	if (armingState == vehicle_status_s::ARMING_STATE_OP_ARMED ||
		armingState == vehicle_status_s::ARMING_STATE_OP_STANDBY ||
		armingState == vehicle_status_s::ARMING_STATE_OP_STANDBY_ERROR)
	{
		return true;
	}
	return false;
}

bool anyMon(arming_state_t armingState)
{
	if (armingState == vehicle_status_s::ARMING_STATE_MON_ARMED ||
		armingState == vehicle_status_s::ARMING_STATE_MON_STANDBY ||
		armingState == vehicle_status_s::ARMING_STATE_MON_STANDBY_ERROR)
	{
		return true;
	}
	return false;
}
