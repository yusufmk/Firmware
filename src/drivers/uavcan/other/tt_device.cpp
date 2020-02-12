#include "tt_device.hpp"

#include <cstdint>

#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <mathlib/mathlib.h>

const char *const TtDevice::NAME = "redundantUKB";

TtDevice::TtDevice(uavcan::INode &node) :
	_node(node),
	_canSub_armStat(node),
	_canPub_armStat(node),
	_orb_to_uavcan_pub_timer(node, TimerCbBinder(this, &TtDevice::broadcast_from_orb)),
	_yusuf_message_pub{nullptr}
{
}

TtDevice::~TtDevice()
{
	(void) orb_unsubscribe(_yusuf_message_sub);
}

int TtDevice::init()
{
	int res = _canPub_armStat.init(uavcan::TransferPriority::MiddleLower);

	if (res < 0)
	{
		PX4_WARN("arming status CAN publication failed %i", res);
	}

	res = _canSub_armStat.start(ArmStatCbBinder(this, &TtDevice::armStat_cb));
	if (res < 0)
	{
		PX4_WARN("arming status CAN subscription failed %i", res);
	}

	_orb_to_uavcan_pub_timer.startPeriodic(
		uavcan::MonotonicDuration::fromUSec(1000000U / ORB_TO_UAVCAN_FREQUENCY_HZ));

	return res;
}
