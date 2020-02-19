#include "tt_device.hpp"

#include <cstdint>

#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <mathlib/mathlib.h>

#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_defines.h>

// #include "blah.hpp"

const char *const TtDevice::NAME = "redundantUKB";

TtDevice::TtDevice(uavcan::INode &node) :
	_node(node),
	_canSub_armStat(node),
	_canPub_armStat(node),
	// _orb_to_uavcan_pub_timer(node, TimerCbBinder(this, &TtDevice::broadcast_from_orb)),
	_param_to_uavcan_pub_timer(node, TimerCbBinder(this, &TtDevice::broadcast_from_param)),
	_yusuf_message_pub{nullptr},
	_myFrameListener{},
	_frameId{0},
	_sobalak{192}
{
	_p1_handle = param_find("YUSUF_PARAM_1");
	_p2_handle = param_find("YUSUF_PARAM_2");
	_sys_id_handle = param_find("MAV_SYS_ID");
	param_get(_p1_handle, &_p1);
	param_get(_p2_handle, &_p2);
	param_get(_sys_id_handle, &_sys_id);
	// _myFrameListener = new RxFrameListener();

}

TtDevice::~TtDevice()
{
	(void) orb_unsubscribe(_yusuf_message_sub);
	// delete(_myFrameListener);
}

int TtDevice::init()
{
	int res = _canPub_armStat.init(uavcan::TransferPriority::NumericallyMin);

	if (res < 0)
	{
		PX4_WARN("arming status CAN publication failed %i", res);
	}

	res = _canSub_armStat.start(ArmStatCbBinder(this, &TtDevice::armStat_cb));
	if (res < 0)
	{
		PX4_WARN("arming status CAN subscription failed %i", res);
	}

	_param_to_uavcan_pub_timer.startPeriodic(
		uavcan::MonotonicDuration::fromUSec(5000000U / ORB_TO_UAVCAN_FREQUENCY_HZ));


	_node.getDispatcher().installRxFrameListener(&_myFrameListener);

	return res;
}

void TtDevice::armStat_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::safety::ArmingStatus> &msg)
{
	// _benim_mesaj.timestamp = hrt_absolute_time();
	// _benim_mesaj.mesaj_val1 = msg.status;
	// int orb_instance;
	// orb_publish_auto(ORB_ID(yusuf_message), &_yusuf_message_pub,
	// 	&_benim_mesaj, &orb_instance, ORB_PRIO_DEFAULT);

	PX4_INFO("armStat_cb icine girdi, gelen veri: %d, src node id: %d, this.sys_id: %d", msg.status, msg.getSrcNodeID().get(), _sys_id);
	_p2 = msg.status;
	param_set(_p2_handle, &_p2);
	// Doing less time critical stuff here
	// if (_orb_to_uavcan_pub_timer.isRunning())
	// {
	// 	_orb_to_uavcan_pub_timer.stop();
	// 	PX4_WARN("GNSS ORB->UAVCAN bridge stopped, because there are other GNSS publishers");
	// }
}

void TtDevice::broadcast_from_param(const uavcan::TimerEvent &)
{
	param_get(_p1_handle, &_p1);
	// Convert to UAVCAN
	using uavcan::equipment::safety::ArmingStatus;
	ArmingStatus msg;

	msg.status = (uint8_t)_p1;
	(void) _canPub_armStat.broadcast(msg);
	PX4_INFO("canPub_armStat yayınlandı. yayınlanan veri: %d", msg.status);

	if (_frameId > 31)
	{
		_frameId = 0;
	}

	uint8_t tailByte = _frameId | _sobalak;

	uint8_t myData[] = {33, tailByte};
	uavcan::CanFrame myCanFrame{0b10010111000001000100110000000111, myData, 2};
	uavcan::MonotonicTime myMonTime = uavcan::MonotonicTime::fromMSec(100);

	int result = _node.injectTxFrame(myCanFrame,myMonTime,(uavcan::uint8_t)3U);
	PX4_INFO("inject yapildi, result: %d, tailByte:%d", result, tailByte);
	_frameId++;
}

void TtDevice::print_status()
{
	printf("cdev'den inherit etmedigi icin burasini uygulamadim.");
	// printf("devname: %s\n", _class_devname);

	// for (unsigned i = 0; i < _max_channels; i++) {
	// 	if (_channels[i].node_id >= 0) {
	// 		printf("channel %d: node id %d --> class instance %d\n",
	// 		       i, _channels[i].node_id, _channels[i].class_instance);

	// 	} else {
	// 		printf("channel %d: empty\n", i);
	// 	}
	// }
}
