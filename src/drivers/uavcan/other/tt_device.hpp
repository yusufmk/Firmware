/**
 * @file tt_device.hpp
 *
 * UAVCAN <--> ORB bridge for redundant fcu messages:
 *
 * @author M.Yusuf Korkut <mkorkut@thy.com>
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/yusuf_message.h>

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/safety/ArmingStatus.hpp>

class TtDevice
{
	static constexpr unsigned ORB_TO_UAVCAN_FREQUENCY_HZ = 10;

public:
	static const char *const NAME;

	TtDevice(uavcan::INode &node);
	~TtDevice();

	const char *get_name() { return NAME; }

	int init();

	// void print_status();

private:
	/**
	 * armStatus message will be reported via this callback.
	 */
	void armStat_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::safety::ArmingStatus> &msg);

	void broadcast_from_orb(const uavcan::TimerEvent &);



	typedef uavcan::MethodBinder <TtDevice *,
		void (TtDevice::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::safety::ArmingStatus> &)>
		ArmStatCbBinder;

	typedef uavcan::MethodBinder<TtDevice *,
		void (TtDevice::*)(const uavcan::TimerEvent &)>
		TimerCbBinder;

	uavcan::INode &_node;
	uavcan::Subscriber<uavcan::equipment::safety::ArmingStatus, ArmStatCbBinder> _canSub_armStat;
	uavcan::Publisher<uavcan::equipment::safety::ArmingStatus> _canPub_armStat;
	uavcan::TimerEventForwarder<TimerCbBinder> _orb_to_uavcan_pub_timer;

	orb_advert_t _yusuf_message_pub;                ///< uORB pub for yusuf_message
	int _yusuf_message_sub = -1;
	struct yusuf_message_s	_benim_mesaj;
};
