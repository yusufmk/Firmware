/**
 * @file tt_device.hpp
 *
 * UAVCAN <--> ORB bridge for redundant fcu messages:
 *
 * @author M.Yusuf Korkut <mkorkut@thy.com>
 */

#pragma once

#include <px4_module_params.h>
#include <drivers/drv_hrt.h>
#include <px4_defines.h>
#include <px4_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/yusuf_message.h>
#include <uORB/topics/mavlink_log.h>

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/safety/ArmingStatus.hpp>


class RxFrameListener : public uavcan::IRxFrameListener
{
public:
	~RxFrameListener()
	{

	}

	void handleRxFrame(const uavcan::CanRxFrame &frame,
			   uavcan::CanIOFlags flags) override
	{
		if (_prevTimestamp != 0)
		{
			_diff = hrt_elapsed_time(&_prevTimestamp);
			_minIval = _diff < _minIval ? _diff : _minIval;
			_maxIval = _diff > _maxIval ? _diff : _maxIval;
		}
		_prevTimestamp = hrt_absolute_time();

		uint8_t srcId = frame.id & 127;
		// uint8_t msgOrService = (frame.id >> 7) & 1;
		// uint16_t msgId;
		// uint16_t disc = 0;
		// uint8_t prio = (frame.id >> 24) & 31;
		// uint8_t other = (frame.id >> 29) & 7;
		// if (srcId == 0)
		// {
		// 	msgId = (frame.id >> 8) & 3;
		// 	disc = (frame.id >> 10) & 16383;
		// }
		// else
		// {
		// 	msgId = (frame.id >> 8) & 65535;
		// }

		if (srcId == 0) {
			_anonymRcvCnt++;

		} else if (srcId == 125) {
			_gpsRcvCnt++;
			if (_lastGpsTime != 0)
			{
				_diff = hrt_elapsed_time(&_lastGpsTime);
				_gpsMaxIval = _diff > _gpsMaxIval ? _diff : _gpsMaxIval;
				_gpsMinIval = _diff < _gpsMinIval ? _diff : _gpsMinIval;
			}
			_lastGpsTime = hrt_absolute_time();

		} else if (srcId == _node_id) {
			_sendCnt++;

		} else {
			_redRcvCnt++;
			if (_lastRedTime != 0)
			{
				_diff = hrt_elapsed_time(&_lastRedTime);
				_redMaxIval = _diff > _redMaxIval ? _diff : _redMaxIval;
				_redMinIval = _diff < _redMinIval ? _diff : _redMinIval;
			}
			_lastRedTime = hrt_absolute_time();
		}

		// PX4_INFO("frameId: %X, other:%d, prio:%3d, disc:%5d, msgId:%4d  msgOrServ:%d  srcId:%3d  dlc:%d, d[0]:%2X, d[1]:%2X, d[2]:%2X, d[3]:%2X, d[4]:%2X, d[5]:%2X, d[6]:%2X, d[7]:%2X",
		// 	frame.id, other, prio, disc, msgId, msgOrService, srcId, frame.dlc, frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5],
		// 	frame.data[6], frame.data[7]);
	}
	uint32_t 	_node_id;
	uint64_t 	_sendCnt{0};
	uint64_t	_redRcvCnt{0};
	uint64_t	_gpsRcvCnt{0};
	uint64_t	_anonymRcvCnt{0};

	hrt_abstime	_lastRedTime{0};
	hrt_abstime	_redMinIval{9999999};
	hrt_abstime	_redMaxIval{0};

	hrt_abstime	_lastGpsTime{0};
	hrt_abstime	_gpsMinIval{9999999};
	hrt_abstime	_gpsMaxIval{0};

	hrt_abstime	_currTimestamp{0};
	hrt_abstime	_prevTimestamp{0};
	hrt_abstime	_minIval{9999999};
	hrt_abstime	_maxIval{0};

	hrt_abstime	_diff{0};
};

class TtDevice
{
	static constexpr unsigned ORB_TO_UAVCAN_FREQUENCY_HZ = 1;

public:
	static const char *const NAME;

	TtDevice(uavcan::INode &node);
	~TtDevice();

	const char *get_name() { return NAME; }

	int init();

	void print_status();

private:
	/**
	 * armStatus message will be reported via this callback.
	 */
	void armStat_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::safety::ArmingStatus> &msg);

	void broadcast_from_param(const uavcan::TimerEvent &);



	typedef uavcan::MethodBinder <TtDevice *,
		void (TtDevice::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::safety::ArmingStatus> &)>
		ArmStatCbBinder;

	typedef uavcan::MethodBinder<TtDevice *,
		void (TtDevice::*)(const uavcan::TimerEvent &)>
		TimerCbBinder;

	/*
	 * libuavcan related things
	 */
	uavcan::INode &_node;
	uavcan::Subscriber<uavcan::equipment::safety::ArmingStatus, ArmStatCbBinder> _canSub_armStat;
	uavcan::Publisher<uavcan::equipment::safety::ArmingStatus> _canPub_armStat;
	// uavcan::TimerEventForwarder<TimerCbBinder> _orb_to_uavcan_pub_timer;
	uavcan::TimerEventForwarder<TimerCbBinder> _param_to_uavcan_pub_timer;


	orb_advert_t _yusuf_message_pub;                ///< uORB pub for yusuf_message
	int _yusuf_message_sub = -1;
	struct yusuf_message_s	_benim_mesaj;

	param_t _sys_id_handle;
	int32_t _sys_id;

	// DEFINE_PARAMETERS(
	// 	(ParamInt<px4::params::UAVCAN_NODE_ID>) _param_node_id,
	// )


	param_t _p1_handle;
	int32_t _p1;

	param_t _p2_handle;
	int32_t _p2;

	RxFrameListener _myFrameListener;
	uint32_t _frameId;
	// uint8_t _sobalak;

	orb_advert_t _mavlink_log_pub{nullptr};

	uint32_t 	_node_id;
	hrt_abstime		_initTime;
	uint64_t 	_sendCnt{0};
};


