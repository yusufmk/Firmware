#include "mgm_hbc.hpp"

mgm_hbc::mgm_hbc(uavcan::INode &node, uint8_t comAddr) :
	_tx_deadline(uavcan::MonotonicTime::fromMSec(100)),
	_P15_com_add(comAddr),
	_node(node)
{
}

mgm_hbc::~mgm_hbc()
{

}

mgm_hbc::init()
{
	// ESC status will be relayed from UAVCAN bus into ORB at this rate
	_orb_timer.setCallback(TimerCbBinder(this, &mgm_hbc::orb_pub_timer_cb));
	_orb_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / ESC_STATUS_UPDATE_RATE_HZ));
}

void mgm_hbc::handle_message(const uavcan::CanRxFrame *frame)
{
	switch (frame & (uint32_t)7) {
	case PACKET1_ID:
		handle_message_packet1(frame.data);
		break;

	case PACKET2_ID:
		handle_message_packet2(frame.data);
		break;

	case PACKET3_ID:
		handle_message_packet3(frame.data);
		break;

	case PACKET4_ID:
		handle_message_packet4(frame.data);
		break;

	default:
		// TODO: error handling
		break;
	}
}

void mgm_hbc::handle_message_packet1(const uint8_t *data)
{
	float bat_v = (data[0] + 256 * data[1]) / 57.45f;
	float bat_i = (data[2] + 256 * data[3]) / 10.0f;
	uint32_t mtr_rpm = (data[4] + 256 * data[5] + 65536 * data[6]) * 10;
	// TODO: publish values
	// failsafe'te kullanılmalı
}

void mgm_hbc::handle_message_packet2(const uint8_t *data)
{
	uint32_t odometer = data[0] + 256 * data[1] + 65536 * data[2] + 16777216 * data[3];
	uint8_t esc_temp = data[4];
	uint8_t mtr_temp = data[5];
	uint8_t bat_temp = data[6];
	// TODO: publish values
	// failsafe'te kullanilmali
}

void mgm_hbc::handle_message_packet3(const uint8_t *data)
{
	float req_pwm = (data[0] + 256 * data[1]) / 10.0f;
	float real_pwm = (data[2] + 256 * data[3]) / 10.0f;
	uint16_t warnings = data[4] + 256 * data[5];
	uint16_t failures = data[6] + 256 * data[7];
	// TODO: publish values
	// commander'da, navigator'da, failsafe'te kesin kullanılmalı
}

void mgm_hbc::handle_message_packet4(const uint8_t *data)
{
	uint16_t bat_lvl = data[0] + 256 * data[1];
	// TODO: publish values
}

void UavcanEscController::orb_pub_timer_cb(const uavcan::TimerEvent &)
{
	_esc_status.counter += 1;
	_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
	int instance = null;

	orb_publish_auto(ORB_ID(esc_status), &_esc_status_pub, &_esc_status, &instance, ORB_PRIO_DEFAULT);


	// if (_esc_status_pub != nullptr) {
	// 	(void)orb_publish(ORB_ID(esc_status), _esc_status_pub, &_esc_status);

	// } else {
	// 	_esc_status_pub = orb_advertise(ORB_ID(esc_status), &_esc_status);
	// }
}

void mgm_hbc::send_thrl_cmd(int16_t thrl_cmd)
{
	thrl_cmd = thrl_cmd > 1023 ? 1023 : thrl_cmd;
	thrl_cmd = thrl_cmd < -1024 ? -1024 : thrl_cmd;

	uint32_t frameId = (BS | (1U << 31)) + _P15_com_add + CMD_ID;
	uint8_t data[] = {thrl_cmd & 0xff, thrl_cmd >> 8 };

	uavcan::CanFrame myCanFrame{frameId, data, 2};
	int result = _node.injectTxFrame(myCanFrame, _tx_deadline, 3U);
}
