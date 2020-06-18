/**
 * @file mgm_hbc.hpp
 *
 * CAN <--> ORB bridge for mgm compro hbc series ESC messages:
 *
 * @author M.Yusuf Korkut
 */
#include <uavcan/uavcan.hpp>
#include <perf/perf_counter.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/actuator_outputs.h>


class mgm_hbc
{
public:
	mgm_hbc(uavcan::INode &node, uint8_t comAddr);
	~mgm_hbc();
	int init();

	void update_outputs(float *outputs, unsigned num_outputs);
	void arm_all_escs(bool arm);
	void arm_single_esc(int num, bool arm);
	void enable_idle_throttle_when_armed(bool value) { _run_at_idle_throttle_when_armed = value; }

	// bunlarin hepsi void, error handling icin biseyler uygulayabilir miyiz?
	void handle_message(const uavcan::CanRxFrame *frame);
	void handle_message_packet1(const uint8_t *data);
	void handle_message_packet2(const uint8_t *data);
	void handle_message_packet3(const uint8_t *data);
	void handle_message_packet4(const uint8_t *data);

	// bu fonk, tt_device icinde bir callback olarak ayarlanabilir.
	// yani actuator_controls mesaji yayinlaninca otomatik bu fonk cagrilir mesela.
	// ya da normal poll sekilde mesajin updated() olup olmadigi kontrol edilir.
	// yayinlandiysa bu fonk cagrilir.
	// YA DA
	// bu sınıfı scheduledworkitem ya da module yapabiliriz. run fonksiyonu bu olur.
	// boylelikle tt_device'tan bagimsiz oluruz.
	void send_thrl_cmd(int16_t thrl_cmd);

	uint8_t _P15_com_add; // 240, 224, 208, 192, 176, 160, 144, 128 olabilir.

private:
	/**
	 * ESC status will be published to ORB from this callback (fixed rate).
	 */
	void orb_pub_timer_cb(const uavcan::TimerEvent &event);

	bool		_armed = false;
	bool		_run_at_idle_throttle_when_armed = false;
	esc_status_s	_esc_status = {};

	uavcan::MonotonicTime _prev_cmd_pub;   ///< rate limiting
	uavcan::INode &_node;
	uavcan::TimerEventForwarder<TimerCbBinder>	_orb_timer;
	uavcan::MonotonicTime _tx_deadline;

	static constexpr unsigned MAX_RATE_HZ = 200;			///< XXX make this configurable
	static constexpr unsigned ESC_STATUS_UPDATE_RATE_HZ = 10;

	// MSG IDs
	static constexpr uint16_t CMD_ID = 0;
	static constexpr uint16_t PACKET1_ID = 1;
	static constexpr uint16_t PACKET2_ID = 2;
	static constexpr uint16_t PACKET3_ID = 3;
	static constexpr uint16_t PACKET4_ID = 4;

	// WARNINGS (bit pattern)
	static constexpr uint16_t LOW_VOLTAGE = 0x0001;
	static constexpr uint16_t HIGH_CURRENT = 0x0002;
	static constexpr uint16_t CTR_OVERHEAT = 0x0004;
	static constexpr uint16_t BAT_OVERHEAT = 0x0008;
	static constexpr uint16_t MTR_OVERHEAT = 0x0010;

	// FAILURES (bit pattern)
	static constexpr uint16_t IN_SIG_OUT = 0x0001;
	static constexpr uint16_t WAIT_THRL_IDLE = 0x0002;
	static constexpr uint16_t PWR_OFF_REQ = 0x0004;
	static constexpr uint16_t MEM_ERR = 0x0008;
	static constexpr uint16_t PWR_OFF_REQ_CHNG = 0x0010;
	static constexpr uint16_t MTR_ERR = 0x0020;
	static constexpr uint16_t INT_PWR_ERR = 0x0040;
	static constexpr uint16_t PWR_OFF_REQ_LRN = 0x0080;
	static constexpr uint16_t BAT_TEMP_SENS = 0x0100;
	static constexpr uint16_t MTR_TEMP_SENS = 0x0200;
	static constexpr uint16_t OTHER_HW_ERR = 0x0800;

	// BULLSHIT CONSTANTS
	static constexpr uint32_t BS = 0x14A10000U;


}
