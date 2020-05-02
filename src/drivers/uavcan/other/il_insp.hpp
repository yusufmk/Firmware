#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/airspeed.h>

#include <uORB/topics/estimator_status.h>

#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_combined.h>

#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_magnetometer.h>

#include <uORB/topics/vehicle_air_data.h>

#include <uavcan/uavcan.hpp>

// TODO !!!! sensors'u mu besleyecem yoksa ekf2'yi mi karar vermem gerek !!!!!
// sensors -> voted_sensors_update::sensors_poll()
// src -> lib -> ecl -> validation -> data_validator_group::put()


// CDEV sınıflarını kullanmam gerekebilir. PX4Accelerometer, PX4Gyroscope turunden objeler olusturup onların degerlerini guncelleyip
// o objelerin sensor_accel topiclerini publish etmeyi saglayabilirim. bu sayede data validator isin icine dahil olmus oluyor.

// aynı şekilde gps driver'ını da kullanabilirim.

// ya da uavnca/sensors/gnss.hpp deki kodları kullanabilirim.

class il_insp
{
private:
	/* data */
	// TODO!!! BUNLARI ENUM YAPACAZ GALIBA
	// CANBUS MESSAGE OFFSETS - ICD p.96 table 6.40
	static constexpr uint8_t MES_OFF_ANGULAR_RATES = 0X0;
	static constexpr uint8_t MES_OFF_ACCELERATIONS = 0x1;
	static constexpr uint8_t MES_OFF_MAGNETIC_FIELD = 0x2;
	static constexpr uint8_t MES_OFF_ORIENTATION = 0x3;
	static constexpr uint8_t MES_OFF_EAST_VELOCITY = 0x4;
	static constexpr uint8_t MES_OFF_NORTH_VELOCITY = 0x5;
	static constexpr uint8_t MES_OFF_VERTICAL_VELOCITY = 0x6;
	static constexpr uint8_t MES_OFF_LONGITUDE = 0x7;
	static constexpr uint8_t MES_OFF_LATITUDE = 0x8;
	static constexpr uint8_t MES_OFF_ALTITUDE = 0x9;
	static constexpr uint8_t MES_OFF_TIME_INFO = 0xA;
	static constexpr uint8_t MES_OFF_INS_ACCURACY = 0xB;
	static constexpr uint8_t MES_OFF_GNSS_INFO = 0xC;
	// TODO - CUSTOM MESAJ GONDERILEBILIYOR, KULLANILABILECEK VERILERE BAK

	// GNSS SOLUTION STATUS - ICD p.68 table 6.16
	static constexpr uint8_t SOL_COMPUTED = 0;
	static constexpr uint8_t INSUF_OBS = 1;
	static constexpr uint8_t NO_CONV = 2;
	static constexpr uint8_t SING_AT_PARAM_MAT = 3;
	static constexpr uint8_t COV_TRACE_EXC_MAX = 4;
	static constexpr uint8_t TEST_DIST_EXC = 5;
	static constexpr uint8_t NOT_CONV_COLD_START = 6;
	static constexpr uint8_t H_V_LIM_EXC = 7;
	static constexpr uint8_t VAR_LIM_EXC = 8;
	static constexpr uint8_t RES_TOO_LARGE = 9;
	static constexpr uint8_t POS_UNREL = 13;
	static constexpr uint8_t FIX_POS_VALID = 18;
	static constexpr uint8_t FIX_POS_NOT_VALID = 19;
	static constexpr uint8_t POS_TYPE_UNAUTH = 20;

	// GNSS POSITION OR VELOCITY TYPE - ICD p.68 table 6.17
	static constexpr uint8_t NO_SOL = 0;
	static constexpr uint8_t INST_DOP = 8;
	static constexpr uint8_t SINGLE_PNT = 16;
	static constexpr uint8_t PSEUDO_DIFF_SOL = 17;
	static constexpr uint8_t WAAS = 18;
	static constexpr uint8_t PROP_WO_OBS = 19;
	static constexpr uint8_t FLT_L1_AMBG = 32;
	static constexpr uint8_t FLT_IONO_FREE_AMBG = 33;
	static constexpr uint8_t FLT_NAR_LANE_AMBG = 34;
	static constexpr uint8_t INT_L1_AMBG = 48;
	static constexpr uint8_t INT_NAR_LANE_AMBG = 50;

	// GPS REFERENCE TIME STATUS - ICD p.73 table 6.24
	static constexpr uint8_t UNKNOWN = 20;
	static constexpr uint8_t APPROX = 60;
	static constexpr uint8_t APPR_COARSE = 80;
	static constexpr uint8_t VALID_COARSE = 100;
	static constexpr uint8_t COARSE_SET = 120;
	static constexpr uint8_t POS_LOST = 130;
	static constexpr uint8_t ADJ_FINE = 140;
	static constexpr uint8_t HAS_FINE = 160;
	static constexpr uint8_t FINE_SET_BACKUP = 170;
	static constexpr uint8_t FINE_SET = 180;

	// INS SOLUTION STATUS - ICS p.173 table 6.97
	static constexpr uint8_t SOL_GOOD = 0;
	static constexpr uint8_t SOL_SATIS = 1;
	static constexpr uint8_t GNSS_ABSENT = 2;
	static constexpr uint8_t HEADING_NOT_STARTED = 3;
	static constexpr uint8_t NO_AID = 4;
	static constexpr uint8_t EXT_AID = 5;
	static constexpr uint8_t AUTO_TIMEOUT = 6;
	static constexpr uint8_t ZUPT = 7;
	static constexpr uint8_t SOL_INVALID = 8;

public:
	il_insp(/* args */);
	~il_insp();
	void handle_message(const uavcan::CanRxFrame* frame);
	void handle_message_angular_rates(const uint8_t* data);
	void handle_message_accelerations(const uint8_t* data);
	void handle_message_magnetic_field(const uint8_t* data);
	void handle_message_orientation(const uint8_t* data);
	void handle_message_east_velocity(const uint8_t* data);
	void handle_message_north_velocity(const uint8_t* data);
	void handle_message_vertical_velocity(const uint8_t* data);
	void handle_message_longitude(const uint8_t* data);
	void handle_message_latitude(const uint8_t* data);
	void handle_message_altitude(const uint8_t* data);
	void handle_message_time_info(const uint8_t* data);
	void handle_message_ins_accuracy(const uint8_t* data);
	void handle_message_gnss_info(const uint8_t* data);
};


