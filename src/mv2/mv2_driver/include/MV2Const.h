/**
 * @file	MV2Const.h
 * @brief	MV2ライブラリの変数定義
 * @date	2013/03/1
 * @author	sekiguchi
 * @par		Copyright:
 *		2013 ZMP Inc. All rights reserved.
 */

#ifndef MV2CONST_H_
#define MV2CONST_H_

#include <stdio.h>
namespace zmp
{
	namespace mv2 
	{
/**
 * @brief ドライブ状態通知Index
 */
enum REP_DRIVE_INFO_INDEX
{
	REP_DRV_MODE,			/*!< モード */
	REP_DRV_PEDAL,			/*!< アクセルペダル */
	REP_DRV_VELOCITY,		/*!< 速度 */
	REP_DRV_WHEEL_VELOCITY,		/*!< 4輪速度 */
	REP_BRAKE_PEDAL,		/*!< ブレーキペダル */
	REP_DRV_SHIFT_POS,		/*!< シフトポジション */
	REP_BRAKE_STATUS,
};

/**
 * @brief ステアリング状態通知Index
 */
enum REP_STEER_INFO_INDEX
{
	REP_STR_MODE,		/*!< モード変更 */
	REP_STR_TORQUE,		/*!< トルク */
	REP_STR_ANGLE,		/*!< 角度 */
};

/**
 * @brief バッテリ状態通知Index
 */
enum REP_BATT_INFO_INDEX
{
	REP_BATT_MAIN_CURRENT,	/*!< Mainバッテリ電流 */
	REP_BATT_SUB_CURRENT,	/*!< Subバッテリ電流 */
	REP_BATT_MAIN_VOLTAGE,	/*!< Mainバッテリ電圧 */
	REP_BATT_SUB_VOLTAGE,	/*!< Subバッテリ電圧 */
};

/**
 * @brief 他状態通知Index
 */
enum REP_OTHER_INFO_INDEX
{
	RES_SYSCOM_ECHO,	/*!< エコー応答 */ 
	RES_SYSCOM_ERR_STATE,	/*!< エラー状態 */
	REP_SYSCOM_SWITCH_STATE,/*!< スイッチ状態 */
	REP_SYSCOM_ADC_STATUS,
	REP_SYSCOM_WATCHDOG_STATUS,
};

/**
 * @brief 車両情報通知Index
 */
enum REP_VEHICLE_INFO_INDEX
{
    REP_VEHICLE_BATT_SOC,	/*!< 車両バッテリSOC */
    REP_VEHICLE_VELOC,		/*!< 車両速度 */
    REP_VEHICLE_BATT_TEMP,	/*!< 車両バッテリ温度 */
    REP_VEHICLE_METER_VELOC,	/*!< 車両メータ速度 */
    REP_VEHICLE_BATT_LEVEL,	/*!< 車両バッテリレベル */
    REP_VEHICLE_ODOMETER,	/*!< 車両Odometer */
    REP_VEHICLE_TRIP,		/*!< 車両Trip */
};

/**
 * @brief IMU-Z2情報通知Index
 */
enum REP_IMUZ2_INFO_INDEX
{
    REP_IMUZ2_ACC_DATA,	/*!< 加速度データ */
    REP_IMUZ2_GYRO_DATA,/*!< 角速度データ */
    REP_IMUZ2_COMP_DATA,/*!< 地磁気データ */
    RES_IMUZ2_STATE,	/*!< IMU-Z2状態 */	
    RES_IMUZ2_PROFILE,	/*!< デバイスプロファイル */
};

/**
 * @brief Position-Z情報通知Index
 */
enum REP_POSZ_INFO_INDEX
{
    RES_POSZ_STATE,	/*!< Position-Z状態 */
    RES_POSZ_PROFILE,	/*!< デバイスプロファイル */
    REP_POSZ_GPGGA,	/*!< GPS GPGGAデータ */
    REP_POSZ_GPZDA,	/*!< GPS GPZDAデータ */
    REP_POSZ_GPGLL,	/*!< GPS GPGLLデータ */
    REP_POSZ_GPGSA,	/*!< GPS GPGSAデータ */
    REP_POSZ_GPGSV,	/*!< GPS GPGSVデータ */
    REP_POSZ_GPVTG,	/*!< GPS GPVTGデータ */
    REP_POSZ_GPRMC,	/*!< GPS GPRMCデータ */
    REP_POSZ_PRESSURE,	/*!< 気圧データ */
    REP_POSZ_HUMIDTY,	/*!< 湿度データ */
};

enum REP_INCZ_INFO_INDEX
{
    REP_INCZ_STATE,
    REP_INCZ_ANGLE,
    REP_INCZ_ACC,
};

/**
 * @brief 加速度範囲
 */
enum RANGE_ACC
{
    RANGE_ACC_2,	/*!< -2〜2[G] */
    RANGE_ACC_4,	/*!< -4〜4[G] */
    RANGE_ACC_8,	/*!< -8〜8[G] */
};

/**
 * @brief 角速度範囲
 */
enum RANGE_GYRO
{
    RANGE_GYRO_500,	/*!< -500〜500[deg/sec] */
    RANGE_GYRO_2000,	/*!< -2000〜2000[deg/sec] */
};

/**
 * @brief 地磁気範囲
 */
enum RANGE_COMP
{
    RANGE_COMP_07,	/*!< -0.7〜0.7[Ga] */
    RANGE_COMP_10,	/*!< -1.0〜1.0[Ga] */
    RANGE_COMP_15,	/*!< -1.5〜1.5[Ga] */
    RANGE_COMP_20,	/*!< -2.0〜2.0[Ga] */
    RANGE_COMP_32,	/*!< -3.2〜3.2[Ga] */
    RANGE_COMP_38,	/*!< -3.8〜3.8[Ga] */
    RANGE_COMP_45,	/*!< -4.5〜4.5[Ga] */
};

/**
 * @brief IMU-Z2 ロール
 */
enum IMUZ2_ROLE
{
    ROLE_SIGLE_BT,	/*!< シングルBluetooth */
    ROLE_CANMASTER_BT,	/*!< CANマスタ(Bluetooth接続あり) */
    ROLE_CANSLAVE,	/*!< CANスレーブ */
};

/**
 * @brief IMU-Z2 フォーマット
 */
enum IMUZ2_FORMAT
{
    FORMAT_TEXT,	/*!< テキスト */
    FORMAT_BINARY,	/*!< バイナリ */
    FORMAT_EXTEND,	/*!< 拡張フォーマット */
};

/**
 * @brief シフトポジション
 */
enum SHIFT_POSITION
{
	SHIFT_POS_D = 0x10,	/*!< シフトポジション D */
	SHIFT_POS_N = 0x20,	/*!< シフトポジション N */
	SHIFT_POS_R = 0x40,	/*!< シフトポジション R */
	SHIFT_POS_U = 0xff,	/*!< シフトポジション unknown */
};

/**
 * @brief サーボ 
 */
enum MV2_SERVO
{
	SERVO_ON	= 0x00,	/*!< サーボON */
	SERVO_OFF	= 0x10	/*!< サーボOFF */
};

/**
 * @brief モード
 */
enum MV2_MODE
{
	MODE_MANUAL	= 0x00,	/*!< マニュアルモード */
	MODE_PROGRAM	= 0x10	/*!< プログラムモード */
};

/**
 * @brief ステアリングコントロールモード
 */
enum STEER_CONTROL_MODE
{
	CONT_MODE_ANGLE	 = 0x00,	/*!< 角度制御モード */
	CONT_MODE_TORQUE = 0x10,	/*!< トルク制御モード */
};

/**
 * @brief ドライブコントロールモード
 */
enum DRIVE_CONTROL_MODE
{
	CONT_MODE_STROKE	= 0x00,	/*!< ペダル制御モード */
	CONT_MODE_VELOCITY	= 0x10	/*!< 速度制御モード */
};

/**
 * @brief オーバーライドモード
 */
enum OVERRIDE_MODE
{
	OVERRIDE_MODE_ON	= 0x00,	/*!< オーバーライドモードON */
	OVERRIDE_MODE_OFF	= 0x10	/*!< オーバーライドモードOFF */
};

/**
 * @brief ヘッドライトの点灯状態
 */
enum LIGHT_STATE
{
	LIGHT_OFF	= 0x00,	/*!< ライトOFF */
	LIGHT_PARK	= 0x10,	/*!< スモールライト点灯 */
	LIGHT_ON	= 0x30,	/*!< ライトON */
	LIGHT_HIGH	= 0x38	/*!< ハイビーム点灯 */
};

/**
 * @brief Config種別
 */
enum MV2_CONFIG
{
    CONFIG_STEER_KP		= 100,
    CONFIG_STEER_KI		= 101,
    CONFIG_STEER_KD		= 102,
    CONFIG_STEER_DEC_TIME	= 103,
    CONFIG_STEER_WINDOW		= 104,
    CONFIG_STEER_PUNCH		= 105,
    CONFIG_STEER_OUTPUT_MAX	= 106,
    CONFIG_STEER_CCW_FACTOR	= 107,
    CONFIG_STEER_MAX_VELOC	= 108,
    CONFIG_STEER_MAX_ACC	= 109,
    CONFIG_STEER_ENCODER_OFFSET = 110,
    CONFIG_STEER_STROKE_LIMIT_P	= 111,
    CONFIG_STEER_STROKE_LIMIT_M = 112,

    CONFIG_VELOC_MAX_SPEED	= 113,
    CONFIG_VELOC_KP_P		= 114,
    CONFIG_VELOC_KP_M		= 115,
    CONFIG_VELOC_WINDOW_P	= 116,
    CONFIG_VELOC_WINDOW_M	= 117,
    CONFIG_VELOC_PUNCH_P	= 118,
    CONFIG_VELOC_PUNCH_M	= 119,
    CONFIG_VELOC_OUTPUT_MAX_P	= 120,
    CONFIG_VELOC_OUTPUT_MAX_M	= 121,

    CONFIG_BRAKE_MOTION_MAX_VELOC = 122,
    CONFIG_BRAKE_MOTION_MAX_ACC   = 123,
    CONFIG_BRAKE_ENCODER_OFFSET   = 124,
    CONFIG_BRAKE_STROKE_LIMIT_P   = 125,
    CONFIG_BRAKE_STROKE_LIMIT_M   = 126,

    CONFIG_OVERRIDE_STEER_TH	= 127,
    CONFIG_OVERRIDE_ACCEL_TH	= 128,
    CONFIG_OVERRIDE_BRAKE_TH	= 129,
    TEST_FLAGS			= 130,
	CONFIG_REPORT_FREQ		= 131,
	CONFIG_BRAKE_POSITION_MIN	= 132,
	CONFIG_BRAKE_POSITION_MAX	= 133,
	CONFIG_STEER_OVERRIDE_STATUS = 134,
	CONFIG_BRAKE_OVERRIDE_STATUS = 135,
	CONFIG_CHK_CLIENT_CONNECTION = 136,
};

enum GPS_KIND
{
    GP_GGA,
    GP_ZDA,
    GP_GLL,
    GP_GSA,
    GP_GSV,
    GP_VTG,
    GP_RMC,
};

struct UTC_TIME {
    unsigned char h;
    unsigned char m;
    unsigned char s;
    unsigned short ms;
};

struct UTC_DATE{
    unsigned char day;
    unsigned char month;
    unsigned short year;
};

struct GPGGA_DATA{
    UTC_TIME utcTime;
    unsigned int latitude;
    unsigned int longitude;
    bool latitudeSign;
    bool longitudeSign;
    unsigned char numSatellite;
    unsigned char qualityIndicator;
    unsigned int hdop;
    unsigned int seaLevelAttitude;
    bool seaLevelAttitudeSign;
    unsigned int geoLevelAttitude;
    bool geoLevelAttitudeSign;
    unsigned int ageOfDgps;
    int dgpsStationID;
};

struct GPZDA_DATA{
    UTC_TIME utcTime;
    unsigned char day;
    unsigned char month;
    unsigned int year;
    unsigned char timeZoneH;
    unsigned char timeZoneM;
    bool timeZoneSign;
};

struct GPGLL_DATA{
    float latitude;
    float longitude;
    bool latitudeSign;
    bool longitudeSign;
    UTC_TIME utcTime;
    bool receiveStatus;
    unsigned char qualityIndicator;
};

struct GPGSA_DATA{
    unsigned char messMode;
    unsigned char qualityIndicator;
    unsigned char satteliteNo[12];
    float pdop;
    float hdop;
    float vdop;
};

struct GSV_SATTELITE_DATA{
    unsigned char satteliteNo;
    float elevation;
    float azimuth;
    float snr;
};

struct GPGSV_DATA{
    unsigned char numOfMsg;
    unsigned char msgNo;
    unsigned char numOfSatellite;
    GSV_SATTELITE_DATA sattelite[4];
};

struct GPVTG_DATA{
    float dirOfTrueNorth;
    float dirOfMagneticNorth;
    float speedKnot;
    float speedKmph;
    unsigned char qualityIndicator;
};

struct GPRMC_DATA{
    UTC_TIME utcTime;
    float latitude;
    float longitude;
    bool latitudeSign;
    bool longitudeSign;
    float speedKnot;
    float dirOfTrueNorthR;
    UTC_DATE utcDate;
    bool receiveStatus;
    unsigned char qualityIndicator;
    float magneticDeviation;
    bool magneticDeviationSign;
};

struct MILLIWAVE_DATA {
    unsigned short angle;
    unsigned short distance;
    unsigned short rangeRate;
};

	}
}

#endif /* MV2CONST_H_ */
