/**
 * @file		CANConst.h
 * @brief		CANI/Fライブラリの変数定義  CANI/F程序库的变量定义
 *
 * @date		2013/07/25
 * @author		k.sekiguchi
 * @par			Copyright:
 *				2013 XXX All rights reserved.
 */

#ifndef CANCONST_H_
#define CANCONST_H_

#define DEVICE_CAN0	"/dev/pcan32"
#define DEVICE_CAN1	"/dev/pcan33"

/**
 * @brief CANメッセージ構造体 CAN信息结构体
 */
typedef struct
{
	int	ID;		/*!< CAN ID */
	unsigned char  LEN;	/*!< DATA length */
	unsigned char  DATA[8];	/*!< DATA */
} CANMsg; 


/**
 * @brief CANメッセージIDのメッセージグループ(上位5bit) 记录CAN信息ID的信息组
 */
enum CAN_MSG_GROUP
{
	MSG_GROUP_PRIORITY	= 0x00,	/*!< Priorityメッセージグループ */
	MSG_GROUP_DRIVE		= 0x01,	/*!< ドライブメッセージグループ */
	MSG_GROUP_STEER		= 0x02,	/*!< ステアリングメッセージグループ */
	MSG_GROUP_BREAK		= 0x03,	/*!< ブレーキメッセージグループ */
	MSG_GROUP_BATTERY	= 0x04, /*!< バッテリーメッセージグループ */
	MSG_GROUP_OPTION	= 0x05, 
 	MSG_GROUP_IMUZ2		= 0x08,
	MSG_GROUP_POSZ		= 0x0a,
	MSG_GROUP_INCLINO	= 0x0b,
	MSG_GROUP_SYSCOM	= 0x18,	/*!< システム共通メッセージグループ */
};

/**
 * @brief CAN PRIORITYメッセージ
 */
enum CAN_MSG_PRIORITY
{
	MSG_PRIORITY_ERROR	= 0x01,	/*!< エラーメッセージ */
};

/**
 * @brief CAN DRIVEメッセージ
 */
enum CAN_MSG_DRIVE
{
	MSG_DRIVE_SET_MODE	= 0x01,	/*!< モード切替メッセージ */
	MSG_DRIVE_SET_CONTROL_MODE = 0x02,/*!< 制御モード切替メッセージ */
//	MSG_DRIVE_SET_OVERRIDE_MODE = 0x03,/*!< オーバライド切替メッセージ */
	MSG_DRIVE_SET_SERVO	= 0x04,	/*!< サーボONOFFメッセージ */
	MSG_DRIVE_SET_VELOCITY	= 0x05,	/*!< ドライブ速度指示メッセージ */
	MSG_DRIVE_SET_PEDAL	= 0x06,	/*!< ドライブペダル指示メッセージ */
	MSG_DRIVE_SET_SHIFT_POS	= 0x07,	/*!< シフト切替指示メッセージ */
	MSG_DRIVE_REP_MODE	= 0x21,	/*!< 制御モード通知メッセージ */
	MSG_DRIVE_REP_PEDAL	= 0x22,	/*!< ペダル情報通知メッセージ */
	MSG_DRIVE_REP_SHIFT_POS	= 0x23,	/*!< シフト情報通知メッセージ */
	MSG_DRIVE_REP_VELOCITY	= 0x24,	/*!< 速度情報通知メッセージ */
//	MSG_DRIVE_REP_WHEEL_VELOC = 0x25, /*!< ホイール速度通知メッセージ */
};

/**
 * @brief CAN STEERINGメッセージ
 */
enum CAN_MSG_STEER
{
	MSG_STEER_SET_MODE	= 0x01,	/*!< モード切替メッセージ */
	MSG_STEER_SET_CONTROL_MODE= 0x02,/*!< 制御モード切替メッセージ */
	MSG_STEER_SET_OVERRIDE_MODE= 0x03,/*!< オーバライドモード切替メッセージ */
	MSG_STEER_SET_SERVO	= 0x04,	/*!< サーボONOFメッセージF */
	MSG_STEER_SET_TORQUE	= 0x05,	/*!< トルク指示メッセージ */
	MSG_STEER_SET_ANGLE	= 0x06,	/*!< 角度指示メッセージ */
	MSG_STEER_REP_MODE	= 0x21,	/*!< 制御モード通知メッセージ */
	MSG_STEER_REP_ANGLE	= 0x22,	/*!< 角度情報通知メッセージ */
	MSG_STEER_REP_TORQUE	= 0x23,	/*!< トルク情報通知メッセージ */
//	MSG_STEER_REP_TORQUE_RAW= 0x24	/*!< トルクセンサ情報通知メッセージ */
};

/**
 * @brief CAN BRAKEメッセージ
 */
enum CAN_MSG_BRAKE
{
	MSG_BRAKE_SET_PEDAL	= 0x01,	/*!< ペダル指示メッセージ */

//	MSG_BRAKE_SET_SRCM	= 0x03,
//	MSG_BRAKE_SET_SRC	= 0x04,
//	MSG_BRAKE_SET_SLAR	= 0x05,
//	MSG_BRAKE_SET_SLAD	= 0x06,
//	MSG_BRAKE_SET_SLAVAL	= 0x07,
	MSG_BRAKE_SET_TAIL_LAMP	= 0x08,
	MSG_BRAKE_SET_BLINKER_LEFT  = 0x09,
	MSG_BRAKE_SET_BLINKER_RIGHT = 0x0a,
//	MSG_BRAKE_SET_AUTOLAMP	= 0x0b,
	MSG_BRAKE_SET_CAL	= 0x0c,

	MSG_BRAKE_REP_PEDAL	= 0x21,	/*!< ペダル情報通知メッセージ */
//	MSG_BRAKE_REP_PEDAL_RAW	= 0x22,	/*!< ペダルセンサ情報通知メッセージ */
//	MSG_BRAKE_REP_RAW	= 0x23,	/*!< ブレーキセンサ情報通知メッセージ */
//	MSG_BRAKE_REP_SRC	= 0x24,
//	MSG_BRAKE_REP_SLA	= 0x25,
//	MSG_BRAKE_REP_STATUS	= 0x26,
};

/**
 * @brief CAN BATTERYメッセージ
 */
enum CAN_MSG_BATTERY
{
//    MSG_BATTERY_REP_SOC		= 0x21, /*!< SOC通知メッセージ */
//    MSG_BATTERY_REP_STATUS	= 0x22, /*!< 充電状態通知メッセージ */
//    MSG_BATTERY_REP_CHG_CURRENT	= 0x23, /*!< 充電電流通知メッセージ */
//    MSG_BATTERY_REP_TEMP	= 0x24, /*!< バッテリ温度通知メッセージ */
//    MSG_BATTERY_REP_LEVEL	= 0x25, /*!< 充電レベル通知メッセージ */
    MSG_BATTERY_REP_MAIN_CURRENT= 0x26, /*!< Mainバッテリ電流通知メッセージ */
    MSG_BATTERY_REP_SUB_CURRENT	= 0x27, /*!< Subバッテリ電流通知メッセージ */
    MSG_BATTERY_REP_MAIN_VOLTAGE= 0x28, /*!< Mainバッテリ電圧通知メッセージ */
    MSG_BATTERY_REP_SUB_VOLTAGE	= 0x29, /*!< Subバッテリ電圧通知メッセージ */
};


enum CAN_MSG_OPTION
{
    MSG_OPTION_REP_WHEEL_ENC = 0x21,
};


/**
 * @brief IMU-Zメッセージ
 */
enum IMU_MSG_ID {
    IMU_REQ_ECHO        = 0x01, /*!< Echo要求 */
    IMU_REQ_STATUS      = 0x05,
    IMU_SET_PERIOD      = 0x07,
    IMU_SET_RANGE       = 0x08,
    IMU_RESET_TIMESTAMP = 0x0b,
    IMU_SET_MEASUREMENT_STATE = 0x0c,
    IMU_REQ_DEVICE_PROFILE = 0x0d,
    IMU_RES_ECHO        = 0x21,
    IMU_REP_MEASUREMENT_ACC = 0x22,
    IMU_REP_MEASUREMENT_GYRO = 0x23,
    IMU_REP_MEASUREMENT_COMP = 0x24,
    IMU_RES_STATUS      = 0x25,
    IMU_RES_DEVICE_PROFILE = 0x2d,
};

/**
 * @brief IMU 加速度レンジ種別 
 */
enum IMU_RANGE_ACC {
    RANGE_ACC_2G = 0,
    RANGE_ACC_4G,
    RANGE_ACC_8G,
};

/**
 * @brief IMU ジャイロレンジ種別
 */
enum IMU_RANGE_GYRO {
    RANGE_GYRO_30DPS = 0,
    RANGE_GYRO_120DPS,
};

/**
 * @brief IMU 地磁気レンジ種別
 */
enum IMU_RANGE_COMP {
    RANGE_COMP_07GA = 0,
    RANGE_COMP_10GA,
    RANGE_COMP_15GA,
    RANGE_COMP_20GA,
    RANGE_COMP_32GA,
    RANGE_COMP_38GA,
    RANGE_COMP_45GA,
};




/**
 * @brief CAN SYSTEM COMMONメッセージ
 */
enum CAN_MSG_SYSCOM
{
	MSG_SYSCOM_REQ_ECHO	= 0x01,
	MSG_SYSCOM_READ_CONFIG	= 0x02,	/*!< コンフィグ読み出しメッセージ */
	MSG_SYSCOM_SET_CONFIG	= 0x03,	/*!< コンフィグ設定メッセージ */
	MSG_SYSCOM_SAVE_CONFIG	= 0x04,	/*!< コンフィグ書き込みメッセージ */

	MSG_SYSCOM_REQ_ERR      = 0x05,
	MSG_SYSCOM_REQ_MAINTAINANCE = 0x06,
	MSG_SYSCOM_REQ_VERSION = 0x07,
	MSG_SYSCOM_REP_WATCHDOG_STATUS = 0x08, // from WATCHDOG
	MSG_SYSCOM_SND_PC_STATUS = 0x09, 		// 500ms timeout

	MSG_SYSCOM_REQ_ADC_FIRM_ENTER = 0x10,
	MSG_SYSCOM_REQ_ADC_FIRM_ERASE = 0x11,
	MSG_SYSCOM_REQ_ADC_FIRM_WRITE = 0x12,
	MSG_SYSCOM_REQ_ADC_FIRM_VERIFY = 0x13,

	MSG_SYSCOM_REQ_WATCHDOC_FIRM_ENTER = 0x15,
	MSG_SYSCOM_REQ_WATCHDOC_FIRM_ERASE = 0x16,
	MSG_SYSCOM_REQ_WATCHDOC_FIRM_WRITE = 0x17,
	MSG_SYSCOM_REQ_WATCHDOC_FIRM_VERIFY = 0x18,


//	MSG_SYSCOM_REP_ADC_STATUS = 0x08,
//	MSG_SYSCOM_REP_WATCHDOG_STATUS = 0x09,

	MSG_SYSCOM_RES_ECHO		= 0x21,
	MSG_SYSCOM_RES_CONFIG	= 0x22,	/*!< コンフィグ読み出し応答メッセージ */
	MSG_SYSCOM_RES_ERR      = 0x23,
	MSG_SYSCOM_REP_SWITCH	= 0x24,
	MSG_SYSCOM_RES_VERSION	= 0x25,
	MSG_SYSCOM_REP_ADC_STATUS = 0x26,
};

enum CAN_MSG_IMUZ2
{
    MSG_IMUZ2_REQ_ECHO                  = 0x01,
    MSG_IMUZ2_REQ_STATUS                = 0x05,
    MSG_IMUZ2_SET_ROLE                  = 0x06,
    MSG_IMUZ2_SET_PERIOD                = 0x07,
    MSG_IMUZ2_SET_RANGE                 = 0x08,
    MSG_IMUZ2_SET_NODE_NO               = 0x09,
    MSG_IMUZ2_SAVE                      = 0x0a,
    MSG_IMUZ2_RESET_TIMESTAMP           = 0x0b,
    MSG_IMUZ2_SET_MEASUREMENT_STATE     = 0x0c,
    MSG_IMUZ2_REQ_DEVICE_PROFILE        = 0x0d,
    MSG_IMUZ2_SET_FACTORY_RESET         = 0x0e,
    MSG_IMUZ2_SET_BINARY                = 0x0f,
    MSG_IMUZ2_RES_ECHO                  = 0x21,
    MSG_IMUZ2_REP_MEASUREMENT_ACC       = 0x22,
    MSG_IMUZ2_REP_MEASUREMENT_GYRO      = 0x23,
    MSG_IMUZ2_REP_MEASUREMENT_COMP      = 0x24,
    MSG_IMUZ2_RES_STATUS                = 0x25,
    MSG_IMUZ2_RES_DEVICE_PROFILE        = 0x2d,
    MSG_IMUZ2_REP_GPS                   = 0x30,
    MSG_IMUZ2_REP_PRESSURE              = 0x31,
    MSG_IMUZ2_REP_HUMIDITY              = 0x32,
    MSG_IMUZ2_RES_STATUS_2              = 0x33,
};

enum CAN_MSG_INCLINO
{
    MSG_INCLINO_REP_ACC         = 0x22,
    MSG_INCLINO_RES_STATUS      = 0x25,
    MSG_INCLINO_REP_INCLINO     = 0x33,
};


/**
 * @brief CANデバイスチャネル
 */
enum CAN_DEVICE_CHANNEL
{
	CAN_CHANNEL_0,	/*!< CAN 0 */
	CAN_CHANNEL_1	/*!< CAN 1 */
};

/**
 * @brief CAN bit rate 種別
 */
enum CAN_BITRATE
{
	CAN_BITRATE_1M		= 0x0014, /*!< 1Mbps */
	CAN_BITRATE_500K	= 0x001C, /*!< 500kbps */
	CAN_BITRATE_250K	= 0x011C, /*!< 250kbps */
	CAN_BITRATE_125K	= 0x031C, /*!< 125kbps */
	CAN_BITRATE_100K	= 0x432F, /*!< 100kbps */
	CAN_BITRATE_50K		= 0x472F, /*!< 50kbps */
	CAN_BITRATE_20K		= 0x532F, /*!< 20kbps */
	CAN_BITRATE_10K		= 0x672F, /*!< 10kbps */
	CAN_BITRATE_5K		= 0x7F7F /*!< 5kbps */
};

#endif /* CANCONST_H_ */
