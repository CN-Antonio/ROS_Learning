#ifndef __RCZ_HMSG_H__
#define __RCZ_HMSG_H__

#include "types.h"
#include "MV2Const.h"

using namespace zmp::mv2;

#define IMAGE_BLOCK_SIZE_MAX	(1024)

enum RCZHMSG_ID {
	/*** Vision info ***/
	RCZHMSG_ID_VISION_DIST_MEAN = 0x01,
	RCZHMSG_ID_VISION_DIST_MIN = 0x02,
	RCZHMSG_ID_VISION_DIST_POS = 0x03,
	/*** Drive control ***/
	RCZHMSG_ID_SET_DRV_MODE = 0x10,
	RCZHMSG_ID_SET_DRV_CMODE = 0x11,
	RCZHMSG_ID_SET_DRV_OMODE = 0x12,
	RCZHMSG_ID_SET_DRV_SERVO = 0x13,
	RCZHMSG_ID_SET_DRV_SHIFT = 0x14,
	RCZHMSG_ID_SET_DRV_STROKE = 0x15,
	RCZHMSG_ID_SET_DRV_VELOC = 0x16,
	RCZHMSG_ID_SET_BRK_STROKE = 0x17,
	RCZHMSG_ID_SET_BRK_LAMP = 0x18,
	RCZHMSG_ID_SET_BRK_BLINK_L = 0x19,
	RCZHMSG_ID_SET_BRK_BLINK_R = 0x1A,
	/*** Drive info ***/
	RCZHMSG_ID_REP_DRV_MODE = 0x20,
	RCZHMSG_ID_REP_DRV_SHIFT = 0x21,
	RCZHMSG_ID_REP_DRV_STROKE = 0x22,
	RCZHMSG_ID_REP_DRV_VELOC = 0x23,
	RCZHMSG_ID_REP_DRV_WHEELVELOC = 0x24,
	RCZHMSG_ID_REP_BRK_STROKE = 0x25,
	RCZHMSG_ID_REP_BRK_STATUS = 0x26,
	/*** Steer control ***/
	RCZHMSG_ID_SET_STR_MODE = 0x30,
	RCZHMSG_ID_SET_STR_CMODE = 0x31,
	RCZHMSG_ID_SET_STR_OMODE = 0x32,
	RCZHMSG_ID_SET_STR_SERVO = 0x33,
	RCZHMSG_ID_SET_STR_TORQUE = 0x34,
	RCZHMSG_ID_SET_STR_ANGLE = 0x35, 
	/*** Steer info ***/
	RCZHMSG_ID_REP_STR_MODE = 0x40,
	RCZHMSG_ID_REP_STR_ANGLE = 0x41,
	RCZHMSG_ID_REP_STR_TORQUE = 0x42,
	/*** Other control ***/
	RCZHMSG_ID_SET_CNF_DATA = 0x50,
	RCZHMSG_ID_GET_CNF_DATA = 0x51,
	RCZHMSG_ID_REQ_CNF_SAVE = 0x52,
	RCZHMSG_ID_GET_ERR_INF = 0x53,
	RCZHMSG_ID_CLEAR_ERR   = 0x54,
	RCZHMSG_ID_GET_FRM_VER = 0x55,
	/*** Other info ***/
	RCZHMSG_ID_REP_BATT_INF = 0x60,
	RCZHMSG_ID_RES_CNF_DATA = 0x61,
	RCZHMSG_ID_RES_ERR_INF = 0x62,
	RCZHMSG_ID_RES_FRM_VER = 0x63,
	RCZHMSG_ID_REP_VEHICLE_INF = 0x64,
	/*** Option sensor (ZMP)***/
	RCZHMSG_ID_REP_IMU_ACC = 0x80,
	RCZHMSG_ID_REP_IMU_GYRO = 0x81,
	RCZHMSG_ID_REP_IMU_COMP = 0x82,
	RCZHMSG_ID_REP_POS_HUMID = 0x83,
	RCZHMSG_ID_REP_POS_PRESS = 0x84,
	RCZHMSG_ID_REP_POS_GPGGA = 0x85,
	RCZHMSG_ID_REP_POS_GPZDA = 0x86,
	RCZHMSG_ID_REP_POS_GPGLL = 0x87,
	RCZHMSG_ID_REP_POS_GPGSA = 0x88,
	RCZHMSG_ID_REP_POS_GPGSV = 0x89,
	RCZHMSG_ID_REP_POS_GPVTG = 0x8A,
	RCZHMSG_ID_REP_POS_GPRMC = 0x8B,
	RCZHMSG_ID_REP_INC_ANGLE = 0x8C,
	RCZHMSG_ID_REP_INC_ACC = 0x8D,
	/*** Option sensor (other)***/
	RCZHMSG_ID_REP_MW_DATA = 0x90,
};

#pragma pack(push, 1)

struct RczHMsg {
	uchar	header[2];
	uint	length;
	uchar	msg_id;
};


// RCZHMSG_ID_LOAD_PROGRAM
struct RczHMsgRcvVisionDistMean : public RczHMsg
{
	double distMean;
};

struct RczHMsgRcvVisionDistMin : public RczHMsg
{
	double distMin;
};

struct RczHMsgRcvVisionDistPos : public RczHMsg
{
	int posX1;
	int posX2;
	int posY1;
	int posY2;
};

struct RczHMsgSetDrvMode : public RczHMsg
{
	unsigned char mode;
};

struct RczHMsgSetDrvCMode : public RczHMsg
{
	unsigned char cmode;
};

struct RczHMsgSetDrvOMode : public RczHMsg
{
	unsigned char omode;
};

struct RczHMsgSetDrvServo : public RczHMsg
{
	unsigned char servo;
};

struct RczHMsgSetDrvShift : public RczHMsg
{
	unsigned char shift;
};

struct RczHMsgSetDrvStroke : public RczHMsg
{
	unsigned short stroke;
};

struct RczHMsgSetDrvVeloc : public RczHMsg
{
	int veloc;
};

struct RczHMsgSetBrkStroke : public RczHMsg
{
	unsigned short stroke;
};

struct RczHMsgSetBrkLamp : public RczHMsg
{
	bool lamp;
};

struct RczHMsgSetBlinkLeft : public RczHMsg
{
	bool blinkLeft;
};

struct RczHMsgSetBlinkRight : public RczHMsg
{
	bool blinkRight;
};

struct RczHMsgRepDrvMode : public RczHMsg
{
	unsigned char mode;
	unsigned char cmode;
	unsigned char omode;
	unsigned char servo;
};

struct RczHMsgRepDrvShift : public RczHMsg
{
	unsigned char input;
	unsigned char target;
	unsigned char actual;
};

struct RczHMsgRepDrvStroke : public RczHMsg
{
	unsigned short input;
	unsigned short target;
	unsigned short actual;
};

struct RczHMsgRepDrvVeloc : public RczHMsg
{
	int	target;
	int actual;
};

struct RczHMsgRepDrvWheelVeloc : public RczHMsg
{
	int	wheel_fr;
	int wheel_fl;
	int wheel_rr;
	int wheel_rl;
};

struct RczHMsgRepBrkStroke : public RczHMsg
{
	unsigned short input;
	unsigned short target;
	unsigned short actual;
};

struct RczHMsgRepBrkStatus : public RczHMsg
{
	bool lamp;
	bool blink_l;
	bool blink_r;
};

struct RczHMsgSetStrMode : public RczHMsg
{
	unsigned char mode;
};

struct RczHMsgSetStrCMode : public RczHMsg
{
	unsigned char cmode;
};

struct RczHMsgSetStrOMode : public RczHMsg
{
	unsigned char omode;
};

struct RczHMsgSetStrServo : public RczHMsg
{
	unsigned char servo;
};

struct RczHMsgSetStrTorque : public RczHMsg
{
	int torque;
};

struct RczHMsgSetStrAngle : public RczHMsg
{
	int angle;
};

struct RczHMsgRepStrMode : public RczHMsg
{
	unsigned char mode;
	unsigned char cmode;
	unsigned char omode;
	unsigned char servo;
};

struct RczHMsgRepStrAngle : public RczHMsg
{
	int target;
	int actual;
};

struct RczHMsgRepStrTorque : public RczHMsg
{
	int target;
	int actual;
};

struct RczHMsgSetCnfData : public RczHMsg
{
	int index;
	short data;
};

struct RczHMsgGetCnfData : public RczHMsg
{
	int index;
};

struct RczHMsgReqCnfSave : public RczHMsg
{
};

struct RczHMsgGetErrInf : public RczHMsg
{
};

struct RczHMsgClearErr : public RczHMsg
{
};

struct RczHMsgGetFrmVer : public RczHMsg
{
};

struct RczHMsgResErrInf : public RczHMsg
{
	int level;
	int code;
};

struct RczHMsgResFrmVer : public RczHMsg
{
	int version;
};

struct RczHMsgRepBattInf : public RczHMsg
{
	float main_current;
	float main_volt;
	float sub_current;
	float sub_volt;
};

struct RczHMsgResCnfData : public RczHMsg
{
	int index;
	short data;
};

struct RczHMsgRepVehicleInf : public RczHMsg
{
	float	battSOC;
	float	battTemp;
	int		battLevel;
	float	veloc;
	int		meterVeloc;
	int		odometer;
	float	trip;
	int		emergency;
};

struct RczHMsgRepImuAcc : public RczHMsg
{
	double accX;
	double accY;
	double accZ;
};

struct RczHMsgRepImuGyro : public RczHMsg
{
	double gyroX;
	double gyroY;
	double gyroZ;
};

struct RczHMsgRepImuComp : public RczHMsg
{
	double compX;
	double compY;
	double compZ;
};

struct RczHMsgRepPosHumid : public RczHMsg
{
	float humid;
	float temp;
};

struct RczHMsgRepPosPress : public RczHMsg
{
	float press;
	float temp;
};

struct RczHMsgRepPosGGA : public RczHMsg
{
	GPGGA_DATA gga;
};

struct RczHMsgRepPosZDA : public RczHMsg
{
	GPZDA_DATA zda;
};

struct RczHMsgRepPosGLL : public RczHMsg
{
	GPGLL_DATA gll;
};

struct RczHMsgRepPosGSA : public RczHMsg
{
	GPGSA_DATA gsa;
};

struct RczHMsgRepPosGSV : public RczHMsg
{
	GPGSV_DATA gsv;
};

struct RczHMsgRepPosVTG : public RczHMsg
{
	GPVTG_DATA vtg;
};

struct RczHMsgRepPosRMC : public RczHMsg
{
	GPRMC_DATA rmc;
};

struct RczHMsgRepIncAngle : public RczHMsg
{
	double angleX;
	double angleY;
};

struct RczHMsgRepIncAcc : public RczHMsg
{
	double accX;
	double accY;
};

struct RczHMsgRepMillwaveData : public RczHMsg
{
	MILLIWAVE_DATA data[20];
};


#pragma pack(pop)

#endif // __RCZ_HMSG_H__
