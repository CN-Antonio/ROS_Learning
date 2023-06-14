/*
 * CommApp.cpp
 *
 *  Created on: 2014/03/03
 *      Author: sekiguchi
 */

#include "CommApp.h"

#include <stdlib.h>
#include <string.h>

#define DEFAULT_PORT_COMM 9000

CommApp::CommApp()
	: _port_comm(DEFAULT_PORT_COMM)
	, _handler_comm(NULL)
	, _server_comm(NULL)
	, _mvCnt(NULL)
{
}


CommApp::~CommApp()
{
}


/*void CommApp::SetCanServer(CanServer* can)
{
    _canServer = can;
}

void CommApp::SetLrfServer(LrfServer* lrf)
{
    _lrfServer = lrf;
}*/


bool CommApp::Init()
{
printf("CommApp::Init()\n");
    _distMean = 0;
    _distMin = 0;
    _posX1 = 0;
    _posX2 = 0;
    _posY1 = 0;
    _posY2 = 0;

	return true;
}

bool CommApp::SetCommServer(CommServer* comm)
{
printf("CommApp::SetCommServer()\n");
    _server_comm = comm;
    return true;
}

bool CommApp::SetMvCnt(MvCnt* cnt)
{
	_mvCnt = cnt;
	return true;
}

bool CommApp::Start()
{
printf("CommApp::Start()\n");
    _server_comm->Init(9001);
    _server_comm->SetReceiveCommHandler(this);
    _server_comm->Start();
    return true;
}

bool CommApp::Stop(){
	return true;
}

bool CommApp::Close(){
	return true;
}


// for measurement
#define CPU_CLOCK_PER_USEC 500.0 //[MHz]=[clock/usec]
#define CPU_USEC_PER_CLOCK 0.002 //[usec/clock]

inline unsigned long long int rdtsc(void)
{
	unsigned long long int x;
	__asm__ volatile (".byte 0x0f, 0x31" : "=A" (x));
	return x;
}

bool CommApp::IsConnectComm()
{
	return _server_comm->IsConnect();
}

bool CommApp::GetClientNameComm(char *buff)
{
	return _server_comm->GetClientName(buff);
}

bool CommApp::Process()
{
    while (_server_comm->IsReceiveMessage()) {
        _server_comm->KickCallback();
    }
//    RczHMsgResIncComm msg;
//    msg.onoff = 0;
//    msg.period = 100;
//    SendIncCommInfo(msg);
    
    return true;
}




void CommApp::onRcvVisionDistMean(const RczHMsgRcvVisionDistMean *msg)
{
    _distMean = msg->distMean;
}

void CommApp::onRcvVisionDistMin(const RczHMsgRcvVisionDistMin *msg)
{
    _distMin = msg->distMin;
}

void CommApp::onRcvVisionDistPos(const RczHMsgRcvVisionDistPos *msg)
{
    _posX1 = msg->posX1;
    _posY1 = msg->posY1;
    _posX2 = msg->posX2;
    _posY2 = msg->posY2;
}

void CommApp::onRcvSetDrvMode(const RczHMsgSetDrvMode *msg)
{
	if(_mvCnt == NULL)
		return;
	if(msg->mode != MODE_MANUAL && msg->mode != MODE_PROGRAM)
		return;
	_mvCnt->SetDrvMode((MV2_MODE)msg->mode);	
}

void CommApp::onRcvSetDrvCMode(const RczHMsgSetDrvCMode *msg)
{
	if(_mvCnt == NULL)
		return;
	if(msg->cmode != CONT_MODE_STROKE && msg->cmode != CONT_MODE_VELOCITY)
		return;
	_mvCnt->SetDrvCMode((DRIVE_CONTROL_MODE)msg->cmode);	
}

/*void CommApp::onRcvSetDrvOMode(const RczHMsgSetDrvOMode *msg)
{
	if(_mvCnt == NULL)
		return;
	if(msg->omode != OVERRIDE_MODE_ON && msg->omode != OVERRIDE_MODE_OFF)
		return;
	_mvCnt->SetDrvOMode((OVERRIDE_MODE)msg->omode);	
}
*/
void CommApp::onRcvSetDrvServo(const RczHMsgSetDrvServo *msg)
{
	if(_mvCnt == NULL)
		return;
	if(msg->servo != SERVO_ON && msg->servo != SERVO_OFF)
		return;
	_mvCnt->SetDrvServo((MV2_SERVO)msg->servo);	
}

void CommApp::onRcvSetDrvShift(const RczHMsgSetDrvShift *msg)
{
	if(_mvCnt == NULL)
		return;
	if(msg->shift == SHIFT_POS_R)
		return;
	_mvCnt->SetDrvShiftMode((SHIFT_POSITION)msg->shift);	
}

void CommApp::onRcvSetDrvStroke(const RczHMsgSetDrvStroke *msg)
{
	if(_mvCnt == NULL)
		return;
	if(msg->stroke < 0 || msg->stroke > 4095)
		return;
	_mvCnt->SetDrvStroke(msg->stroke);	
}

void CommApp::onRcvSetDrvVeloc(const RczHMsgSetDrvVeloc *msg)
{
	if(_mvCnt == NULL)
		return;
	if(msg->veloc < 0 || msg->veloc > 6000)
		return;
	_mvCnt->SetDrvVeloc(msg->veloc);
}

void CommApp::onRcvSetBrkStroke(const RczHMsgSetBrkStroke *msg)
{
	if(_mvCnt == NULL)
		return;
	if(msg->stroke < 0 || msg->stroke > 4095)
		return;
	_mvCnt->SetBrakeStroke(msg->stroke);
}

void CommApp::onRcvSetBrkLamp(const RczHMsgSetBrkLamp *msg)
{
	if(_mvCnt == NULL)
		return;
	_mvCnt->SetBrakeLamp(msg->lamp);
}

void CommApp::onRcvSetBlinkLeft(const RczHMsgSetBlinkLeft *msg)
{
	if(_mvCnt == NULL)
		return;
	_mvCnt->SetBlinkLeft(msg->blinkLeft);
}

void CommApp::onRcvSetBlinkRight(const RczHMsgSetBlinkRight *msg)
{
	if(_mvCnt == NULL)
		return;
	_mvCnt->SetBlinkRight(msg->blinkRight);
}

void CommApp::onRcvSetStrMode(const RczHMsgSetStrMode *msg)
{
	if(_mvCnt == NULL)
		return;
	if(msg->mode != MODE_MANUAL && msg->mode != MODE_PROGRAM)
		return;
	_mvCnt->SetStrMode((MV2_MODE)msg->mode);
}

void CommApp::onRcvSetStrCMode(const RczHMsgSetStrCMode *msg)
{
	if(_mvCnt == NULL)
		return;
	if(msg->cmode != CONT_MODE_TORQUE && msg->cmode != CONT_MODE_ANGLE)
		return;
	_mvCnt->SetStrCMode((STEER_CONTROL_MODE)msg->cmode);
}

/*void CommApp::onRcvSetStrOMode(const RczHMsgSetStrOMode *msg)
{
	if(_mvCnt == NULL)
		return;
	if(msg->omode != OVERRIDE_MODE_ON && msg->omode != OVERRIDE_MODE_OFF)
		return;
	_mvCnt->SetStrOMode((OVERRIDE_MODE)msg->omode);
}*/

void CommApp::onRcvSetStrServo(const RczHMsgSetStrServo *msg)
{
	if(_mvCnt == NULL)
		return;
	if(msg->servo != SERVO_ON && msg->servo != SERVO_OFF)
		return;
	_mvCnt->SetStrServo((MV2_SERVO)msg->servo);
}

void CommApp::onRcvSetStrTorque(const RczHMsgSetStrTorque *msg)
{
	if(_mvCnt == NULL)
		return;
	if(msg->torque < -4096 || msg->torque > 4096)
		return;
	_mvCnt->SetStrTorque(msg->torque);

}

void CommApp::onRcvSetStrAngle(const RczHMsgSetStrAngle *msg)
{
	if(_mvCnt == NULL)
		return;
	if(msg->angle < -6600 || msg->angle > 6600)
		return;
	_mvCnt->SetStrAngle(msg->angle);
}

void CommApp::onRcvSetCnfData(const RczHMsgSetCnfData *msg)
{
	if(_mvCnt == NULL)
		return;
	if(msg->index < CONFIG_STEER_KP || msg->index > TEST_FLAGS)
		return;
	_mvCnt->SetConfig((MV2_CONFIG)msg->index, msg->data);
}

void CommApp::onRcvGetCnfData(const RczHMsgGetCnfData *msg)
{
	if(_mvCnt == NULL)
		return;
	if(msg->index < CONFIG_STEER_KP || msg->index > TEST_FLAGS)
		return;
	_mvCnt->GetConfig((MV2_CONFIG)msg->index);
}

void CommApp::onRcvReqCnfSave(const RczHMsgReqCnfSave *msg)
{
	if(_mvCnt == NULL)
		return;
	_mvCnt->SaveConfig();
}

void CommApp::onRcvGetErrInf(const RczHMsgGetErrInf *msg)
{
	if(_mvCnt == NULL)
		return;
    _mvCnt->ReqOtherErr();
}

void CommApp::onRcvClearErr(const RczHMsgClearErr *msg)
{
    if(_mvCnt == NULL)
        return;
    _mvCnt->ReqClearErr();
}

void CommApp::onRcvGetFrmVer(const RczHMsgGetFrmVer *msg)
{
	if(_mvCnt == NULL)
		return;
	_mvCnt->ReqFirmVersion();
}

bool CommApp::GetDistMean(double *mean)
{
    *mean = _distMean;
    return true;
}

bool CommApp::GetDistMin(double *min)
{
    *min = _distMin;
    return true;
}

bool CommApp::GetDistPos(int *x1, int *x2, int *y1, int *y2)
{
    *x1 = _posX1;
    *x2 = _posX2;
    *y1 = _posY1;
    *y2 = _posY2;
    return true;
}

bool CommApp::SendDrvMode(unsigned char mode, unsigned char cmode, unsigned char omode, unsigned char servo)
{
	RczHMsgRepDrvMode msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepDrvMode);
	msg.msg_id		= RCZHMSG_ID_REP_DRV_MODE;
	msg.mode		= mode;
	msg.cmode		= cmode;
	msg.omode		= omode;
	msg.servo		= servo;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendDrvShift(unsigned char input, unsigned char target, unsigned char actual)
{
	RczHMsgRepDrvShift msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepDrvShift);
	msg.msg_id		= RCZHMSG_ID_REP_DRV_SHIFT;
	msg.input		= input;
	msg.target		= target;
	msg.actual		= actual;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendDrvStroke(unsigned short input, unsigned short target, unsigned short actual)
{
	RczHMsgRepDrvStroke msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepDrvStroke);
	msg.msg_id		= RCZHMSG_ID_REP_DRV_STROKE;
	msg.input		= input;
	msg.target		= target;
	msg.actual		= actual;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendDrvVeloc(int target, int actual)
{
	RczHMsgRepDrvVeloc msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepDrvVeloc);
	msg.msg_id		= RCZHMSG_ID_REP_DRV_VELOC;
	msg.target		= target;
	msg.actual		= actual;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendDrvWheelVeloc(int fr, int fl, int rr, int rl)
{
	RczHMsgRepDrvWheelVeloc msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepDrvWheelVeloc);
	msg.msg_id		= RCZHMSG_ID_REP_DRV_WHEELVELOC;
	msg.wheel_fr	= fr;
	msg.wheel_fl	= fl;
	msg.wheel_rr	= rr;
	msg.wheel_rl	= rl;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendBrkStroke(unsigned short input, unsigned short target, unsigned short actual)
{
	RczHMsgRepBrkStroke msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepBrkStroke);
	msg.msg_id		= RCZHMSG_ID_REP_BRK_STROKE;
	msg.input		= input;
	msg.target		= target;
	msg.actual		= actual;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendBrkStatus(bool lamp, bool blinkLeft, bool blinkRight)
{
	RczHMsgRepBrkStatus msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepBrkStatus);
	msg.msg_id		= RCZHMSG_ID_REP_BRK_STATUS;
	msg.lamp		= lamp;
	msg.blink_l		= blinkLeft;
	msg.blink_r		= blinkRight;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendStrMode(unsigned char mode, unsigned char cmode, unsigned char omode, unsigned char servo)
{
	RczHMsgRepStrMode msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepStrMode);
	msg.msg_id		= RCZHMSG_ID_REP_STR_MODE;
	msg.mode		= mode;
	msg.cmode		= cmode;
	msg.omode		= omode;
	msg.servo		= servo;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendStrAngle(int target, int actual)
{
	RczHMsgRepStrAngle msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepStrAngle);
	msg.msg_id		= RCZHMSG_ID_REP_STR_ANGLE;
	msg.target		= target;
	msg.actual		= actual;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendStrTorque(int target, int actual)
{
	RczHMsgRepStrTorque msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepStrTorque);
	msg.msg_id		= RCZHMSG_ID_REP_STR_TORQUE;
	msg.target		= target;
	msg.actual		= actual;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendBattInf(float mCurrent, float mVolt, float sCurrent, float sVolt)
{
	RczHMsgRepBattInf msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepBattInf);
	msg.msg_id		= RCZHMSG_ID_REP_BATT_INF;
	msg.main_current = mCurrent;
	msg.main_volt	= mVolt;
	msg.sub_current	= sCurrent;
	msg.sub_volt	= sVolt;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendCnfData(int index, short data)
{
	RczHMsgResCnfData msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgResCnfData);
	msg.msg_id		= RCZHMSG_ID_RES_CNF_DATA;
	msg.index		= index;
	msg.data		= data;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendImuAcc(double accX, double accY, double accZ)
{
	RczHMsgRepImuAcc msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepImuAcc);
	msg.msg_id		= RCZHMSG_ID_REP_IMU_ACC;
	msg.accX		= accX;
	msg.accY		= accY;
	msg.accZ		= accZ;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendImuGyro(double gyroX, double gyroY, double gyroZ)
{
	RczHMsgRepImuGyro msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepImuGyro);
	msg.msg_id		= RCZHMSG_ID_REP_IMU_GYRO;
	msg.gyroX		= gyroX;
	msg.gyroY		= gyroY;
	msg.gyroZ		= gyroZ;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendImuComp(double compX, double compY, double compZ)
{
	RczHMsgRepImuComp msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepImuComp);
	msg.msg_id		= RCZHMSG_ID_REP_IMU_COMP;
	msg.compX		= compX;
	msg.compY		= compY;
	msg.compZ		= compZ;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendPosHumid(float humid, float temp)
{
	RczHMsgRepPosHumid msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepPosHumid);
	msg.msg_id		= RCZHMSG_ID_REP_POS_HUMID;
	msg.humid		= humid;
	msg.temp		= temp;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendPosPress(float press, float temp)
{
	RczHMsgRepPosPress msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepPosPress);
	msg.msg_id		= RCZHMSG_ID_REP_POS_PRESS;
	msg.press		= press;
	msg.temp		= temp;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendPosGGA(GPGGA_DATA data)
{
	RczHMsgRepPosGGA msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepPosGGA);
	msg.msg_id		= RCZHMSG_ID_REP_POS_GPGGA;
	msg.gga			= data;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendPosZDA(GPZDA_DATA data)
{
	RczHMsgRepPosZDA msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepPosZDA);
	msg.msg_id		= RCZHMSG_ID_REP_POS_GPZDA;
	msg.zda			= data;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendPosGLL(GPGLL_DATA data)
{
	RczHMsgRepPosGLL msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepPosGLL);
	msg.msg_id		= RCZHMSG_ID_REP_POS_GPGLL;
	msg.gll			= data;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendPosGSA(GPGSA_DATA data)
{
	RczHMsgRepPosGSA msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepPosGSA);
	msg.msg_id		= RCZHMSG_ID_REP_POS_GPGSA;
	msg.gsa			= data;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendPosGSV(GPGSV_DATA data)
{
	RczHMsgRepPosGSV msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepPosGSV);
	msg.msg_id		= RCZHMSG_ID_REP_POS_GPGSV;
	msg.gsv			= data;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendPosVTG(GPVTG_DATA data)
{
	RczHMsgRepPosVTG msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepPosVTG);
	msg.msg_id		= RCZHMSG_ID_REP_POS_GPVTG;
	msg.vtg			= data;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendPosRMC(GPRMC_DATA data)
{
	RczHMsgRepPosRMC msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepPosRMC);
	msg.msg_id		= RCZHMSG_ID_REP_POS_GPRMC;
	msg.rmc			= data;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendIncAngle(double angleX, double angleY)
{
	RczHMsgRepIncAngle msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepIncAngle);
	msg.msg_id		= RCZHMSG_ID_REP_INC_ANGLE;
	msg.angleX		= angleX;
	msg.angleY		= angleY;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendIncAcc(double accX, double accY)
{
	RczHMsgRepIncAcc msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepIncAcc);
	msg.msg_id		= RCZHMSG_ID_REP_INC_ACC;
	msg.accX		= accX;
	msg.accY		= accY;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendMillWave(MWInf data)
{
	RczHMsgRepMillwaveData msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepMillwaveData);
	msg.msg_id		= RCZHMSG_ID_REP_MW_DATA;
	for(int i=0; i<20; i++){
		msg.data[i] = data.data[i];
	}

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendErrInf(int level, int code)
{
	RczHMsgResErrInf msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgResErrInf);
	msg.msg_id		= RCZHMSG_ID_RES_ERR_INF;
	msg.level		= level;
	msg.code		= code;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendFrmVer(int frmVer)
{
	RczHMsgResFrmVer msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgResFrmVer);	
	msg.msg_id		= RCZHMSG_ID_RES_FRM_VER;
	msg.version = frmVer;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}

bool CommApp::SendVehicleInf(VehicleInf inf, int emergency)
{
	RczHMsgRepVehicleInf msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length		= sizeof(RczHMsgRepVehicleInf);
	msg.msg_id		= RCZHMSG_ID_REP_VEHICLE_INF;
	msg.battSOC		= inf.soc;
	msg.battTemp	= inf.battTemp;
	msg.battLevel	= inf.battLevel;
	msg.veloc		= inf.veloc;
	msg.meterVeloc	= inf.meterVeloc;
	msg.odometer	= inf.odometer;
	msg.trip		= inf.trip;
	msg.emergency	= emergency;

	return _server_comm->Send((const uchar*)&msg, msg.length);
}
