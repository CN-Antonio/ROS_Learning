/*
 * CommApp.h
 *
 *  Created on: 2014/03/03
 *      Author: sekiguchi
 */

#ifndef COMMAPP_H_
#define COMMAPP_H_


#include <unistd.h>
#include "CommServer.h"
#include "MvCnt.h"
//#include "CanServer.h"
//#include "LrfServer.h"

using namespace zmp::mv2;

class CommApp : public CommControlCommandHandler
{
public:
	CommApp();
	virtual ~CommApp();

public:
	bool Init();
	bool SetCommServer(CommServer* comm);
	bool SetMvCnt(MvCnt* cnt);
	bool Start();
	bool Process();
	bool Stop();
	bool Close();
	bool IsConnectComm();
	bool GetClientNameComm(char *buff);

    bool GetDistMean(double *mean);
    bool GetDistMin(double *min);
    bool GetDistPos(int *x1, int *x2, int *y1, int *y2);

	bool SendDrvMode(unsigned char mode, unsigned char cmode,
					 unsigned char omode, unsigned char servo);
	bool SendDrvShift(unsigned char input, unsigned char target, unsigned char  actual);
	bool SendDrvStroke(unsigned short input, unsigned short target, unsigned short actual);
	bool SendDrvVeloc(int target, int actual);
	bool SendDrvWheelVeloc(int fr, int fl, int rr, int rl);
	bool SendBrkStroke(unsigned short input, unsigned short target, unsigned short actual);
	bool SendBrkStatus(bool lamp, bool blinkLeft, bool blinkRight);
	bool SendStrMode(unsigned char mode, unsigned char cmode,
					 unsigned char omode, unsigned char servo);
	bool SendStrAngle(int target, int actual);
	bool SendStrTorque(int target, int actual);
	bool SendBattInf(float mCurrent, float mVolt, float sCurrent, float sVolt);
	bool SendCnfData(int index, short data);
	bool SendImuAcc(double accX, double accY, double accZ);
	bool SendImuGyro(double gyroX, double gyroY, double gyroZ);
	bool SendImuComp(double compX, double compY, double compZ);
	bool SendPosHumid(float humid, float temp);
	bool SendPosPress(float press, float temp);
	bool SendPosGGA(GPGGA_DATA gga);
	bool SendPosZDA(GPZDA_DATA zda);
	bool SendPosGLL(GPGLL_DATA gll);
	bool SendPosGSA(GPGSA_DATA gsa);
	bool SendPosGSV(GPGSV_DATA gsv);
	bool SendPosVTG(GPVTG_DATA vtg);
	bool SendPosRMC(GPRMC_DATA rmc);
	bool SendIncAngle(double angleX, double angleY);
	bool SendIncAcc(double accX, double accY);
	bool SendMillWave(MWInf data);
	bool SendErrInf(int leve, int code);
	bool SendFrmVer(int frmVer);
	bool SendVehicleInf(VehicleInf inf, int emergency);

	void onRcvVisionDistMean(const RczHMsgRcvVisionDistMean *msg);
    void onRcvVisionDistMin(const RczHMsgRcvVisionDistMin *msg);
    void onRcvVisionDistPos(const RczHMsgRcvVisionDistPos *msg);
	void onRcvSetDrvMode(const RczHMsgSetDrvMode *msg);
	void onRcvSetDrvCMode(const RczHMsgSetDrvCMode *msg);
//	void onRcvSetDrvOMode(const RczHMsgSetDrvOMode *msg);
	void onRcvSetDrvServo(const RczHMsgSetDrvServo *msg);
	void onRcvSetDrvShift(const RczHMsgSetDrvShift *msg);
	void onRcvSetDrvStroke(const RczHMsgSetDrvStroke *msg);
	void onRcvSetDrvVeloc(const RczHMsgSetDrvVeloc *msg);
	void onRcvSetBrkStroke(const RczHMsgSetBrkStroke *msg);
	void onRcvSetBrkLamp(const RczHMsgSetBrkLamp *msg);
	void onRcvSetBlinkLeft(const RczHMsgSetBlinkLeft *msg);
	void onRcvSetBlinkRight(const RczHMsgSetBlinkRight *msg);
	void onRcvSetStrMode(const RczHMsgSetStrMode *msg);
	void onRcvSetStrCMode(const RczHMsgSetStrCMode *msg);
//	void onRcvSetStrOMode(const RczHMsgSetStrOMode *msg);
	void onRcvSetStrServo(const RczHMsgSetStrServo *msg);
	void onRcvSetStrTorque(const RczHMsgSetStrTorque *msg);
	void onRcvSetStrAngle(const RczHMsgSetStrAngle *msg);
	void onRcvSetCnfData(const RczHMsgSetCnfData *msg);
	void onRcvGetCnfData(const RczHMsgGetCnfData *msg);
	void onRcvReqCnfSave(const RczHMsgReqCnfSave *msg);
	void onRcvGetErrInf(const RczHMsgGetErrInf *msg);
    void onRcvClearErr(const RczHMsgClearErr *msg);
	void onRcvGetFrmVer(const RczHMsgGetFrmVer *msg);

private:
	int _port_comm;
	CommControlCommandHandler *_handler_comm;
	CommServer *_server_comm;
	MvCnt *_mvCnt;

	bool _mv2Comm;
	ushort _mv2Period;

    double _distMean;
    double _distMin;
    int _posX1;
    int _posX2;
    int _posY1;
    int _posY2;

};

#endif /* SERVERAPP_H_ */
