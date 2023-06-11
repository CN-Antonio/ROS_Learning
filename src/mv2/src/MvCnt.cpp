#include "MvCnt.h"
#include <time.h>
#define USE_DEMO 1

MvCnt::MvCnt()
{
    _errCode = 0;
    _errLevel = 0;
    memset(&_battInf, 0x0, sizeof(BattInf));
    memset(&_brakeInf, 0x0, sizeof(BrakeInf));
    memset(&_otherInf, 0x0, sizeof(OtherInf));
    memset(&_drvInf, 0x0, sizeof(DrvInf));
    memset(&_strInf, 0x0, sizeof(StrInf));
    memset(&_config, 0x0, sizeof(ConfigInf));
    memset(&_vehicleInf, 0x0, sizeof(VehicleInf));
    memset(&_imuInf, 0x0, sizeof(ImuInf));
    memset(&_incInf, 0x0, sizeof(IncInf));
    memset(&_posInf, 0x0, sizeof(PosInf));
    memset(&_mwInf, 0x0, sizeof(MWInf));

}

MvCnt::~MvCnt()
{}

bool MvCnt::Init()
{
    _canCom = new zmp::hev::CANUSBZ();
//    _canCom2 = new zmp::hev::CANUSBZ();
    _canCom->Init(1);
//    _canCom2->Init(2);

    _canCom->Start();
//    _canCom2->Start();
    _canCom->SetCANUSBZParam(CAN_CHANNEL_0, CAN_SPEED_500, CANID_KIND_11);
    _canCom->SetCANUSBZParam(CAN_CHANNEL_1, CAN_SPEED_1000, CANID_KIND_11);
//    _canCom2->SetCANUSBZParam(CAN_CHANNEL_0, CAN_SPEED_500, CANID_KIND_11);
//    _canCom2->SetCANUSBZParam(CAN_CHANNEL_1, CAN_SPEED_1000, CANID_KIND_11);

    _mvCnt = new Mv2Control();
    _mvCnt->InitMv2Control(_canCom);//, _canCom2);
    _mvCnt->SetStatusCallback(this);
    _callback = NULL;

    return true;
}

bool MvCnt::Start()
{
    _mvCnt->SetStrMode(MODE_MANUAL);
    _mvCnt->SetStrControlMode(CONT_MODE_ANGLE);
//    _mvCnt->SetStrOverrideMode(OVERRIDE_MODE_ON);
    _mvCnt->SetStrAngle(0.0f);
    _mvCnt->SetStrServo(SERVO_OFF);
    _mvCnt->SetStrAngle(0.0f);

    return true;
}

bool MvCnt::SetConfigCallback(ChangeConfig* callback)
{
    _callback = callback;
    return true;
}

bool MvCnt::Stop()
{
    _mvCnt->SetStrServo(SERVO_OFF);
    _mvCnt->SetDrvServo(SERVO_OFF);
    _mvCnt->SetStrMode(MODE_MANUAL);
    _mvCnt->SetDrvMode(MODE_MANUAL);
    return true;
}

bool MvCnt::Close()
{
    _canCom->Close();
//    _canCom2->Close();
    return true;
}


void MvCnt::GetBattInf(BattInf* batt)
{
    batt->mCurrent = _battInf.mCurrent;
    batt->sCurrent = _battInf.sCurrent;
    batt->mVoltage = _battInf.mVoltage;
    batt->sVoltage = _battInf.sVoltage;
}

void MvCnt::GetBrakeInf(BrakeInf* brake)
{
    brake->iStroke = _brakeInf.iStroke;
    brake->tStroke = _brakeInf.tStroke;
    brake->aStroke = _brakeInf.aStroke;
	brake->lamp		= _brakeInf.lamp;
	brake->blinkL	= _brakeInf.blinkL;
	brake->blinkR	= _brakeInf.blinkR;
}

void MvCnt::GetOtherInf(OtherInf* other)
{
    other->errLevel = _otherInf.errLevel;
    other->errCode = _otherInf.errCode;
    other->ignition = _otherInf.ignition;
    other->emergency = _otherInf.emergency;
    other->adcStatus = _otherInf.adcStatus;
    other->watchdogStatus = _otherInf.watchdogStatus;
    _otherInf.adcStatus = 0x10;
}

void MvCnt::GetDrvInf(DrvInf* drv)
{
    drv->servo = _drvInf.servo;
    drv->mode = _drvInf.mode;
    drv->cmode = _drvInf.cmode;
    drv->omode = _drvInf.omode;
    drv->iShift = _drvInf.iShift;
    drv->tShift = _drvInf.tShift;
    drv->aShift = _drvInf.aShift;
    drv->iStroke = _drvInf.iStroke;
    drv->tStroke = _drvInf.tStroke;
    drv->aStroke = _drvInf.aStroke;
    drv->tVeloc = _drvInf.tVeloc;
    drv->aVeloc = _drvInf.aVeloc;
    drv->fl = _drvInf.fl;
    drv->fr = _drvInf.fr;
    drv->rl = _drvInf.rl;
    drv->rr = _drvInf.rr;
}

void MvCnt::GetStrInf(StrInf* str)
{
    str->mode = _strInf.mode;
    str->cmode = _strInf.cmode;
    str->omode = _strInf.omode;
    str->servo = _strInf.servo;
    str->tTorque = _strInf.tTorque;
    str->aTorque = _strInf.aTorque;
    str->tAngle = _strInf.tAngle;
    str->aAngle = _strInf.aAngle;
}

void MvCnt::GetVehicleInf(VehicleInf* vehicle)
{
    vehicle->soc = _vehicleInf.soc;
    vehicle->veloc = _vehicleInf.veloc;
    vehicle->battTemp = _vehicleInf.battTemp;
    vehicle->meterVeloc = _vehicleInf.meterVeloc;
    vehicle->battLevel = _vehicleInf.battLevel;
    vehicle->odometer = _vehicleInf.odometer;
    vehicle->trip = _vehicleInf.trip;
}

void MvCnt::GetPosInf(PosInf *pos)
{
    pos->gga = _posInf.gga;
    pos->hTemp = _posInf.hTemp;
    pos->humid = _posInf.humid;
    pos->press = _posInf.press;
    pos->pTemp = _posInf.pTemp;
}

void MvCnt::GetIncInf(IncInf *inc)
{
    inc->angleX = _incInf.angleX;
    inc->angleY = _incInf.angleY;
}

void MvCnt::GetMWInf(MWInf *mwInf)
{
    for(int i=0; i<20; i++){
	mwInf->data[i].angle = _mwInf.data[i].angle;
	mwInf->data[i].distance = _mwInf.data[i].distance;
	mwInf->data[i].rangeRate = _mwInf.data[i].rangeRate;
    }
}

void MvCnt::GetImuInf(ImuInf* imuInf)
{
    imuInf->accX = _imuInf.accX;
    imuInf->accY = _imuInf.accY;
    imuInf->accZ = _imuInf.accZ;

    imuInf->gyroX = _imuInf.gyroX;
    imuInf->gyroY = _imuInf.gyroY;
    imuInf->gyroZ = _imuInf.gyroZ;

    imuInf->compX = _imuInf.compX;
    imuInf->compY = _imuInf.compY;
    imuInf->compZ = _imuInf.compZ;

    imuInf->battLevel = _imuInf.battLevel;
    imuInf->firmVersion = _imuInf.firmVersion;
    imuInf->format = _imuInf.format;
    imuInf->hardVersion = _imuInf.hardVersion;
    imuInf->period = _imuInf.period;
    imuInf->rangeAcc = _imuInf.rangeAcc;
    imuInf->rangeComp = _imuInf.rangeComp;
    imuInf->rangeGyro = _imuInf.rangeGyro;
    imuInf->role = _imuInf.role;
}

void MvCnt::GetFirmVersion(int* version)
{
	*version = _firmVersion;
}


void MvCnt::UpdateSteerState(REP_STEER_INFO_INDEX index)
{
    switch(index){
    case REP_STR_MODE:
        _mvCnt->GetStrMode((int&)_strInf.mode);
        _mvCnt->GetStrControlMode((int&)_strInf.cmode);
        _mvCnt->GetStrOverrideMode((int&)_strInf.omode);
        _mvCnt->GetStrServo((int&)_strInf.servo);
        break;
    case REP_STR_TORQUE:
        _mvCnt->GetStrActualTorque((int&)_strInf.aTorque);
        _mvCnt->GetStrTargetTorque((int&)_strInf.tTorque);
        break;
    case REP_STR_ANGLE:
        _mvCnt->GetStrActualAngle((float&)_strInf.aAngle);
        _mvCnt->GetStrTargetAngle((float&)_strInf.tAngle);
        break;
	default: break;
    }

    return;
}

void MvCnt::UpdateDriveState(REP_DRIVE_INFO_INDEX index)
{
    switch(index){
    case REP_DRV_MODE:
        _mvCnt->GetDrvMode((int&)_drvInf.mode);
        _mvCnt->GetDrvControlMode((int&)_drvInf.cmode);
        _mvCnt->GetDrvOverrideMode((int&)_drvInf.omode);
        _mvCnt->GetDrvServo((int&)_drvInf.servo);
        break;
    case REP_DRV_PEDAL:
        _mvCnt->GetDrvActualStroke((int&)_drvInf.aStroke);
        _mvCnt->GetDrvTargetStroke((int&)_drvInf.tStroke);
        _mvCnt->GetDrvInputStroke((int&)_drvInf.iStroke);
        break;
    case REP_DRV_VELOCITY:
        _mvCnt->GetDrvActualVeloc((float&)_drvInf.aVeloc);
        _mvCnt->GetDrvTargetVeloc((float&)_drvInf.tVeloc);
        break;
    case REP_DRV_WHEEL_VELOCITY:
        _mvCnt->GetDrvWheelVeloc((int&)_drvInf.fr,
                                 (int&)_drvInf.fl,
                                 (int&)_drvInf.rr,
                                 (int&)_drvInf.rl);
        break;
    case REP_BRAKE_PEDAL:
        _mvCnt->GetBrakeActualStroke((int&)_brakeInf.aStroke);
        _mvCnt->GetBrakeTargetStroke((int&)_brakeInf.tStroke);
        _mvCnt->GetBrakeInputStroke((int&)_brakeInf.iStroke);
        break;
    case REP_DRV_SHIFT_POS:
        _mvCnt->GetDrvActualShift((int&)_drvInf.aShift);
        _mvCnt->GetDrvTargetShift((int&)_drvInf.tShift);
        _mvCnt->GetDrvInputShift((int&)_drvInf.iShift);
        break;
	case REP_BRAKE_STATUS:
        _mvCnt->GetBrakeStatus((unsigned char&)_brakeInf.lamp, (unsigned char&)_brakeInf.blinkL, (unsigned char&)_brakeInf.blinkR);
		break;
	default: break;
    }
    return;
}

void MvCnt::UpdateBattState(REP_BATT_INFO_INDEX index)
{
    switch(index){
    case REP_BATT_MAIN_CURRENT:
        _mvCnt->GetBattMainCurrent((float&)_battInf.mCurrent);
        break;
    case REP_BATT_SUB_CURRENT:
        _mvCnt->GetBattSubCurrent((float&)_battInf.sCurrent);
        break;
    case REP_BATT_MAIN_VOLTAGE:
        _mvCnt->GetBattMainVoltage((float&)_battInf.mVoltage);
        break;
    case REP_BATT_SUB_VOLTAGE:
        _mvCnt->GetBattSubVoltage((float&)_battInf.sVoltage);
        break;
	default: break;
    }

    return;
}

void MvCnt::UpdateOtherState(REP_OTHER_INFO_INDEX index)
{
    switch(index){
    case RES_SYSCOM_ECHO:
        break;
    case RES_SYSCOM_ERR_STATE:
        _mvCnt->GetOtherErrState((int&)_otherInf.errLevel,
                                 (int&)_otherInf.errCode);
        break;
    case REP_SYSCOM_SWITCH_STATE:
        _mvCnt->GetOtherSwitchState((int&)_otherInf.ignition,
                                    (int&)_otherInf.emergency);
        break;
    case REP_SYSCOM_ADC_STATUS:
        _mvCnt->GetOtherAdcStatus((int&)_otherInf.adcStatus);
        break;
    case REP_SYSCOM_WATCHDOG_STATUS:
        _mvCnt->GetOtherWatchdogStatus((int&)_otherInf.watchdogStatus);
        break;
    default: break;
    }

    return;
}

void MvCnt::UpdateVehicleState(REP_VEHICLE_INFO_INDEX index)
{
    switch(index){
    case REP_VEHICLE_BATT_SOC:
        _mvCnt->GetVehicleBattSOC((float&)_vehicleInf.soc);
        break;
    case REP_VEHICLE_VELOC:
        _mvCnt->GetVehicleVeloc((float&)_vehicleInf.veloc);
        break;
    case REP_VEHICLE_BATT_TEMP:
        _mvCnt->GetVehicleBattTemp((float&)_vehicleInf.battTemp);
        break;
    case REP_VEHICLE_METER_VELOC:
        _mvCnt->GetVehicleMeterVeloc((int&)_vehicleInf.meterVeloc);
        break;
    case REP_VEHICLE_BATT_LEVEL:
        _mvCnt->GetVehicleBattLevel((int&)_vehicleInf.battLevel);
        break;
    case REP_VEHICLE_ODOMETER:
        _mvCnt->GetVehicleOdometer((int&)_vehicleInf.odometer);
        break;
    case REP_VEHICLE_TRIP:
        _mvCnt->GetVehicleTrip((float&)_vehicleInf.trip);
        break;
    default:
        break;
    }
    return;
}

void MvCnt::ReceiveConfig(int num, int index, int value[])
{
    int data[3];
    for(int i=0; i<num; i++){
        _config.data[index - 100] = value[i];
        data[i] = value[i];
    }
    if(NULL != _callback){
        _callback->UpdateConfig(num, index, data);
    }
}

void MvCnt::ReceiveError(int level, int errCode)
{
    _errCode = errCode;
    _errLevel = level;
}

void MvCnt::ReceiveEcho(int kind, int no)
{
}

void MvCnt::UpdatePoszState(REP_POSZ_INFO_INDEX index)
{
    switch(index){
    case REP_POSZ_GPGGA:
        _mvCnt->GetPoszGPGGA((GPGGA_DATA&)_posInf); break;
    case REP_POSZ_GPZDA: break;
    case REP_POSZ_GPGLL: break;
    case REP_POSZ_GPGSA: break;
    case REP_POSZ_GPGSV: break;
    case REP_POSZ_GPVTG: break;
    case REP_POSZ_GPRMC: break;
    case REP_POSZ_PRESSURE:
        _mvCnt->GetPoszPressData((float&)_posInf.press, (float&)_posInf.pTemp); break;
    case REP_POSZ_HUMIDTY:
        _mvCnt->GetPoszHumidData((float&)_posInf.humid, (float&)_posInf.hTemp); break;
    default: break;
    }
}

void MvCnt::UpdateInczState(REP_INCZ_INFO_INDEX index)
{
    switch(index){
    case REP_INCZ_STATE:
    case REP_INCZ_ACC:
        break;
    case REP_INCZ_ANGLE:
        _mvCnt->GetInczInf((double&)_incInf.angleX, (double&)_incInf.angleY);
        break;
        default: break;
    }
}

void MvCnt::UpdateMilliWave(int index)
{
    MILLIWAVE_DATA data;
    _mvCnt->GetMilliWaveInf(index, data);
    _mwInf.data[index].angle = data.angle;
    _mwInf.data[index].distance = data.distance;
    _mwInf.data[index].rangeRate = data.rangeRate;
}

void MvCnt::UpdateImuz2State(REP_IMUZ2_INFO_INDEX index)
{
    switch(index){
    case REP_IMUZ2_ACC_DATA:	/*!< 加速度データ */
        _mvCnt->GetImuAccX((double&)_imuInf.accX);
        _mvCnt->GetImuAccY((double&)_imuInf.accY);
        _mvCnt->GetImuAccZ((double&)_imuInf.accZ);
        break;
    case REP_IMUZ2_GYRO_DATA:/*!< 角速度データ */
        _mvCnt->GetImuGyroX((double&)_imuInf.gyroX);
        _mvCnt->GetImuGyroY((double&)_imuInf.gyroY);
        _mvCnt->GetImuGyroZ((double&)_imuInf.gyroZ);
        break;
    case REP_IMUZ2_COMP_DATA: /*!< 地磁気データ */
        _mvCnt->GetImuCompX((double&)_imuInf.compX);
        _mvCnt->GetImuCompY((double&)_imuInf.compY);
        _mvCnt->GetImuCompZ((double&)_imuInf.compZ);
        break;
    case RES_IMUZ2_STATE:	/*!< IMU-Z2状態 */
        _mvCnt->GetImuRole((IMUZ2_ROLE&)_imuInf.role);
        _mvCnt->GetImuPeriod((int&)_imuInf.period);
        _mvCnt->GetImuRangeAcc((zmp::mv2::RANGE_ACC&)_imuInf.rangeAcc);
        _mvCnt->GetImuRangeGyro((zmp::mv2::RANGE_GYRO&)_imuInf.rangeGyro);
        _mvCnt->GetImuRangeComp((zmp::mv2::RANGE_COMP&)_imuInf.rangeComp);
        _mvCnt->GetImuBattLevel((float&)_imuInf.battLevel);
        _mvCnt->GetImuFormat((IMUZ2_FORMAT&)_imuInf.format);
        break;
    case RES_IMUZ2_PROFILE:/*!< デバイスプロファイル */
        _mvCnt->GetImuHardVersion((unsigned char&)_imuInf.hardVersion);
        _mvCnt->GetImuFirmVersion((unsigned char&)_imuInf.firmVersion);
        break;
    default: break;
    }
}

void MvCnt::ReceiveVersion(int version)
{
    printf("ReceiveVersion() %d\n", version );
	_firmVersion = version;
}


void MvCnt::SetStrMode(int mode)
{
    _mvCnt->SetStrMode((MV2_MODE)mode);
}

void MvCnt::SetStrCMode(int cmode)
{
    _mvCnt->SetStrControlMode((STEER_CONTROL_MODE)cmode);
}

/*void MvCnt::SetStrOMode(int omode)
{
    _mvCnt->SetStrOverrideMode((OVERRIDE_MODE)omode);
}
*/
void MvCnt::SetStrTorque(int torque)
{
    _mvCnt->SetStrTorque(torque);
}

void MvCnt::SetStrAngle(int angle)
{
    float sndVal = angle;// /10.0f;
    _mvCnt->SetStrAngle(sndVal);
}

void MvCnt::SetStrServo(int servo)
{
    _mvCnt->SetStrServo((MV2_SERVO)servo);
}

void MvCnt::SetDrvMode(int mode)
{
    _mvCnt->SetDrvMode((MV2_MODE)mode);
}

void MvCnt::SetDrvCMode(int cmode)
{
    if((_drvInf.tShift == SHIFT_POS_R) && (cmode == CONT_MODE_VELOCITY))
        return;
    _mvCnt->SetDrvControlMode((DRIVE_CONTROL_MODE)cmode);
}

/*void MvCnt::SetDrvOMode(int omode)
{
    _mvCnt->SetDrvOverrideMode((OVERRIDE_MODE)omode);
}
*/
void MvCnt::SetDrvStroke(int stroke)
{
    _mvCnt->SetDrvStroke(stroke);
}

void MvCnt::SetDrvVeloc(int veloc)
{
    float sndVal = veloc;
    _mvCnt->SetDrvVeloc(sndVal);
}

void MvCnt::SetDrvShiftMode(int shift)
{
    if((_drvInf.cmode == CONT_MODE_VELOCITY) && (shift == SHIFT_POS_R))
        return;
    _mvCnt->SetDrvShiftMode((SHIFT_POSITION)shift);
}

void MvCnt::SetDrvServo(int servo)
{
    _mvCnt->SetDrvServo((MV2_SERVO)servo);
}

void MvCnt::SetBrakeStroke(int stroke)
{
    _mvCnt->SetBrakeStroke(stroke);
}

void MvCnt::SetBrakeLamp(unsigned char lamp)
{
    _mvCnt->SetBrakeLamp(lamp);
}

void MvCnt::SetBlinkLeft(unsigned char left)
{
    _mvCnt->SetBlinkerLeft(left);
}

void MvCnt::SetBlinkRight(unsigned char right)
{
    _mvCnt->SetBlinkerRight(right);
}

void MvCnt::SetBrkAutoCal()
{
    _mvCnt->SetBrkAutoCal();
}

void MvCnt::SetConfig(int index, int gain)
{
    _mvCnt->SetCommConfig(index, gain);
}


void MvCnt::GetErrCode(int* errLevel, int* errCode)
{
    *errCode = _errCode;
    *errLevel = _errLevel;
}

void MvCnt::GetConfig(MV2_CONFIG kind)
{
    _mvCnt->ReqCommConfig((int)kind, 1);
}

void MvCnt::ReqOtherErr()
{
    _mvCnt->ReqOtherErr();
}

void MvCnt::ReqClearErr()
{
    _mvCnt->ReqClearErr();
}

void MvCnt::resetPort()
{
    _mvCnt->ResetCANUSBz(1);
    _mvCnt->ResetCANUSBz(2);
}

void MvCnt::SaveConfig()
{
    _mvCnt->SaveCommConfig();
}

void MvCnt::ReqFirmVersion()
{
    _mvCnt->ReadVersionReq();
}

void MvCnt::SndPCStatus()
{
    _mvCnt->SndPCStatus();
}
