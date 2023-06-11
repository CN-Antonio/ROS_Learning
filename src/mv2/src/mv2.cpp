#include "mv2.h"

// using namespace std;
using namespace zmp::mv2;

namespace mv2_driver
{
Mv2Driver::Mv2Driver(ros::NodeHandle node,
          ros::NodeHandle private_nh,
          std::string const & node_name)
{
    mv = new MvCnt();
    mv->Init();
    mv->Start();
    ROS_INFO_STREAM("mv2 init");

    _comSvr = new CommServer();
    _comApp = new CommApp();
    _comApp->Init();
    _comApp->SetCommServer(_comSvr);
    _comApp->SetMvCnt(mv);
    _comApp->Start();
    ROS_INFO_STREAM("comSvr/App started, able to communicate to vehicle");

    _drvTargetVeloc = 0;
    _drvTargetStroke = 0;
    _brkTargetStroke = 0;
    _strTargetAngle = 0;
    _strTargetTorque = 0;

    _selectLog.start = false;

    for(int i=0; i<40; i++)
    {
        _config.data[i] = 0;
    }

    /* mv2 control */
    // TODO
}

Mv2Driver::~Mv2Driver()
{
  // TODO: set all mode "manual"
}

void Mv2Driver::GetTimeStamp(char* date) // TODO: 
{
    // time(&_day_time);
    // _s_time = gmtime(&_day_time);
    // gettimeofday(&_getTime, NULL);
    // sprintf(date, "%04d_%02d_%02d_%02d_%02d_%02d.%06ld", 
    //     _s_time->tm_year+1900, 
    //     _s_time->tm_mon+1,
    //     _s_time->tm_mday,
    //     _s_time->tm_hour,
    //     _s_time->tm_min,
    //     _s_time->tm_sec,
    //     _getTime.tv_usec);              
    // return;
}

/////////////////////////////////////////////////////////////////////////////
// drive mode
/////////////////////////////////////////////////////////////////////////////
// drive mode manual
void Mv2Driver::sndDrvModeM()
{
    mv->SetDrvMode(MODE_MANUAL);
    //ui->horizontalSlider_drvStroke->setValue(0);
    //ui->horizontalSlider_drvVeloc->setValue(0);
    // ui->scrollBar_accStroke->setValue(0);
    // ui->scrollBar_drvVeloc->setValue(0);
    _drvTargetVeloc = 0;
    _drvTargetStroke = 0;
}

void Mv2Driver::readInfTimer()
{
    mv->GetBattInf(&_battInf);
    mv->GetBrakeInf(&_brakeInf);
    mv->GetDrvInf(&_drvInf);
    mv->GetOtherInf(&_otherInf);
    mv->GetStrInf(&_strInf);
    mv->GetErrCode(&_errLevel, &_errCode);
    mv->GetVehicleInf(&_vehicleInf);
    mv->GetImuInf(&_imuInf);
    mv->GetPosInf(&_posInf);
    mv->GetIncInf(&_incInf);
    mv->GetMWInf(&_mwInf);
    mv->GetFirmVersion(&_firmVersion);

    _comApp->Process();
    _comApp->GetDistMean(&_visionDistMean);
    _comApp->GetDistMin(&_visionDistMin);
    _comApp->GetDistPos(&_visionPosX1, &_visionPosX2, 
			&_visionPosY1, &_visionPosY2);

    // TODO: function to replace
    // viewBattInf();
    // viewBrakeInf();
    // viewDrvInf();
    // viewOtherInf();
    // viewStrInf();
    // viewErrCode();
    // viewImuInf();
    // viewPosInf();
    // viewIncInf();
    // viewMWInf();
    // viewFirmVersion();

    // viewVision2Inf();

//    if(_drvInf.mode == MODE_PROGRAM || _strInf.mode == MODE_PROGRAM){
        mv->SndPCStatus();
//    }
}
}