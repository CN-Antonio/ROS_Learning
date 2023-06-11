#include "mv2.h"

// using namespace std;
using namespace zmp::mv2;

namespace mv2_driver
{
Mv2Driver::Mv2Driver(ros::NodeHandle node,
          ros::NodeHandle private_nh,
          std::string const & node_name)
{
    /* ROS init */
    pub_ = node.advertise<mv2::status>("mv2/status", 10);
    
    // ros::Rate loop_rate(100);    //控制rate=100.0Hz

    /* mv2 init*/
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

    // for(int i=0; i<40; i++)
    // {
    //     _config.data[i] = 0;
    // }

    /* mv2 control */
    // init control mode
    mv->SetDrvMode(MODE_MANUAL);
    mv->SetStrMode(MODE_MANUAL);
    // init control value
    _drvTargetVeloc = 0;
    _drvTargetStroke = 0;
    _brkTargetStroke = 0;
    _strTargetAngle = 0;
    _strTargetTorque = 0;

    _selectLog.start = false;

    // _gameData.angle = 0;
    // _gameData.brake = 0;
    // _gameData.button = 0;
    // _gameData.drive = 0;

    _visionDistMean = 0;
    _visionDistMin = 0;
    _visionPosX1 = 0;
    _visionPosX2 = 0;
    _visionPosY1 = 0;
    _visionPosY2 = 0;

    // _gameEnable = false;
    // _gameRes = _Game.GameInit();
    // _Game.SetGameReceiveHandler(this);
    // _Game.GameStart();
    
    // callback
    // mv->SetConfigCallback(this);

}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool Mv2Driver::poll(void)
{
    // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    mv2::statusPtr status(new mv2::status);

    // publish message using time of last packet read
    ROS_DEBUG("Publishing a full vehicle status.");

    return true;
}

Mv2Driver::~Mv2Driver()
{
    // set all mode "manual"
    mv->Stop();
    mv->Close();
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

} // namespace mv2_driver