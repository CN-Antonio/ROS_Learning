#include "mv2.h"

// using namespace std;
using namespace zmp::mv2;

namespace mv2_driver
{

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

    viewBattInf();
    viewBrakeInf();
    viewDrvInf();
    viewOtherInf();
    viewStrInf();
    viewErrCode();
    viewImuInf();
    viewPosInf();
    viewIncInf();
    viewMWInf();
    viewFirmVersion();

    viewVision2Inf();

//    if(_drvInf.mode == MODE_PROGRAM || _strInf.mode == MODE_PROGRAM){
        mv->SndPCStatus();
//    }
}
}