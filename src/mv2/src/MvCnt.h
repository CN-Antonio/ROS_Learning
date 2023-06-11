#ifndef MVCONT_H_
#define MVCONT_H_

#include "Mv2Control.h"
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>

using namespace std;
using namespace zmp::mv2;
using namespace zmp::hev;

#define USE_DEMO 1

// 車両情報
struct VehicleInf {
    float soc;				// バッテリ残容量[%]
    float veloc;			// 車両速度[km/h]
    float battTemp;			// 車両バッテリ温度[℃]
    int meterVeloc;			// 車両メータ速度[km/h]
    int battLevel;			// 充電レベル[0〜8]
    int odometer;			// 車両odometer
    float trip;				// 車両trip
};

// バッテリー情報
struct BattInf {
    float mCurrent;     	// Mainバッテリ電流[A]
    float sCurrent;      	// Subバッテリ電流[A]
    float mVoltage;     	// Mainバッテリ電圧[V]
    float sVoltage;      	// Subバッテリ電圧[V]
};

// ブレーキ情報
struct BrakeInf {
    int iStroke;			// ペダルストローク入力値
    int tStroke;			// ペダルストローク目標値
    int aStroke;			// ペダルストローク現在値
	unsigned char lamp;		// ブレーキランプ
	unsigned char blinkL;	// 左ウィンカー
	unsigned char blinkR;	// 右ウィンカー
};

// その他情報
struct OtherInf {
    int errLevel;          // コントローラエラーレベル
    int errCode;           // コントローラエラーコード
    int ignition;           // ignitionスイッチ状態
    int emergency;          // emergencyスイッチ状態
    int adcStatus;
    int watchdogStatus;
};

// ドライブ情報
struct DrvInf {
    int mode;               // ドライブモード(manual=0x00, program=0x10)
    int cmode;           	// ドライブ制御モード(velocity=0x00, stroke=0x10)
    int omode;				// オーバーライドモード(ON=0x00, OFF=0x10)
    int servo;              // 制御のON/OFF(ON=true, OFF=false)
    int iShift;         	// シフトポジション入力値
    int tShift;				// シフトポジション目標値
    int aShift; 	       	// シフトポジション現在値
    int iStroke;			// ペダルストローク入力値
    int tStroke;			// ペダルストローク目標値
    int aStroke;			// ペダルストローク現在値
    float tVeloc;			// 目標速度[km/h*100]
    float aVeloc;      		// 現在速度[km/h*100]
    int fr;					// 右前輪エンコーダカウント
    int fl;           		// 左前輪エンコーダカウント
    int rr;           		// 右後輪エンコーダカウント
    int rl;           		// 左後輪エンコーダカウント
};

// ステアリング情報
struct StrInf {
    int mode;           // ステアリングモード(manual=0x00, program=0x10)
    int cmode;      // ステアリング制御モード(torque=0x00, angle=0x10)
    int omode;   // オーバーライドモード(ON=0x00, OFF=0x10)
    int servo;          // 制御のON/OFF(ON=true, OFF=false)
    int tTorque;   		// 目標トルク
    int aTorque;   		// 操舵トルク
    float aAngle;  		// 操舵角度[deg * 10]
    float tAngle;  		// 目標操舵角[deg*10]
};

struct ImuInf{
    double accX;
    double accY;
    double accZ;

    double gyroX;
    double gyroY;
    double gyroZ;

    double compX;
    double compY;
    double compZ;

    IMUZ2_ROLE role;
    int period;
    zmp::mv2::RANGE_ACC rangeAcc;
    zmp::mv2::RANGE_GYRO rangeGyro;
    zmp::mv2::RANGE_COMP rangeComp;
    float battLevel;
    IMUZ2_FORMAT format;

    unsigned char hardVersion;
    unsigned char firmVersion;
};

struct IncInf{
    double accX;
    double accY;
    double angleX;
    double angleY;

    short period;
    float battLevel;
    short mode;
};

struct PosInf{
    GPGGA_DATA gga;

    float humid;
    float hTemp;

    float press;
    float pTemp;
};


// コンフィグ情報
struct ConfigInf {
    int data[40];
};

struct MWInf {
    MILLIWAVE_DATA data[20];
};

struct selectLogInf{
    bool start;
    bool drvInf;
    bool brkInf;
    bool strInf;
    bool battInf;
    bool vehicleInf;
    bool switchInf;
    bool errorInf;
    bool imuInf;
    bool posInf;
    bool incInf;
    bool milliInf;
};

class ChangeConfig
{
public:
    virtual void UpdateConfig(int num, int index, int data[]) = 0; // コンフィグ情報受信コールバック関数
};

class MvCnt : public ChangeStateObserver
{
public:
    MvCnt();
    virtual ~MvCnt();

    bool Init();                            // Mv2制御クラス初期化
    bool Start();                           // Mv2制御クラススタート
//    bool Process();                         //
    bool Stop();                            // Mv2制御クラス停止
    bool Close();                           // Mv2制御クラスクローズ
    bool SetConfigCallback(ChangeConfig* callback); // コンフィグ情報受信コールバック関数ハンドラ設定

    void GetBattInf(BattInf* batt);         // バッテリー情報取得
    void GetBrakeInf(BrakeInf* brake);      // ブレーキ情報取得
    void GetOtherInf(OtherInf* other);      // その他情報取得
    void GetDrvInf(DrvInf* drv);            // ドライブ情報取得
    void GetStrInf(StrInf* str);            // ステアリング情報取得
    void GetErrCode(int* leve, int* err);   // エラー情報取得
    void GetVehicleInf(VehicleInf* vehicle);// 車両情報取得
    void GetImuInf(ImuInf* imu);            // Imuセンサ情報取得
    void GetIncInf(IncInf* inc);
    void GetPosInf(PosInf* pos);
    void GetMWInf(MWInf *mwInf);
    void GetFirmVersion(int* version);

    // Set Steer
    void SetStrMode(int mode);              // モード設定(0=manual 1=program)
    void SetStrCMode(int cmode);            // 制御モード設定(0=angle 1=torque)
//    void SetStrOMode(int omode);            // オーバーライドモード設定(0=OFF 1=ON)
    void SetStrTorque(int torque);          // ステアリングトルク設定(-4096〜4095)
    void SetStrAngle(int angle);            // ステアリング角度設定(-660〜660[deg])
    void SetStrServo(int servo);            // ステアリングサーボ設定(0=OFF 1=ON)

    // Set Drive
    void SetDrvMode(int mode);              // モード設定(0=manual 1=program)
    void SetDrvCMode(int cmode);            // 制御モード設定(0=velocity 1=stroke)
//    void SetDrvOMode(int omode);            // オーバーライドモード設定(0=OFF 1=ON)
    void SetDrvStroke(int stroke);          // ドライブストローク設定(0〜4095)
    void SetDrvVeloc(int veloc);            // ドライブ速度設定(0〜60[Km/h])
    void SetDrvShiftMode(int shift);        // ドライブシフト設定(0=N 1=D 2=R)
    void SetDrvServo(int servo);            // ドライブサーボ設定(0=OFF 1=ON)

    // Set Brake
    void SetBrakeStroke(int stroke);        // ブレーキストローク設定(0〜4095)
    void SetBrakeLamp(unsigned char lamp);
    void SetBlinkRight(unsigned char blinkR);
    void SetBlinkLeft(unsigned char blinkL);
    void SetBrkAutoCal();

    // Set Other
//    void SetDist(float dist);
//    void SetTargetAngle(int target);
    void ReqOtherErr();
    void ReqClearErr();

    void resetPort();

    void GetConfig(MV2_CONFIG kind);            // コンフィグデータ取得
    void SetConfig(int index, int gain);       // コンフィグデータ設定
    void SaveConfig();                          // コンフィグ保存
    void ReqFirmVersion();

    void SndPCStatus();

private:
    // ステアリング情報受信コールバック関数
    void UpdateSteerState(REP_STEER_INFO_INDEX index);
    // ドライブ情報受信コールバック関数
    void UpdateDriveState(REP_DRIVE_INFO_INDEX index);
    // バッテリ情報受信コールバック関数
    void UpdateBattState(REP_BATT_INFO_INDEX index);
    // その他情報受信コールバック関数
    void UpdateOtherState(REP_OTHER_INFO_INDEX index);
    // 車両情報受信コールバック関数
    void UpdateVehicleState(REP_VEHICLE_INFO_INDEX index);
    // コンフィグ情報受信コールバック関数
    void ReceiveConfig(int num, int index, int value[]);
    // エラー情報受信コールバック関数
    void ReceiveError(int leve, int errCode);
    // ECHO受信コールバック関数
    void ReceiveEcho(int ctlKind, int ctlNo);
    // IMUZ2情報受信コールバック関数
    void UpdateImuz2State(REP_IMUZ2_INFO_INDEX index);

    // Position-Z情報通知コールバック関数
    void UpdatePoszState(REP_POSZ_INFO_INDEX index);

    // IncZ情報通知コールバック関数
    void UpdateInczState(REP_INCZ_INFO_INDEX index);

    void UpdateMilliWave(int index);

    void ReceiveVersion(int version);
//    void rcvTime(char* date);
//    void getDate();

    Mv2Control* _mvCnt;         // Mv2コントロールクラス
    zmp::hev::CANUSBZ* _canCom;  // CAN通信クラス
//    zmp::hev::CANUSBZ* _canCom2;  // CAN通信クラス
    ChangeConfig* _callback;    // コンフィグ受信コールバックハンドラ

    int _errCode;               // Mv2システムエラーコード
    int _errLevel;              // Mv2システムエラーレベル
    BattInf _battInf;           // バッテリ情報格納エリア
    BrakeInf _brakeInf;         // ブレーキ情報格納エリア
    OtherInf _otherInf;         // その他情報格納エリア
    DrvInf _drvInf;             // ドライブ情報格納エリア
    StrInf _strInf;             // ステアリング情報格納エリア
    ConfigInf _config;          // コンフィグ情報格納エリア
    VehicleInf _vehicleInf;     // 車両情報格納エリア
    ImuInf      _imuInf;        // IMUセンサ情報格納エリア
    IncInf      _incInf;
    PosInf      _posInf;
    MWInf _mwInf;

    selectLogInf _selectLog;
    int _firmVersion;


//    int _beforAngle;
//    int _targetCnt;
//    int _targetAngle;

//    int _asistTrq;              // ステアリングアシストトルク
};


#endif /* MVCONT_H_ */ 
