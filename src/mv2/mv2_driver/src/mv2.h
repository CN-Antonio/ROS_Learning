#ifndef MV2_H
#define MV2_H

// cpp
#include <iostream>
// #include <chrono>
// #include <memory>
#include <string>

// ROS
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "mv2_msgs/control.h"
#include "mv2_msgs/status.h"

// ROS2
// #include <rclcpp/rclcpp.hpp>
// #include "std_msgs/msg/string.hpp"

// mv2
// #include "GameControl.h"
#include "MvCnt.h"
#include "CommApp.h"
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <pthread.h>

namespace mv2_driver {
class Mv2Driver
{
public:
    Mv2Driver(ros::NodeHandle node,
              ros::NodeHandle private_nh,
              std::string const & node_name = ros::this_node::getName());
    ~Mv2Driver();

    bool poll(void);

protected:
    void closeEvent(/* QCloseEvent * */);

private:
/* ROS */
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    // mv2_msgs::status statusMsg;
    mv2_msgs::control ctrlMsg;

    void controlMsgCb(const mv2_msgs::control::ConstPtr &msg);

/* private slots */
    void ReqOtherErr();
    void ReqClearErr();

    void resetPort();

// コンフィグ情報受信
    void GetConfStrGainKP();        // ステアリング角度制御ゲイン(Kp)
    void GetConfStrEncOffset();     // エンコーダーオフセット
    void GetConfStrStrokeLimitP();  // ストロークリミットプラス側
    void GetConfStrStrokeLimitM();  // ストロークリミットマイナス側

    void GetConfVelocMaxSpeed();    // 最大速度
    void GetConfVelocGainKPP();     // 速度制御ゲインKp プラス側
    void GetConfVelocGainKPM();     // 速度制御ゲインKp マイナス側
    void GetConfVelocWindowP();     // ウィンドウ幅 プラス側
    void GetConfVelocWindowM();     // ウィンドウ幅 マイナス側
    void GetConfVelocPunchP();      // パンチ プラス側
    void GetConfVelocPunchM();      // パンチ マイナス側
    void GetConfVelocMaxOutputP();  // 最大アクセル出力
    void GetConfVelocMaxOutputM();  // 最大ブレーキ出力

    void GetConfBrkMotionMaxAcc();  // 最大加速度
    void GetConfBrkEncOffset();     // エンコーダーオフセット
    void GetConfBrkStrokeLimitP();  // ストロークリミットプラス側
    void GetConfBrkStrokeLimitM();  // ストロークリミットマイナス側

    void GetConfStrOverrideTh();    // ステアリングオーバーライド閾値
    void GetConfAccOverrideTh();    // アクセルオーバーライド閾値
    void GetConfBrkOverrideTh();    // ブレーキオーバーライド閾値
    void GetConfBrkPosMin();        // brake position min
    void GetConfBrkPosMax();        // brake position max

    void GetConfDrvOverrideEnable();
    void GetConfStrOverrideEnable();

    void GetConfChkClientConnect();

    void SetConfStrGainKP();        // ステアリング角度制御ゲイン(Kp)
    void SetConfStrEncOffset();     // エンコーダーオフセット
    void SetConfStrStrokeLimitP();  // ストロークリミットプラス側
    void SetConfStrStrokeLimitM();  // ストロークリミットマイナス側

    void SetConfVelocMaxSpeed();    // 最大速度
    void SetConfVelocGainKPP();     // 速度制御ゲインKp プラス側
    void SetConfVelocGainKPM();     // 速度制御ゲインKp マイナス側
    void SetConfVelocWindowP();     // ウィンドウ幅 プラス側
    void SetConfVelocWindowM();     // ウィンドウ幅 マイナス側
    void SetConfVelocPunchP();      // パンチ プラス側
    void SetConfVelocPunchM();      // パンチ マイナス
    void SetConfVelocMaxOutputP();  // 最大アクセル出力
    void SetConfVelocMaxOutputM();  // 最大ブレーキ出力

    void SetConfBrkMotionMaxAcc();  // 最大加速度
    void SetConfBrkEncOffset();     // エンコーダーオフセット
    void SetConfBrkStrokeLimitP();  // ストロークリミットプラス側
    void SetConfBrkStrokeLimitM();  // ストロークリミットマイナス側
    void SetConfBrkPosMin();        // brake position min
    void SetConfBrkPosMax();        // brake position max

    void SetConfStrOverrideTh();    // ステアリングオーバーライド閾値
    void SetConfAccOverrideTh();    // アクセルオーバーライド閾値
    void SetConfBrkOverrideTh();    // ブレーキオーバーライド閾値

    void SetConfDrvOverrideEnable();
    void SetConfStrOverrideEnable();

    void SetConfChkClientConnect();


    void SaveConfig();              // コンフィグ情報保存
    void GetConfAll();
    void SetConfBrkAutoCal();
    void GetFirmVersion();

    // Timer loop
    void readInfTimer();            // データ読み出しタイマー処理
    void sndDrvTargetTimer();       // ターゲット veloc/torque 送信タイマー処理
	void sndInfToPCTimer();			// 各種データの送信タイマー(to PC)

    void startLog();                // ログ保存開始処理
    void stopLog();                 // ログ保存終了処理

    // Game slots
    void setGameEnable();           // ゲームコントローラ enable/disable 設定

    void GetTimeStamp(char* date);

    MvCnt* mv;              // Mv2制御ライブラリクラス
    BattInf _battInf;       // バッテリ情報
    BrakeInf _brakeInf;     // ブレーキ情報
    OtherInf _otherInf;     // その他情報
    DrvInf _drvInf;         // ドライブ情報
    StrInf _strInf;         // ステアリング情報
    ConfigInf _config;      // コンフィグ情報
    VehicleInf _vehicleInf; // 車両情報
    ImuInf _imuInf;         // IMUセンサ情報
    PosInf _posInf;
    IncInf _incInf;
    MWInf _mwInf;
    CommServer* _comSvr;
    CommApp* _comApp;

    int _errCode;           // エラーコード
    int _errLevel;          // エラーレベル
    int _firmVersion;

    selectLogInf _selectLog;

    double _visionDistMean;
    double _visionDistMin;
    int _visionPosX1;
    int _visionPosX2;
    int _visionPosY1;
    int _visionPosY2;
    
    int _drvTargetVeloc;    // タイマー処理で送信するターゲット速度
    int _drvTargetStroke;   // タイマー処理で送信するターゲットストローク
    int _brkTargetStroke;
    int _strTargetTorque;
    int _strTargetAngle;
};

} // namespace mv2_driver

#endif