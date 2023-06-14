/**
 * @file	Mv2Control.h
 * @brief	MV2制御クラスヘッダファイル
 * @date	2013/03/04
 * @author	sekiguchi
 * @par		Copyright:
 *		2013 ZMP Inc. rights reserved.
 */
#ifndef MV2CONTROL_H_
#define MV2CONTROL_H_

#include <math.h>
#include "DriveInformation.h"
#include "SteeringInformation.h"
#include "BrakeInformation.h"
#include "BatteryInformation.h"
#include "OtherInformation.h"
#include "VehicleInformation.h"
#include "Imuz2Information.h"
#include "PoszInf.h"
#include "InclinoInf.h"
#include "Mv2ControlCommon.h"
#include "MV2Const.h"
//#include "CANConst.h"
//#include "CanCommunication.h"
#include "CANUSB.h"
#include "MilliWaveInf.h"

/*! @brief MV2接続チャンネルの定義 */
#define CAN_CHANNEL_VEHICLE	CAN_CHANNEL_0	// 500kbps

/*! @brief コントローラ接続チャンネルの定義 */
#define CAN_CHANNEL_CONTROL	CAN_CHANNEL_1	// 1000kbps

namespace zmp
{
	namespace mv2
	{
/**
 * @brief MV2の状態変化通知コールバック関数を提供するオブザーバクラス
 *
 * @author	sekiguchi	
 * @date	2013/03/04
 */
	  class ChangeStateObserver
{
public:
	/**
	 * @brief	ステアリング状態通知コールバック関数
	 * @param	index 通知インデックス
	 */
	virtual void UpdateSteerState(REP_STEER_INFO_INDEX index) = 0;

	/**
	 * @brief	ドライブ状態通知コールバック関数
	 * @param	index 通知インデックス
	 */
	virtual void UpdateDriveState(REP_DRIVE_INFO_INDEX index) = 0;

	/**
	 * @brief	バッテリ状態通知コールバック関数
	 * @param	index 通知インデックス
	 */
	virtual void UpdateBattState(REP_BATT_INFO_INDEX index) = 0;

	/**
	 * @brief	他状態通知コールバック関数
	 * @param	index 通知インデックス
	 */
	virtual void UpdateOtherState(REP_OTHER_INFO_INDEX index) = 0;

	/**
	 * @brief	車両情報通知コールバック関数
	 * @param	index 通知インデックス
	 */
	virtual void UpdateVehicleState(REP_VEHICLE_INFO_INDEX index) = 0;

	virtual void UpdateMilliWave(int index) = 0;

	/**
	 * @brief	IMU-Z2情報通知コールバック関数
	 * @param	index 通知インデックス
	 */
	virtual void UpdateImuz2State(REP_IMUZ2_INFO_INDEX index) = 0;

	/**
	 * @brief	Position-Z情報通知コールバック関数
	 * @param	index 通知インデックス
	 */
	virtual void UpdatePoszState(REP_POSZ_INFO_INDEX index) = 0;

	/**
	 * @brief	コンフィグ応答コールバック関数
	 * @param	num コンフィグ数
	 * @param	index コンフィグの開始INDEX
	 * @param	value[] コンフィグ
	 */
	virtual void ReceiveConfig(int num, int index, int value[]) = 0;

	/**
	 * @brief	エラー通知コールバック関数
	 * @param	level エラーレベル 
	 * @param	code エラーコード
	 */
	virtual void ReceiveError(int level, int code) = 0;
	virtual void UpdateInczState(REP_INCZ_INFO_INDEX index) = 0;
	virtual void ReceiveVersion(int version) = 0;
};

/**
 * @brief MV2をコントロールするためのクラス
 *
 * CANを介した情報取得
 *
 * @author	sekiguchi	
 * @date	2013/03/04
 */
class Mv2Control : public hev::SerialReceiveHandler
{
public:
	/** コンストラクタ */
	Mv2Control();

	/** デストラクタ */
	virtual ~Mv2Control();

	/**
	 * @brief  Mv2Controlクラス初期化処理
	 * 	   CANUSBZクラスの設定を行う
	 * @param[in]	can_comm CANUSBZクラス
	 * @retval	0 成功
	 * @retval	0以外 失敗
	 */
	//int InitMv2Control(CanCommunication* can_comm);
	int InitMv2Control(hev::CANUSBZ* can_comm);
//	int InitMv2Control(hev::CANUSBZ* can_comm, hev::CANUSBZ* can_comm2);

	int ResetCANUSBz(int index);	

	/**
	 * @brief  状態変化通知コールバック関数のハンドラ登録
	 * @param[in]	callback コールバックハンドラ
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int SetStatusCallback(ChangeStateObserver* callback);

	/**
	 * @brief  状態変化通知コールバック関数のハンドラ削除
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int RemoveStatusCallback();

	/**
	 * @brief  ステアリングモード取得
	 * @param[out]	mode モード(MODE_MANUAL or MODE_PROGRAM)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetStrMode(int& mode);

	/**
	 * @brief  ステアリング制御モード取得
	 * @param[out]	cont_mode 制御モード(CONT_MODE_TORQUE or CONT_MODE_ANGLE)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetStrControlMode(int& cont_mode);
	
	/**
	 * @brief  ステアリングオーバーライドモード取得
	 * @param[out]	override_mode オーバーライドモード(OVERRIDE_MODE_ON or OVERRIDE_MODE_OFF)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetStrOverrideMode(int& override_mode);

	/**
	 * @brief  ステアリングtargetトルク取得
	 * @param[out]	target_torque targetトルク(-4096〜4095)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetStrTargetTorque(int& target_torque);

	/**
	 * @brief  ステアリングactualトルク取得
	 * @param[out]	torque actualトルク(-4096〜4095)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetStrActualTorque(int& torque);

	/**
	 * @brief  ステアリングtarget角度取得
	 * @param[out]	torque_angle target角度(-660.0〜660.0[deg])
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetStrTargetAngle(float& target_angle);

	/**
	 * @brief  ステアリングactual角度取得
	 * @param[out]	angle actual角度(-660.0〜660.0[deg])
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetStrActualAngle(float& angle);

	/**
	 * @brief  ステアリングサーボ取得
	 * @param[out]	servo サーボ状態(SERVO_ON or SERVO_OFF)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetStrServo(int& servo);

	/**
	 * @brief  ドライブサーボ取得
	 * @param[out]	servo サーボ状態(SERVO_ON or SERVO_OFF)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetDrvServo(int& servo);

	/**
	 * @brief  ドライブモード取得
	 * @param[out]	mode モード(MODE_MANUAL or MODE_PROGRAM)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetDrvMode(int& mode);

	/**
	 * @brief  ドライブ制御モード取得
	 * @param[out]	cont_mode 制御モード(CONT_MODE_VELOCITY or CONT_MODE_STROKE)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetDrvControlMode(int& cont_mode);

	/**
	 * @brief  ドライブオーバーライドモード取得
	 * @param[out]	override_mode オーバーライドモード(OVERRIDE_MODE_ON or OVERRIDE_MODE_OFF)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetDrvOverrideMode(int& override_mode);

	/**
	 * @brief  ドライブtargetシフト取得
	 * @param[out]  shift targetシフト(SHIFT_POS_D or SHIFT_POS_N or SHIFT_POS_R)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetDrvTargetShift(int& shift);

	/**
	 * @brief  ドライブactualシフト取得
	 * @param[out]  shift actualシフト(SHIFT_POS_D or SHIFT_POS_N or SHIFT_POS_R)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetDrvActualShift(int& shift);

	/**
	 * @brief  ドライブinputシフト取得
	 * @param[out]  shift inputシフト(SHIFT_POS_D or SHIFT_POS_N or SHIFT_POS_R)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetDrvInputShift(int& shift);

	/**
	 * @brief  ドライブtargetストローク取得
	 * @param[out]  target_stroke targetストローク(0〜4095)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetDrvTargetStroke(int& target_stroke);

	/**
	 * @brief  ドライブactualストローク取得
	 * @param[out]  actual_stroke actualストローク(0〜4095)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetDrvActualStroke(int& actual_stroke);

	/**
	 * @brief  ドライブinputストローク取得
	 * @param[out]  input_stroke inputストローク(0〜4095)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetDrvInputStroke(int& input_stroke);

	/**
	 * @brief  ドライブtarget速度取得
	 * @param[out]  target_veloc targe速度(0〜60[Km/h])
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetDrvTargetVeloc(float& target_veloc);

	/**
	 * @brief  ドライブactual速度取得
	 * @param[out]  veloc actual速度(0〜60[Km/h])
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetDrvActualVeloc(float& veloc);

	/**
	 * @brief  ドライブWheel速度取得
	 * @param[out]  veloc_fr 右前輪エンコーダカウント(0〜4095)
	 * @param[out]  veloc_fl 左前輪エンコーダカウント(0〜4095)
	 * @param[out]  veloc_rr 右後輪エンコーダカウント(0〜4095)
	 * @param[out]  veloc_rl 右後輪エンコーダカウント(0〜4095)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetDrvWheelVeloc(int& veloc_fr, int& veloc_fl, int& veloc_rr, int& veloc_rl);

	/**
	 * @brief  ドライブWheel速度取得(右前輪)
	 * @param[out]  fr 右前輪エンコーダカウント(0〜4095)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetDrvVelocFR(float& fr);

	/**
	 * @brief  ドライブWheel速度取得(左前輪)
	 * @param[out]  fl 左前輪エンコーダカウント(0〜4095)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetDrvVelocFL(float& fl);

	/**
	 * @brief  ドライブWheel速度取得(右後輪)
	 * @param[out]  rr 右後輪エンコーダカウント(0〜4095)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetDrvVelocRR(float& rr);

	/**
	 * @brief  ドライブWheel速度取得(左後輪)
	 * @param[out]  rl 左後輪エンコーダカウント(0〜4095)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetDrvVelocRL(float& rl);

	/**
	 * @brief　ブレーキactualストローク取得 
	 * @param[out]  stroke actualストローク(0〜4095)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetBrakeActualStroke(int& stroke);

	/**
	 * @brief　ブレーキtargetストローク取得
	 * @param[out]  target_stroke targetストローク(0〜4095)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetBrakeTargetStroke(int& target_stroke);

	/**
	 * @brief　ブレーキinputストローク取得
	 * @param[out]  input_stroke inputストローク(0〜4095)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetBrakeInputStroke(int& input_stroke);

	/**
	 * @brief　ブレーキランプ状態取得
	 * @param[out]  lamp(0=OFF, 1=ON)
	 * @param[out]  left(0=OFF, 1=ON)
	 * @param[out]  right(0=OFF, 1=ON)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetBrakeStatus(unsigned char& lamp, unsigned char& left, unsigned char& right);

	/**
	 * @brief　ブレーキセンサ情報取得
	 * @param[out]  pressed ブレーキセンサ情報(0=OFF, 1=ON)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
//	int GetBrakePressState(bool& pressed);

	/**
	 * @brief　バッテリー情報取得
	 * @param[out]  soc SOC(0〜100[%])
	 * @param[out]  status バッテリ電圧()
	 * @param[out]  chg_current バッテリ電流()
	 * @param[out]  temp バッテリ温度()
	 * @param[out]  level 充電レベル(0〜8)
	 * @param[out]  main_current Mainバッテリ電流()
	 * @param[out]  sub_current Subバッテリ電流()
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
//	int GetBattInfo(int& soc, int& status, int& chg_current, int& temp, int& level, float& main_current, float& sub_current);

	/**
	 * @brief　充電状態取得
	 * @param[out]  SOC 充電状態(0〜100%)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
//	int GetBattSOC(int& soc);
	/**
	 * @brief　バッテリ状態取得
	 * @param[out]  status バッテリ状態()
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
//	int GetBattStatus(int& status);
	/**
	 * @brief　充電電流取得
	 * @param[out]  chg_current 充電電流()
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
//	int GetBattChgCurrent(int& chg_current);
	/**
	 * @brief　バッテリ温度取得
	 * @param[out]  temp バッテリ温度()
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
//	int GetBattTemp(int& temp);
	/**
	 * @brief　充電レベル取得
	 * @param[out]  level 充電レベル(0〜8)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
//	int GetBattLevel(int& level);

	/**
	 * @brief　Mainバッテリ電圧取得
	 * @param[out]  volt メインバッテリ電圧()
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetBattMainVoltage(float& volt);
	/**
	 * @brief　Subバッテリ電圧取得
	 * @param[out]  volt Subバッテリ電圧)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetBattSubVoltage(float& volt);

	/**
	 * @brief　Mainバッテリ電流取得
	 * @param[out]  main_current 充電電流()
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetBattMainCurrent(float& current);
	/**
	 * @brief　Subバッテリ電流取得
	 * @param[out]  current Subバッテリ電流()
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetBattSubCurrent(float& current);

	/**
	 * @brief　ライト情報取得
	 * @param[out]  light ライト状態()
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
//	int GetOtherLightState(LIGHT_STATE& light);
	/**
	 * @brief　エラー情報取得
	 * @param[out]  level エラーレベル()
	 * @param[out]  code エラーコード()
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetOtherErrState(int& level, int& code);

	/**
	 * @brief　スイッチ情報取得
	 * @param[out]  ignition ignition状態(ON/OFF)
	 * @param[out]  emergency emergency状態(ON/OFF)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetOtherSwitchState(int& ignition, int& emergency);

	/**
	 * @brief　ADC状態取得
	 * @param[out]  ADC状態(0x10=Not ready, 0x20=ready)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetOtherAdcStatus(int& adc);

	/**
	 * @brief　Watchdog状態取得
	 * @param[out]  Watchdog状態(0x10=Not ready, 0x20=ready)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetOtherWatchdogStatus(int& watchdog);

	/**
	 * @brief  X軸加速度取得
	 * @param[out]  val X軸加速度[G]
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetImuAccX(double& val);
	/**
	 * @brief  Y軸加速度取得
	 * @param[out]  val Y軸加速度[G]
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetImuAccY(double& val);
	/**
	 * @brief  Z軸加速度取得
	 * @param[out]  val Z軸加速度[G]
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetImuAccZ(double& val);
	/**
	 * @brief  X軸角速度取得
	 * @param[out]  val X軸角速度[G]
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetImuGyroX(double& val);
	/**
	 * @brief  Y軸角速度取得
	 * @param[out]  val Y軸角速度[G]
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetImuGyroY(double& val);
	/**
	 * @brief  Z軸角速度取得
	 * @param[out]  val Z軸角速度[G]
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetImuGyroZ(double& val);
	/**
	 * @brief  X軸地磁気取得
	 * @param[out]  val X軸地磁気[G]
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetImuCompX(double& val);
	/**
	 * @brief  Y軸地磁気取得
	 * @param[out]  val Y軸地磁気[G]
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetImuCompY(double& val);
	/**
	 * @brief  Z軸地磁気取得
	 * @param[out]  val Z軸地磁気[G]
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int GetImuCompZ(double& val);

	int GetImuRole(IMUZ2_ROLE& role);
	int GetImuPeriod(int& period); 
	int GetImuRangeAcc(RANGE_ACC& range);
	int GetImuRangeGyro(RANGE_GYRO& range);
	int GetImuRangeComp(RANGE_COMP& range);
	int GetImuBattLevel(float& vol);
	int GetImuFormat(IMUZ2_FORMAT& format);
	int GetImuHardVersion(unsigned char& version);
	int GetImuFirmVersion(unsigned char& version);

	int GetPoszGPGGA(GPGGA_DATA& data);
	int GetPoszGPZDA(GPZDA_DATA& data);
	int GetPoszGPGLL(GPGLL_DATA& data);
	int GetPoszGPGSA(GPGSA_DATA& data);
	int GetPoszGPGSV(GPGSV_DATA& data);
	int GetPoszGPVTG(GPVTG_DATA& data);
	int GetPoszGPRMC(GPRMC_DATA& data);

	int GetPoszHumidData(float& humid, float& temp);
	int GetPoszPressData(float& press, float& temp);
	int GetInczInf(double& angleX, double& angleY);

	int GetMilliWaveInf(int index, MILLIWAVE_DATA& data);

//	int GetVehicleSysState(int& state);
//	int GetVehicleDrvErrCode(int& err);
//	int GetVehicleChargeState(int& state);
//	int GetVehicleBattErrCode(int& err);
//	int GetVehicleChargeCurrent(float& current);
	int GetVehicleBattSOC(float& soc);
	int GetVehicleBattTemp(float& temp);
	int GetVehicleBattLevel(int& level);
	int GetVehicleVeloc(float& veloc);
	int GetVehicleMeterVeloc(int& veloc);
	int GetVehicleOdometer(int& odometer);
	int GetVehicleTrip(float& trip);

	/**
	 * @brief　コンフィグ情報取得要求
	 * @param[in]  index コンフィグ読み出し位置()
	 * @param[in]  num 読み出しデータ数(1〜3)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int ReqCommConfig(int index, int num);
	/**
	 * @brief  コンフィグ情報保存　
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int SaveCommConfig();
	/**
	 * @brief　エラー情報取得要求
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int ReqOtherErr();

	/**
	 * @brief　エラー情報クリア要求
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int ReqClearErr();

	int ReadVersionReq();

	/**
	 * @brief  ステアリングモード設定　
	 * @param[in]  mode ステアリングモード(MODE_MANUAL or MODE_PROGRAM)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int SetStrMode(MV2_MODE mode);
	/**
	 * @brief  ステアリング制御モード設定　
	 * @param[in]  cont_mode ステアリング制御モード(CONT_MODE_TORQUE or CONT_MODE_ANGLE)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int SetStrControlMode(STEER_CONTROL_MODE cont_mode);
	/**
	 * @brief  ステアリングオーバーライド設定　
	 * @param[in]  override_mode オーバーライドモード(OVERRIDE_MODE_ON or OVERRIDE_MODE_OFF)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
//	int SetStrOverrideMode(OVERRIDE_MODE override_mode);
	/**
	 * @brief  ステアリングトルク設定　
	 * @param[in]  torque ステアリングトルク(-4096〜4095)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int SetStrTorque(int torque);
	/**
	 * @brief  ステアリング角度設定　
	 * @param[in]  torque ステアリン角度(-6300〜6300 1=0.1[deg])
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int SetStrAngle(int angle);
	/**
	 * @brief  ステアリングサーボ設定　
	 * @param[in]  servo ステアリングservo(SERVO_ON or SERVO_OFF)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int SetStrServo(MV2_SERVO servo);

	/**
	 * @brief  ドライブサーボ設定　
	 * @param[in]  servo ドライブservo(SERVO_ON or SERVO_OFF)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int SetDrvServo(MV2_SERVO servo);
	/**
	 * @brief  ドライブモード設定　
	 * @param[in]  mode ドライブモード(MODE_MANUAL or MODE_PROGRAM)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int SetDrvMode(MV2_MODE mode);
	/**
	 * @brief  ドライブ制御モード設定　
	 * @param[in]  cont_mode ドライブ制御モード(CONT_MODE_VELOCITY or CONT_MODE_STROKE)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int SetDrvControlMode(DRIVE_CONTROL_MODE cont_mode);
	/**
	 * @brief  ドライブオーバーライドモード設定　
	 * @param[in]  override_mode ドライブオーバーライドモード(OVERRIDE_MODE_ON or OVERRIDE_MODE_OFF)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
//	int SetDrvOverrideMode(OVERRIDE_MODE override_mode);
	/**
	 * @brief  ドライブシフト設定　
	 * @param[in]	shift ドライブシフト(SHIFT_POS_D or SHIFT_POS_N or SHIFT_POS_R)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int SetDrvShiftMode(SHIFT_POSITION shift);
	/**
	 * @brief  ドライブストローク設定　
	 * @param[in]  stroke ドライブストローク(0〜4095)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int SetDrvStroke(int stroke);
	/**
	 * @brief  ドライブ速度設定　
	 * @param[in]  veloc ドライブ速度(0〜60[km/h])
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int SetDrvVeloc(int veloc);

	/**
	 * @brief  ブレーキストローク設定　
	 * @param[in]  stroke ブレーキストローク(0〜4095)
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int SetBrakeStroke(int stroke);

	int SetBrakeLamp(unsigned char lamp);
	int SetBlinkerLeft(unsigned char blink_left);
	int SetBlinkerRight(unsigned char blink_right);

	/**
	 * @brief  コンフィグ設定　
	 * @param[in]  index コンフィグ書き込み位置()
	 * @param[in]  value 設定値()
	 * retval	0 成功
	 * retval	0以外 失敗
	 */
	int SetCommConfig(int index, int value);

	int SetBrkAutoCal();

	int SndPCStatus();

private:

	void Update(CAN_DEVICE_CHANNEL channel, CANMsg* msg, long long timestamp);
	void OnUSBReceive(int channel, long timeStamp, CANMsg* msg);
	void OnReceiveCANUSB(int channel, long timeStamp, CANMsg* msg);

	int ProcPriorityMessage(int msg_id, CANMsg* msg);
	int ProcDriveMessage(int msg_id, CANMsg* msg);
	int ProcSteerMessage(int msg_id, CANMsg* msg);
	int ProcBrakeMessage(int msg_id, CANMsg* msg);
	int ProcBatteryMessage(int msg_id, CANMsg* msg);
	int ProcSysComMessage(int msg_id, CANMsg* msg);
	int ProcOptionMessage(int msg_id, CANMsg* msg);
	int ProcMessageVehicle(CANMsg* msg);
	int ProcMessageControl(CANMsg* msg);
	int ProcImuz2Message(int msg_id, CANMsg* msg);
	int ProcPoszMessage(int msg_id, CANMsg* msg);
        int ProcInclinoMessage(int msg_id, CANMsg* msg);
	int ProcMilliWave(CANMsg* msg);
	void ProcVehicleMessage(CANMsg* msg);

//	ChangeStateObserver*	_callback;
	//CanCommunication*	_can_comm;
	zmp::hev::CANUSBZ*	_can_comm;
//	zmp::hev::CANUSBZ*	_can_comm2;

	DriveInformation	_drive_inf;
	BrakeInformation	_brake_inf;
	SteeringInformation	_steer_inf;
	OtherInformation	_other_inf;
	BatteryInformation	_batt_inf;
	VehicleInformation	_vehicle_inf;
	Imuz2Information	_imu_inf;
	PoszInf			_posz_inf;
	InclinoInf		_inclino_inf;
	MilliWaveInf		_milliWave_inf;

	int _milliWave_cnt;
	ChangeStateObserver*	_callback;
};
	}
}

#endif /* MV2CONTROL_ */
