/**
 * @file	SteeringInformation.h
 * @brief	ステアリング情報クラスヘッダファイル
 * @date	2013/03/1
 * @author	sekiguchi
 * @par		Copyright:
 *		2013 ZMP Inc. All rights reserved.
 */
#ifndef STEERINGINFORMATION_H_
#define STEERINGINFORMATION_H_

namespace zmp
{
	namespace mv2
	{
/**
 * @brief ステアリング情報を保持するためのクラス
 * @author	sekiguchi	
 * @date	2013/03/01
 */
class SteeringInformation
{
public:
	SteeringInformation();
	~SteeringInformation();

	int GetMode() const;
	int GetContMode() const;
	int GetOverrideMode() const;
	float GetTargetTorque() const;
	float GetActualTorque() const;
	float GetTargetAngle() const;
	float GetActualAngle() const;
	int GetServo() const;

	void SetRcvMode(int mode);
	void SetRcvContMode(int cont_mode);
	void SetRcvOverrideMode(int override_mode);
	void SetRcvTargetTorque(float input_torque);
	void SetRcvActualTorque(float torque);
	void SetRcvTargetAngle(float angle);
	void SetRcvActualAngle(float input_angle);
	void SetRcvServo(int servo);



private:
	int	_mode;		/*!< @brief ステアリングモード */
	int	_cont_mode;	/*!< @brief ステアリング制御モード */
	int	_override_mode;	/*!< @brief ステアリングオーバライドモード */
	float	_target_torque;	/*!< @brief ステアリング操舵トルク指令値 */
	float	_actual_torque;/*!< @brief ステアリング操舵トルク値 */
	float	_target_angle;	/*!< @brief ステアリング操舵角度指令値 */
	float	_actual_angle;	/*!< @brief ステアリング操舵角度値 */
	int	_is_servo;	/*!< @brief サーボのON/OFF */
};
	}
}

#endif /* STEERINGINFORMATION_H_ */
