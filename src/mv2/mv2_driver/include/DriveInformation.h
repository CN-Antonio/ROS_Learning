/**
 * @file	DriveInformation.h
 * @brief	ドライブ情報クラスヘッダファイル
 * @date	2013/03/1
 * @author	sekiguchi
 * @par		Copyright:
 *		2013 ZMP Inc. All rights reserved.
 */
#ifndef DRIVEINFORMATION_H_
#define DRIVEINFORMATION_H_

#include "MV2Const.h"

namespace zmp
{
	namespace mv2
	{
/**
 * @brief ドライブ情報を保持するためのクラス
 *
 * @author	sekiguchi
 * @date	2013/03/1
		 */
class DriveInformation
{
public:
	DriveInformation();
	virtual ~DriveInformation();

	void SetRcvServo(int servo);
	void SetRcvMode(int mode);
	void SetRcvContMode(int cont_mode);
	void SetRcvOverrideMode(int override_mode);
	void SetRcvInputShift(int shift);
	void SetRcvTargetShift(int shift);
	void SetRcvActualShift(int shift);
	void SetRcvInputStroke(int stroke);
	void SetRcvTargetStroke(int stroke);
	void SetRcvActualStroke(int stroke);
	void SetRcvTargetVeloc(float veloc);
	void SetRcvActualVeloc(float veloc);
	void SetRcvVelocFR(int veloc);
	void SetRcvVelocFL(int veloc);
	void SetRcvVelocRR(int veloc);
	void SetRcvVelocRL(int veloc);

	int GetServo() const;
	int GetMode() const;
	int GetContMode() const;
	int GetOverrideMode() const;
	int GetInputShift() const;
	int GetTargetShift() const;
	int GetActualShift() const;
	int GetInputStroke() const;
	int GetTargetStroke() const;
	int GetActualStroke() const;
	float GetTargetVeloc() const;
	float GetActualVeloc() const;
	int GetVelocFR() const;
	int GetVelocFL() const;
	int GetVelocRR() const;
	int GetVelocRL() const;

	private:
	int	_is_servo;	/*!< @brief サーボのON/OFF */
	int	_mode;		/*!< @brief ドライブモード */
	int	_cont_mode;	/*!< @brief ドライブ制御モード */
	int	_override_mode;	/*!< @brief ドライブオーバーライドモード */
	int	_input_shift;	/*!< @brief シフトレバーからの入力値 */
	int	_target_shift;	/*!< @brief シフト指令値 */
	int	_actual_shift;	/*!< @brief シフト位置 */
	int	_input_stroke;	/*!< @brief アクセルペダルからの入力値 */
	int	_target_stroke; /*!< @brief アクセルペダルストローク指令値 */
	int	_actual_stroke;	/*!< @brief アクセルペダルストロークセンサ値 */
	float	_target_veloc;	/*!< @brief 速度指令値 */
	float	_actual_veloc;	/*!< @brief 速度 */
	int	_veloc_fr;	/*!< @brief 右前ホイール速度 */
	int	_veloc_fl;	/*!< @brief 左前ホイール速度 */
	int	_veloc_rr;	/*!< @brief 右後ホイール速度 */
	int	_veloc_rl;	/*!< @brief 左後ホイール速度 */
};
	}
}

#endif /* DRIVEINFORMATION_H_ */
