/**
 * @file	BrakeInformation.h
 * @brief	ブレーキ情報クラスヘッダファイル  刹车信息记录
 * @date	2013/03/1
 * @author	sekiguchi
 * @par		Copyright:
 *		2013 ZMP Inc. All rights reserved.
 */
#ifndef BRAKEINFORMATION_H_
#define BRAKEINFORMATION_H_

namespace zmp
{
	namespace mv2 
	{
/**
 * @brief ブレーキ情報を保持するためのクラス   保存刹车类别
 * @author	sekiguchi	
 * @date	2013/03/1
 */
class BrakeInformation
{
public:
	BrakeInformation();
	virtual ~BrakeInformation();

	int GetInputStroke() const;
	int GetActualStroke() const;
	int GetTargetStroke() const;
	unsigned char GetBrakeLamp() const;
	unsigned char GetBlinkLeft() const;
	unsigned char GetBlinkRight() const;
//	bool GetPressed() const;
			
	void SetRcvInputStroke(int stroke);
	void SetRcvActualStroke(int stroke);
	void SetRcvTargetStroke(int stroke);
	void SetRcvBrakeLamp(unsigned char lamp);
	void SetRcvBlinkLeft(unsigned char left);
	void SetRcvBlinkRight(unsigned char right);
//	void SetRcvPressed(bool pressed);

private:
	int	_input_stroke;	/*!< @brief ブレーキペダルからの入力値 */ //来自刹车踏板的输入值
	int	_target_stroke;	/*!< @brief ペダルストローク指令値 */   //  
	int	_actual_stroke;	/*!< @brief ペダルストロークセンサ値 */
	unsigned char _brake_lamp;	/*!< @brief ダルストロークセンサ値 */
	unsigned char _blink_left;	/*!< @brief ペダルストロークセンサ値 */
	unsigned char _blink_right;	/*!< @brief ペダルストロークセンサ値 */
//	bool	_brake_pressed;	/*!< @brief ブレーキセンサのONOFF */
};
	}
}

#endif /* BRAKEINFORMATION_H_ */
