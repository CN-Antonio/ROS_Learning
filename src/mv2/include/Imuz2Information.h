/**
 * @file	Imuz2Information.h
 * @brief	IMU-Z2情報クラスヘッダファイル
 *
 * @date	2013/07/31
 * @author	sekiguchi
 * @par		Copyright:
 *		2013 ZMP Inc. All rights reserved.
 */
#ifndef IMUZ2INFORMATION_H_
#define IMUZ2INFORMATION_H_

#include "MV2Const.h"

namespace zmp
{
	namespace mv2 
	{
/**
 * @brief IMU-Z2情報を保持するためのクラス
 *
 * @author	sekiguchi	
 * @date	2013/07/31
 */
class Imuz2Information
{
public:
	Imuz2Information();
	virtual ~Imuz2Information();

	double GetAccX() const;
	double GetAccY() const;
	double GetAccZ() const;
	double GetGyroX() const;
	double GetGyroY() const;
	double GetGyroZ() const;
	double GetCompX() const;
	double GetCompY() const;
	double GetCompZ() const;			

	IMUZ2_ROLE GetRole() const;
	int GetPeriod() const;
	RANGE_ACC GetRangeAcc() const;
	RANGE_GYRO GetRangeGyro() const;
	RANGE_COMP GetRangeComp() const;
	float GetBattLevel() const;
	IMUZ2_FORMAT GetFormat() const;
	unsigned char GetHardVersion() const;
	unsigned char GetFirmVersion() const;
	void GetBtAdr(unsigned char* btAdr);

	void SetAccData(short accX, short accY, short accZ);
	void SetGyroData(short gyroX, short gyroY, short gyroZ);
	void SetCompData(short compX, short compY, short compZ);
	void SetImuzState(unsigned char role, 
			short period,
			unsigned char rangeAcc,
			unsigned char rangeGyro,
			unsigned char rangeComp,
			unsigned char battLevel,
			unsigned char format); 
	void SetImuzDeviceProfile(unsigned char hard, 
				unsigned char firm,
				unsigned char* bt);

private:
	double _accX;		/*!< @brief 加速度X軸[G] */
	double _accY;		/*!< @brief 加速度Y軸[G] */
	double _accZ;		/*!< @brief 加速度Z軸[G] */
	double _gyroX;		/*!< @brief 角速度X軸[deg/sec] */
	double _gyroY;		/*!< @brief 角速度Y軸[deg/sec] */
	double _gyroZ;		/*!< @brief 角速度Z軸[deg/sec] */
	double _compX;		/*!< @brief 地磁気X軸[deg/sec] */
	double _compY;		/*!< @brief 地磁気Y軸[deg/sec] */
	double _compZ;		/*!< @brief 地磁気Z軸[deg/sec] */

	IMUZ2_ROLE _role;	/*!< @brief role(SingleBT/CAN-MasterBT/CAN-Slave) */
	short _period;		/*!< @brief period(10〜10000)[ms] */
	RANGE_ACC _rangeAcc;	/*!< @brief 加速度範囲(2/4/8)[G] */
	RANGE_GYRO _rangeGyro;	/*!< @brief 角速度範囲(500/2000)[deg/sec] */
	RANGE_COMP _rangeComp;	/*!< @brief 地磁気範囲(0.7/1.0/1.5/2.0/3.2/3.8/4.5[Ga]) */
	float	_battLevel;	/*!< @brief バッテリ電圧[V] */
	IMUZ2_FORMAT _format;	/*!< @brief データフォーマット(text/binary/extend) */
	unsigned char _hardNo;	/*!< @brief ハードウェアバージョン */
	unsigned char _firmNo;	/*!< @brief ファームウェアバージョン */
	unsigned char _btAdr[6]; /*!< @brief Bluetooth番号 */
};
	}
}

#endif /* OTHERINFORMATION_H_ */
