/**
 * @file	InclinoInf.h
 * @brief	Inclino情報クラスヘッダファイル
 * @date	2014/02/04
 * @author	sekiguchi
 * @par		Copyright:
 *		2014 ZMP Inc. All rights reserved.
 */
#ifndef INCLINOINF_H_
#define INCLINOINF_H_

#include "MV2Const.h"

namespace zmp
{
	namespace mv2 
	{
/**
 * @brief Inclino情報を保持するためのクラス
 * @author	sekiguchi	
 * @date	2014/02/04
 */
class InclinoInf
{
public:
	InclinoInf();
	virtual ~InclinoInf();

	double GetAccX() const;
	double GetAccY() const;
	double GetAngleX() const;
	double GetAngleY() const;

	short GetMode() const;
	short GetPeriod() const;
	float GetBattLevel() const;

	void SetAccData(short accX, short accY);
	void SetInclinoData(unsigned short angleX, unsigned short angleY);
	void SetStatus(short mode, short period, short battery);

private:
	double _accX;
	double _accY;
	double _angleX;
	double _angleY;

	short _period;
	float _battLevel;
	short _mode;
};
	}
}

#endif /* INCLINOINF_H_ */
