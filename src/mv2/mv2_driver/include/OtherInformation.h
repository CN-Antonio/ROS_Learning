/**
 * @file	OtherInformation.h
 * @brief	その他情報クラスヘッダファイル
 *
 * @date	2013/03/04
 * @author	sekiguchi
 * @par		Copyright:
 *		2013 ZMP Inc. All rights reserved.
 */
#ifndef OTHERINFORMATION_H_
#define OTHERINFORMATION_H_

#include "MV2Const.h"

namespace zmp
{
	namespace mv2 
	{
/**
 * @brief 各種情報を保持するためのクラス
 *
 * @author	sekiguchi	
 * @date	2013/03/04
 */
class OtherInformation
{
public:
	OtherInformation();
	virtual ~OtherInformation();
			
	int GetErrLevel() const;
	int GetErrCode() const;
	int GetIgnitionSwitch() const;
	int GetEmergencySwitch() const;	
	int GetAdcStatus() const;
	int GetWatchdogStatus() const;

	void SetRcvErrLevel(int level);
	void SetRcvErrCode(int code);
	void SetRcvIgnitionSwitch(int ignition);
	void SetRcvEmergencySwitch(int emergency);
	void SetAdcStatus(int adc);
	void SetWatchdogStatus(int watchdog);

private:
	int	_err_level;	/*!< @brief エラーレベル */
	int	_err_code;	/*!< @brief エラーコード */
	int     _ignition;	/*!< @brief Ignitionスイッチ状態 */
	int	_emergency;	/*!< @brief Emergencyスイッチ状態 */
	int _adc_status;
	int _watchdog_status;
};
	}
}

#endif /* OTHERINFORMATION_H_ */
