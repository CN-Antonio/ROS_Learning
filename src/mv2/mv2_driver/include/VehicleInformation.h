/**
 * @file	VehicleInformation.h
 * @brief	車両情報クラスヘッダファイル
 *
 * @date	2013/03/04
 * @author	sekiguchi
 * @par		Copyright:
 *		2013 ZMP Inc. All rights reserved.
 */
#ifndef VEHICLEINFORMATION_H_
#define VEHICLEINFORMATION_H_

#include "MV2Const.h"

namespace zmp
{
	namespace mv2 
	{
/**
 * @brief 車両情報を保持するためのクラス
 *
 * @author	sekiguchi	
 * @date	2013/03/04
 */
class VehicleInformation
{
public:
	VehicleInformation();
	virtual ~VehicleInformation();
			
	int GetMv2SysState() const;
	float GetMv2BattSOC() const;
	float GetMv2Veloc() const;
	int GetMv2DrvErrCode() const;
	int GetMv2ChargeState() const;
	int GetMv2BattErrCode() const;
	float GetMv2ChargeCurrent() const;
	float GetMv2BattTemp() const;
	int GetMv2MeterVeloc() const;
	int GetMv2BattLevel() const;
	int GetMv2Odometer() const;
	float GetMv2Trip() const;	
	
	void SetRcvMv2SysState(int state);
	void SetRcvMv2BattSOC(float soc);
	void SetRcvMv2Veloc(float veloc);
	void SetRcvMv2DrvErrCode(int code);
	void SetRcvMv2ChargeState(int state);
	void SetRcvMv2BattErrCode(int code);
	void SetRcvMv2ChargeCurrent(float current);
	void SetRcvMv2BattTemp(float temp);
	void SetRcvMv2MeterVeloc(int veloc);
	void SetRcvMv2BattLevel(int level);
	void SetRcvMv2Odometer(int odo);
	void SetRcvMv2Trip(float trip);

private:
	int	_mv2_sys_state; /*!< @brief 車両システム起動状態 */
	float	_mv2_batt_soc;	/*!< @brief 車両SOC */
	float	_mv2_veloc;	/*!< @brief 車両速度情報 */
	int	_mv2_drv_errCode;/*!< @brief 車両ドライブエラー情報 */
	int	_mv2_charge_state;  /*!< @brief 車両充電状態 */
	int	_mv2_batt_errCode;/*!< @brief 車両バッテリエラーコード */
	float	_mv2_charge_current;/*!< @brief 車両充電電流 */
	float	_mv2_batt_temp;	/*!< @brief 車両バッテリ温度 */
	int	_mv2_meter_veloc;/*!< @brief 車両メータ速度 */
	int	_mv2_batt_level;/*!< @brief 車両バッテリレベル */
	int	_mv2_odometer;	/*!< @brief 車両odometer */
	float	_mv2_trip;	/*!< @brief 車両trip */ 
};
	}
}

#endif /* VEHICLEINFORMATION_H_ */
