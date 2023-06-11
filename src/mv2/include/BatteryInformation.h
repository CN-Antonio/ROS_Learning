/**
 * @file	BatteryInformation.h
 * @brief	バッテリ情報クラスヘッダファイル   电池信息分类文件
 * @date	2013/03/1
 * @author	sekiguchi
 * @par		Copyright:
 *		2013 ZMP Inc. All rights reserved.
 */
#ifndef BATTERYINFORMATION_H_
#define BATTERYINFORMATION_H_

namespace zmp
{
	namespace mv2 
	{
/**
 * @brief バッテリ情報を保持するためのクラス  保存电池信息
 * @author	sekiguchi	
 * @date	2013/03/1
 */
class BatteryInformation
{
public:
	BatteryInformation();
	virtual ~BatteryInformation();

//	int GetSoc() const;
//	int GetStatus() const;
//	int GetChgCurrent() const;
//	int GetTemp() const;
//	int GetChgLevel() const;

	float GetMainVoltage() const; //获取主电压
	float GetSubVoltage() const;  // 获取分电压
	float GetMainCurrent() const;  // 获取主电流
	float GetSubCurrent() const;  //获取分电流

//	void SetRcvSoc(int soc);
//	void SetRcvStatus(int status);
//	void SetRcvChgCurrent(int current);
//	void SetRcvTemp(int temp);
//	void SetRcvChgLevel(int level);

	void SetRcvMainCurrent(float current);
	void SetRcvSubCurrent(float current);
	void SetRcvMainVoltage(float volt);
	void SetRcvSubVoltage(float volt);

private:
//	int	_soc;		/*!< 充電状態 */
//	int	_status;	/*!< バッテリ状態 */
//	int	_chg_current;	/*!< 充電電流 */
//	int	_temp;		/*!< バッテリ温度 */
//	int	_level;		/*!< バッテリレベル */
	float	_main_current;	/*!< Mainバッテリ電流 */
	float	_sub_current;	/*!< Subバッテリ電流 */	
	float	_main_voltage;	/*!< Mainバッテリ電圧 */
	float	_sub_voltage;	/*!< Subバッテリ電圧 */	
};
	}
}

#endif /* BATTERYINFORMATION_H_ */
