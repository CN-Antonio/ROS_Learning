/**
 * @file	PoszInf.h
 * @brief	Posz情報クラスヘッダファイル
 *
 * @date	2014/02/05
 * @author	sekiguchi
 * @par		Copyright:
 *		2014 ZMP Inc. All rights reserved.
 */
#ifndef POSZINF_H_
#define POSZINF_H_

#include "MV2Const.h"

namespace zmp
{
	namespace mv2 
	{
/**
 * @brief Posz情報を保持するためのクラス
 *
 * @author	sekiguchi	
 * @date	2014/02/05
 */
class PoszInf
{
public:
	PoszInf();
	virtual ~PoszInf();

	GPGGA_DATA GetGpGGA() const;
	GPZDA_DATA GetGpZDA() const;
	GPGLL_DATA GetGpGLL() const;
	GPGSA_DATA GetGpGSA() const;
	GPGSV_DATA GetGpGSV() const;
	GPVTG_DATA GetGpVTG() const;
	GPRMC_DATA GetGpRMC() const;
	float GetHumid() const;
	float GetHumidTemp() const;
	float GetPressure() const;
	float GetPressTemp() const;	

	void SetGpsData(GPS_KIND kind, int index, unsigned char* data);
	void SetHumidData(int humid, int temp);
	void SetPressData(int press, int temp);
private:
	void SetGGAData(int index, unsigned char* data);
	void SetZDAData(int index, unsigned char* data);
	void SetGLLData(int index, unsigned char* data);
	void SetGSAData(int index, unsigned char* data);
	void SetGSVData(int index, unsigned char* data);
	void SetVTGData(int index, unsigned char* data);
	void SetRMCData(int index, unsigned char* data);

	void ParseGGA();
	void ParseZDA();
	void ParseGLL();
	void ParseGSA();
	void ParseGSV();
	void ParseVTG();
	void ParseRMC();

	int decBcd(unsigned char data);
	int decBcd2(unsigned char* data, int index, int len);
	float decLatitude(unsigned char* msg, int index);
	float decLongitude(unsigned char* msg, int index);
	float decDop(unsigned char* data, int index);
	float decGeoidLevel(unsigned char* data, int index);
	int decAgeOfDgps(unsigned char* data, int index);
	float decAzimuth(unsigned char* data, int index);
	float decSnr(unsigned char* data, int index);
	float decBcd3(unsigned char* data, int index);


	GPGGA_DATA _ggaData;
	GPZDA_DATA _zdaData;
	GPGLL_DATA _gllData;
	GPGSA_DATA _gsaData;
	GPGSV_DATA _gsvData;
	GPVTG_DATA _vtgData;
	GPRMC_DATA _rmcData;

	float _humidData;
	float _tempHumidData;
	float _pressData;
	float _tempPressData;

	unsigned char _gga[32];
	unsigned char _zda[32];
	unsigned char _gll[32];
	unsigned char _gsa[32];
	unsigned char _gsv[32];
	unsigned char _vtg[32];
	unsigned char _rmc[32];
};
	}
}

#endif /* POSZINF_H_ */
