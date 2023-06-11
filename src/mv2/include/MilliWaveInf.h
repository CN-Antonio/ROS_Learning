/**
 * @file	MilliWaveInf.h
 * @brief	ミリ波情報クラスヘッダファイル
 * @date	2014/02/04
 * @author	sekiguchi
 * @par		Copyright:
 *		2014 ZMP Inc. All rights reserved.
 */
#ifndef MILLIWAVEINF_H_
#define MILLIWAVEINF_H_

#include "MV2Const.h"

namespace zmp
{
	namespace mv2 
	{
/**
 * @brief MilliWave情報を保持するためのクラス
 * @author	sekiguchi	
 * @date	2014/02/04
 */
class MilliWaveInf
{
public:
    MilliWaveInf();
    virtual ~MilliWaveInf();

    MILLIWAVE_DATA GetMilliWaveData(int index);
    int SetMilliWaveData(int indexm, unsigned char* data);

private:
    MILLIWAVE_DATA _milliWave_data[20];
};
	}
}

#endif /* MILLIWAVEINF_H_ */
