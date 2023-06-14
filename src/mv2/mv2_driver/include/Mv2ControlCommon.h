/**
 * @file	Mv2ControlCommon.h
 * @brief	共通ユーティリティクラスヘッダファイル
 * @date	2013/03/04
 * @author	sekiguchi
 * @par		Copyright:
 *		2013 ZMP Inc. All rights reserved.
 */

#ifndef MV2CONTROLCOMMON_H_
#define MV2CONTROLCOMMON_H_

#include "MV2Const.h"

namespace zmp
{
	namespace mv2 
	{

/**
 * @brief 共通ユーティリティクラス
 * @author	sekiguchi	
 * @date	2013/03/04
 */
class Mv2ControlCommon
{
public:
	Mv2ControlCommon();
	virtual ~Mv2ControlCommon();

	/**
 	 * @brief	CANIDからMESSAGE_GROUP, MESSAGE_IDを取得する
 	 * @param[in]	id CAN ID
 	 * @param[out]	group_id MESSAGE_GROUPのID
 	 * @param[out]	message_id MESSAGE_ID
	 */
	static void ParseID(int id, int& group_id, int& message_id);

	/**
 	 * @brief	2byteを結合する
 	 * @param[in]	c_h 上位バイト
 	 * @param[in]	c_l 下位バイト
 	 * @return	結合された値
	 */
	static short MkShort(unsigned char c_h, unsigned char c_l);

	/**
 	 * @brief	4byteを結合する
 	 * @param[in]	c_0 0バイト目
 	 * @param[in]	c_1 1バイト目
 	 * @param[in]	c_2 2バイト目
 	 * @param[in]	c_3 3バイト目
 	 * @return	結合された値
	 */
	static unsigned int MkUint(unsigned char c_0, unsigned char c_1, unsigned char c_2, unsigned char c_3);

	/**
 	 * @brief	12bit signedの2byteを結合する
 	 * @param[in]	c_h 上位バイト
 	 * @param[in]	c_l 下位バイト
 	 * @return	結合された値
	 */
	static short Signed12ToShort(unsigned char c_h, unsigned char c_l);

};

	} /* namespace mv2 */
} /* namespace zmp */
#endif /* MV2CONTROLCOMMON_H_ */
