/**
 * @file
 * @~english
 * @brief	Host Communication class header.
 * @author	Koji Sekiguchi	
 * @~japanese
 * @brief	ホスト通信のサーバクラスヘッダ.
 * @author	関口浩司
 * @~
 * @date	 2014/02/18
 * Copyright (c) 2014 ZMP Inc. All rights reserved.
 */

#ifndef COMM_SERVER_H_
#define COMM_SERVER_H_

#include "socket.h"
#include "RczHMsg.h"

class CommServerStatusHandler
{
public:
	virtual void onOpen();
	virtual void onClose();
};

class CommControlCommandHandler
{
public:
    virtual void onRcvVisionDistMean(const RczHMsgRcvVisionDistMean *msg) = 0;
    virtual void onRcvVisionDistMin(const RczHMsgRcvVisionDistMin *msg) = 0;
    virtual void onRcvVisionDistPos(const RczHMsgRcvVisionDistPos *msg) = 0;
	virtual void onRcvSetDrvMode(const RczHMsgSetDrvMode *msg) = 0;
	virtual void onRcvSetDrvCMode(const RczHMsgSetDrvCMode *msg) = 0;
//	virtual void onRcvSetDrvOMode(const RczHMsgSetDrvOMode *msg) = 0;
	virtual void onRcvSetDrvServo(const RczHMsgSetDrvServo *msg) = 0;
	virtual void onRcvSetDrvShift(const RczHMsgSetDrvShift *msg) = 0;
	virtual void onRcvSetDrvStroke(const RczHMsgSetDrvStroke *msg) = 0;
	virtual void onRcvSetDrvVeloc(const RczHMsgSetDrvVeloc *msg) = 0;
	virtual void onRcvSetBrkStroke(const RczHMsgSetBrkStroke *msg) = 0;
	virtual void onRcvSetBrkLamp(const RczHMsgSetBrkLamp *msg) = 0;
	virtual void onRcvSetBlinkLeft(const RczHMsgSetBlinkLeft *msg) = 0;
	virtual void onRcvSetBlinkRight(const RczHMsgSetBlinkRight *msg) = 0;
	virtual void onRcvSetStrMode(const RczHMsgSetStrMode *msg) = 0;
	virtual void onRcvSetStrCMode(const RczHMsgSetStrCMode *msg) = 0;
//	virtual void onRcvSetStrOMode(const RczHMsgSetStrOMode *msg) = 0;
	virtual void onRcvSetStrServo(const RczHMsgSetStrServo *msg) = 0;
	virtual void onRcvSetStrTorque(const RczHMsgSetStrTorque *msg) = 0;
	virtual void onRcvSetStrAngle(const RczHMsgSetStrAngle *msg) = 0;
	virtual void onRcvSetCnfData(const RczHMsgSetCnfData *msg) = 0;
	virtual void onRcvGetCnfData(const RczHMsgGetCnfData *msg) = 0;
	virtual void onRcvReqCnfSave(const RczHMsgReqCnfSave *msg) = 0;
	virtual void onRcvGetErrInf(const RczHMsgGetErrInf *msg) = 0;
	virtual void onRcvClearErr(const RczHMsgClearErr *msg) = 0;
	virtual void onRcvGetFrmVer(const RczHMsgGetFrmVer *msg) = 0;
};

class CommControlCommandAdapter : public CommControlCommandHandler
{
public:
    virtual void onRcvVisionDistMean(const RczHMsgRcvVisionDistMean *) {};
    virtual void onRcvVisionDistMin(const RczHMsgRcvVisionDistMin *) {};
    virtual void onRcvVisionDistPos(const RczHMsgRcvVisionDistPos *) {};
	virtual void onRcvSetDrvMode(const RczHMsgSetDrvMode *) {};
	virtual void onRcvSetDrvCMode(const RczHMsgSetDrvCMode *) {};
//	virtual void onRcvSetDrvOMode(const RczHMsgSetDrvOMode *) {};
	virtual void onRcvSetDrvServo(const RczHMsgSetDrvServo *) {};
	virtual void onRcvSetDrvShift(const RczHMsgSetDrvShift *) {};
	virtual void onRcvSetDrvStroke(const RczHMsgSetDrvStroke *) {};
	virtual void onRcvSetDrvVeloc(const RczHMsgSetDrvVeloc *) {};
	virtual void onRcvSetBrkStroke(const RczHMsgSetBrkStroke *) {};
	virtual void onRcvSetBrkLamp(const RczHMsgSetBrkLamp *) {};
	virtual void onRcvSetBlinkLeft(const RczHMsgSetBlinkLeft *) {};
	virtual void onRcvSetBlinkRight(const RczHMsgSetBlinkRight *) {};
	virtual void onRcvSetStrMode(const RczHMsgSetStrMode *) {};
	virtual void onRcvSetStrCMode(const RczHMsgSetStrCMode *) {};
//	virtual void onRcvSetStrOMode(const RczHMsgSetStrOMode *) {};
	virtual void onRcvSetStrServo(const RczHMsgSetStrServo *) {};
	virtual void onRcvSetStrTorque(const RczHMsgSetStrTorque *) {};
	virtual void onRcvSetStrAngle(const RczHMsgSetStrAngle *) {};
	virtual void onRcvSetCnfData(const RczHMsgSetCnfData *) {};
	virtual void onRcvGetCnfData(const RczHMsgGetCnfData *) {};
	virtual void onRcvReqCnfSave(const RczHMsgReqCnfSave *) {};
	virtual void onRcvGetErrInf(const RczHMsgGetErrInf *) {};
	virtual void onRcvClearErr(const RczHMsgClearErr *) {};
	virtual void onRcvGetFrmVer(const RczHMsgGetFrmVer *) {};
};

class CommServer : public SocketEventHandlerInterface
{
public:
    CommServer();
    virtual ~CommServer();

public:
    bool Init(int port_no);
    bool SetReceiveCommHandler(CommControlCommandHandler *handler);
    bool Start();
    bool Send(const uchar *msg, int len);
    bool IsReceiveMessage() const;
    bool KickCallback();
    bool GetMessage(RczHMsg *msg);

    bool GetClientName(char *name);
    bool IsConnect();

public:
	// overrides SocketEventHandlerInterface
    virtual void ReceiveProc(RingBuffer* receive_buf);
    virtual void InitStatus() {};
    virtual void OnConnectionOpen(std::string ) {};
    virtual void OnConnectionAccept(std::string ) {};
    virtual void OnConnectionClose(std::string ) {};
    virtual void OnConnectionDisconnect(std::string ) {};

protected:
    bool OnBody();
    bool QueueMessage(const RczHMsg* msg, int len);
    bool FetchMessage(unsigned char *msg_buff, RCZHMSG_ID *msg_id, int *length);
    bool ProcCallback(RCZHMSG_ID msg_id, RczHMsg *msg);


private:
    enum EN_SOCKET_RECEIVE_STAT {
	RSTAT_SOP,
	RSTAT_HEADER,
	RSTAT_BODY,
	RSTAT_RECOVER,
	RSTAT_RECOVER_STEP2,
    };


private:
//    SocketServerUDP *_sock;
    SocketServer *_sock;
    int _port_no;
    CommControlCommandHandler *_callback_handler_comm;

    EN_SOCKET_RECEIVE_STAT _sock_receive_stat;
    RczHMsg _msg_header;
    uchar *_build_buff;

    RingBuffer *_msg_buffer;
    uchar *_msg_get_buff;
};

#endif /* COMM_SERVER_H_ */




