#ifndef CANUSB_H_
#define CANUSB_H_

#include "CANUSBConst.h"
#include <pthread.h>
#include <termios.h>
#include <poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

namespace zmp {
	namespace hev {
class CANUSBZ {
public:
    CANUSBZ();
    virtual ~CANUSBZ();
    bool Init(int devNo);
    bool Start();
    bool Reset();
    bool Stop();
    bool Close();
    bool SetReceiveHandler(SerialReceiveHandler* handler);
    bool SendMsg(int ch, CANMsg cmsg);
    bool SetCANUSBZParam(int ch, CANUSBZ_SPEED speed, CANUSBZ_IDKIND kind);
    bool SendDeviceIDMsg();
    int GetDeviceID();

private:
    bool _b_on_rcv_thread;
    int _fd;
    pthread_t _thread;
    bool _b_terminated;

    int _devNo;
    int _deviceID;

    struct termios _oldtio;
    struct termios _newtio;
    struct pollfd  _client;
    SerialReceiveHandler* _callback_handler;

    bool InitThread();
    bool InitDevice(int devNo);
    void ReadThread();
    static void* ReadThreadEntry(void* pParam);
    unsigned char calc_sum(const unsigned char *data, int len);
    void ParseData(unsigned char* res, int* ch, long* time, CANMsg* cmsg);
    void ParseConfiguration(unsigned char *res);
};
    }
}

#endif /* CANUSB_H_ */
