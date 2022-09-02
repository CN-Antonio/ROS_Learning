#ifndef __EBUS_NODE_H__
#define __EBUS_NODE_H__


// eBUS SDK
#include <PvSampleUtils.h>
#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvPipeline.h>
#include <PvBuffer.h>
#include <PvBufferConverter.h>
// OpenCV
#include <opencv2/opencv.hpp>

#define SP_20000C_ID "14FB00930ECE"
#define BUFFER_COUNT ( 16 )

///
/// Function Prototypes
///
PvDevice *ConnectToDevice( const PvString &aConnectionID );
PvStream *OpenStream( const PvString &aConnectionID );
PvPipeline* CreatePipeline( PvDevice *aDevice, PvStream *aStream );
void ConfigureStream( PvDevice *aDevice, PvStream *aStream );
bool ProcessImages();
void AcquireImages( PvDevice *aDevice, PvStream *aStream, PvPipeline *aPipeline );
void AcquireROSImages( PvDevice *aDevice, PvStream *aStream, PvPipeline *aPipeline );
cv::Mat PvImage2CV2Image(PvBuffer *aBuffer);
void TestPvBuffer(PvBuffer *aBuffer);

#endif