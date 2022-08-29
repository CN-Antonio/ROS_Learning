// /*
//  * read images with eBUS SDK
//  */

#include "ros/ros.h"
#include "std_msgs/String.h"
// #include "msgs/Msg.h"

#include <PvSampleUtils.h>
#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvPipeline.h>
#include <PvBuffer.h>

PV_INIT_SIGNAL_HANDLER();

#define BUFFER_COUNT ( 16 )

///
/// Function Prototypes
///
PvDevice *ConnectToDevice( const PvString &aConnectionID );
PvStream *OpenStream( const PvString &aConnectionID );
void ConfigureStream( PvDevice *aDevice, PvStream *aStream );
PvPipeline* CreatePipeline( PvDevice *aDevice, PvStream *aStream );
void AcquireImages( PvDevice *aDevice, PvStream *aStream, PvPipeline *aPipeline );

//
// Main function
//
int main(int argc, char **argv)
{
    PvDevice *lDevice = NULL;
    PvStream *lStream = NULL;

    ros::init(argc,argv, "eBUS_CAM_publisher"); //解析参数，命名当前node
    ros::NodeHandle n;  //创建句柄，实例化node
    ros::Rate delay(1);
    ros::Publisher eBUS_publish = n.advertise<std_msgs::String>("eBUS_topic",10); //(发送的目标topic，消息队列长度)
    // ros::Publisher say_pub_new = n.advertise<msgs::Msg>("say_topic_new",10);
    delay.sleep(); // ensure pub register successful
    ros::Rate loop_rate(1);    //控制rate(Hz)

    PV_SAMPLE_INIT();

    cout << "PvPipelineSample:" << endl << endl;
    std_msgs::String msg;
    std::stringstream ss;
    ss << "PvPipelineSample:";
    msg.data = ss.str();
    eBUS_publish.publish(msg);

    PvString lConnectionID;
    if ( PvSelectDevice( &lConnectionID ) )
    {
        msg.data = lConnectionID;
        eBUS_publish.publish(msg);
        lDevice = ConnectToDevice( lConnectionID );
        if ( lDevice != NULL )
        {
            lStream = OpenStream( lConnectionID );
            if ( lStream != NULL )
            {
                PvPipeline *lPipeline = NULL;

                ConfigureStream( lDevice, lStream );
                lPipeline = CreatePipeline( lDevice, lStream );
                if( lPipeline )
                {
                    AcquireImages( lDevice, lStream, lPipeline );
                    delete lPipeline;
                }
                
                // Close the stream
                cout << "Closing stream" << endl;
                lStream->Close();
                PvStream::Free( lStream );
            }

            // Disconnect the device
            cout << "Disconnecting device" << endl;
            lDevice->Disconnect();
            PvDevice::Free( lDevice );
        }
    }

    cout << endl;
    cout << "<press a key to exit>" << endl;
    PvWaitForKeyPress();

    PV_SAMPLE_TERMINATE();

    return 0;
}

PvDevice *ConnectToDevice( const PvString &aConnectionID )
{
    PvDevice *lDevice;
    PvResult lResult;

    // Connect to the GigE Vision or USB3 Vision device
    cout << "Connecting to device." << endl;
    lDevice = PvDevice::CreateAndConnect( aConnectionID, &lResult );
    if ( lDevice == NULL )
    {
        cout << "Unable to connect to device: "
        << lResult.GetCodeString().GetAscii()
        << " ("
        << lResult.GetDescription().GetAscii()
        << ")" << endl;
    }

 
    return lDevice;
}

PvStream *OpenStream( const PvString &aConnectionID )
{
    PvStream *lStream;
    PvResult lResult;

    // Open stream to the GigE Vision or USB3 Vision device
    cout << "Opening stream from device." << endl;
    lStream = PvStream::CreateAndOpen( aConnectionID, &lResult );
    if ( lStream == NULL )
    {
        cout << "Unable to stream from device. "
            << lResult.GetCodeString().GetAscii()
            << " ("
            << lResult.GetDescription().GetAscii()
            << ")"
            << endl;
    }

    return lStream;
}

void ConfigureStream( PvDevice *aDevice, PvStream *aStream )
{
    // If this is a GigE Vision device, configure GigE Vision specific streaming parameters
    PvDeviceGEV* lDeviceGEV = dynamic_cast<PvDeviceGEV *>( aDevice );
    if ( lDeviceGEV != NULL )
    {
        PvStreamGEV *lStreamGEV = static_cast<PvStreamGEV *>( aStream );

        // Negotiate packet size
        lDeviceGEV->NegotiatePacketSize();

        // Configure device streaming destination
        lDeviceGEV->SetStreamDestination( lStreamGEV->GetLocalIPAddress(), lStreamGEV->GetLocalPort() );
    }
}

PvPipeline *CreatePipeline( PvDevice *aDevice, PvStream *aStream )
{
    // Create the PvPipeline object
    PvPipeline* lPipeline = new PvPipeline( aStream );

    if ( lPipeline != NULL )
    {        
        // Reading payload size from device
        uint32_t lSize = aDevice->GetPayloadSize();
    
        // Set the Buffer count and the Buffer size
        lPipeline->SetBufferCount( BUFFER_COUNT );
        lPipeline->SetBufferSize( lSize );
    }
    
    return lPipeline;
}

void AcquireImages( PvDevice *aDevice, PvStream *aStream, PvPipeline *aPipeline )
{
    // Get device parameters need to control streaming
    PvGenParameterArray *lDeviceParams = aDevice->GetParameters();

    // Map the GenICam AcquisitionStart and AcquisitionStop commands
    PvGenCommand *lStart = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStart" ) );
    PvGenCommand *lStop = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStop" ) );

    // Note: the pipeline must be initialized before we start acquisition
    cout << "Starting pipeline" << endl;
    aPipeline->Start();

    // Get stream parameters
    PvGenParameterArray *lStreamParams = aStream->GetParameters();

    // Map a few GenICam stream stats counters
    PvGenFloat *lFrameRate = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "AcquisitionRate" ) );
    PvGenFloat *lBandwidth = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "Bandwidth" ) );

    // Enable streaming and send the AcquisitionStart command
    cout << "Enabling streaming and sending AcquisitionStart command." << endl;
    aDevice->StreamEnable();
    lStart->Execute();

    char lDoodle[] = "|\\-|-/";
    int lDoodleIndex = 0;
    double lFrameRateVal = 0.0;
    double lBandwidthVal = 0.0;

    // Acquire images until the user instructs us to stop.
    cout << endl << "<press a key to stop streaming>" << endl;
    while ( !PvKbHit() )
    {
        PvBuffer *lBuffer = NULL;
        PvResult lOperationResult;

        // Retrieve next buffer
        PvResult lResult = aPipeline->RetrieveNextBuffer( &lBuffer, 1000, &lOperationResult );
        if ( lResult.IsOK() )
        {
            if ( lOperationResult.IsOK() )
            {
                //
                // We now have a valid buffer. This is where you would typically process the buffer.
                // -----------------------------------------------------------------------------------------
                // ...

                lFrameRate->GetValue( lFrameRateVal );
                lBandwidth->GetValue( lBandwidthVal );

                cout << fixed << setprecision( 1 );
                cout << lDoodle[ lDoodleIndex ];
                cout << " BlockID: " << uppercase << hex << setfill( '0' ) << setw( 16 ) << lBuffer->GetBlockID();

                switch ( lBuffer->GetPayloadType() )
                {
                case PvPayloadTypeImage:
                    cout << "  W: " << dec << lBuffer->GetImage()->GetWidth() << " H: " << lBuffer->GetImage()->GetHeight();
                    break;

                case PvPayloadTypeChunkData:
                    cout << " Chunk Data payload type" << " with " << lBuffer->GetChunkCount() << " chunks";
                    break;

                case PvPayloadTypeRawData:
                    cout << " Raw Data with " << lBuffer->GetRawData()->GetPayloadLength() << " bytes";
                    break;

                case PvPayloadTypeMultiPart:
                    cout << " Multi Part with " << lBuffer->GetMultiPartContainer()->GetPartCount() << " parts";
                    break;

                default:
                    cout << " Payload type not supported by this sample";
                    break;
                }

                cout << "  " << lFrameRateVal << " FPS  " << ( lBandwidthVal / 1000000.0 ) << " Mb/s   \r";
            }
            else
            {
                // Non OK operational result
                cout << lDoodle[ lDoodleIndex ] << " " << lOperationResult.GetCodeString().GetAscii() << "\r";
            }

            // Release the buffer back to the pipeline
            aPipeline->ReleaseBuffer( lBuffer );
        }
        else
        {
            // Retrieve buffer failure
            cout << lDoodle[ lDoodleIndex ] << " " << lResult.GetCodeString().GetAscii() << "\r";
        }

        ++lDoodleIndex %= 6;
    }

    PvGetChar(); // Flush key buffer for next stop.
    cout << endl << endl;

    // Tell the device to stop sending images.
    cout << "Sending AcquisitionStop command to the device" << endl;
    lStop->Execute();

    // Disable streaming on the device
    cout << "Disable streaming on the controller." << endl;
    aDevice->StreamDisable();

    // Stop the pipeline
    cout << "Stop pipeline" << endl;
    aPipeline->Stop();
}