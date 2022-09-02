#include "eBUS_node.h"

// eBUS SDK buffers
PvBuffer* gPvBuffers;
// 3rd part

bool SelectDevice(PvString *lConnectionID)
{
    if ( !PvSelectDevice( lConnectionID ) )
    {
        cout << "No device selected." << endl;
        return false;
    }
}

PvDevice *ConnectToDevice( const PvString &aConnectionID )
{
    PvDevice *lDevice;
    PvResult lResult;

    // Connect to the GigE Vision or USB3 Vision device
    cout << "Connecting to device." << endl;
    lDevice = PvDevice::CreateAndConnect( aConnectionID, &lResult );
    if ( (lDevice == NULL ) || (!lResult.IsOK()) )
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
    if ( ( lStream == NULL ) || !lResult.IsOK() )
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

//
// 1. Allocates native 3rd party library buffers.
// 2. Allocates eBUS SDK buffers attached to the 3rd party library buffers.
// 3. Queues the eBUS SDK buffers in the PvStream object.
//
void CreateBuffers( PvDevice* aDevice, PvStream* aStream ) 
{
    gPvBuffers = NULL;
    PvGenParameterArray *lDeviceParams = aDevice->GetParameters();

    // Set device in RGB8 to match what our imaging library expects
    lDeviceParams->SetEnumValue( "PixelFormat", PvPixelRGB8 );

    // Get width, height from device
    int64_t lWidth = 0, lHeight = 0;
    lDeviceParams->GetIntegerValue( "Width", lWidth );
    lDeviceParams->GetIntegerValue( "Height", lHeight );

    // // Use min of BUFFER_COUNT and how many buffers can be queued in PvStream.
    // gBufferCount = ( aStream->GetQueuedBufferMaximum() < BUFFER_COUNT ) ? 
    //     aStream->GetQueuedBufferMaximum() : 
    //     BUFFER_COUNT;
    
    // // Create our image buffers which are holding the real memory buffers
    // gImagingBuffers = new SimpleImagingLib::ImagingBuffer[ gBufferCount ];
    // for ( uint32_t i = 0; i < gBufferCount; i++ )
    // {
    //     gImagingBuffers[ i ].AllocateImage( static_cast<uint32_t>( lWidth ), static_cast<uint32_t>( lHeight ), 3 );
    // }

    // Creates, eBUS SDK buffers, attach out image buffer memory
    // gPvBuffers = new PvBuffer[ gBufferCount ];
    // for ( uint32_t i = 0; i < gBufferCount; i++ )
    // {
    //     // Attach the memory of our imaging buffer to a PvBuffer. The PvBuffer is used as a shell
    //     // that allows directly acquiring image data into the memory owned by our imaging buffer
    //     gPvBuffers[ i ].GetImage()->Attach( gImagingBuffers[ i ].GetTopPtr(), 
    //         static_cast<uint32_t>( lWidth ), static_cast<uint32_t>( lHeight ), PvPixelRGB8 );

    //     // Set eBUS SDK buffer ID to the buffer/image index
    //     gPvBuffers[ i ].SetID( i );
    // }

    // Queue all buffers in the stream
    // for ( uint32_t i = 0; i < gBufferCount; i++ )
    // {
    //     aStream->QueueBuffer( gPvBuffers + i );
    // }
}