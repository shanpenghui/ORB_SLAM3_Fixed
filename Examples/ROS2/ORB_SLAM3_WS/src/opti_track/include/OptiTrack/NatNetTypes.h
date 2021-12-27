//=============================================================================----
// Copyright © 2016 NaturalPoint, Inc. All Rights Reserved.
// 
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall NaturalPoint, Inc. or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//=============================================================================----

/*
NatNetTypes defines the public, common data structures and types
used when working with NatNetServer and NatNetClient objects.

version 3.0.0.0
*/

#pragma once


#include <stdint.h>
#include <string.h>     // for memset

#if !defined( NULL )
#   include <stddef.h>
#endif


#ifdef _WIN32
#   define NATNET_CALLCONV __cdecl
#else
#   define NATNET_CALLCONV
#endif


#ifdef _MSC_VER
#   define NATNET_DEPRECATED( msg )     __declspec(deprecated(msg))
#else
#   define NATNET_DEPRECATED( msg )     __attribute__((deprecated(msg)))
#endif


// storage class specifier
// - to link to NatNet dynamically, define NATNETLIB_IMPORTS and link to the NatNet import library.
// - to link to NatNet statically, link to the NatNet static library.
#if defined( _WIN32 )
#   if defined( NATNETLIB_EXPORTS )
#       define NATNET_API               __declspec(dllexport)
#   elif defined( NATNETLIB_IMPORTS )
#       define NATNET_API               __declspec(dllimport)
#   else
#       define NATNET_API
#   endif
#else
#   if defined( NATNETLIB_EXPORTS )
#       define NATNET_API               __attribute((visibility("default")))
#   elif defined( NATNETLIB_IMPORTS )
#       define NATNET_API
#   else
#       define NATNET_API
#   endif
#endif


#define NATNET_DEFAULT_PORT_COMMAND         1510
#define NATNET_DEFAULT_PORT_DATA            1511
#define NATNET_DEFAULT_MULTICAST_ADDRESS    "239.255.42.99"     // IANA, local network


// model limits
#define MAX_MODELS                  2000    // maximum number of total models (data descriptions)
#define MAX_MARKERSETS              1000    // maximum number of MarkerSets 
#define MAX_RIGIDBODIES             1000    // maximum number of RigidBodies
#define MAX_NAMELENGTH              256     // maximum length for strings
#define MAX_MARKERS                 200     // maximum number of markers per MarkerSet
#define MAX_RBMARKERS               20      // maximum number of markers per RigidBody
#define MAX_SKELETONS               100     // maximum number of skeletons
#define MAX_SKELRIGIDBODIES         200     // maximum number of RididBodies per Skeleton
#define MAX_LABELED_MARKERS         1000    // maximum number of labeled markers per frame
#define MAX_UNLABELED_MARKERS       1000    // maximum number of unlabeled (other) markers per frame

#define MAX_FORCEPLATES             32      // maximum number of force plates
#define MAX_DEVICES                 32      // maximum number of peripheral devices
#define MAX_ANALOG_CHANNELS         32      // maximum number of data channels (signals) per analog/force plate device
#define MAX_ANALOG_SUBFRAMES        30      // maximum number of analog/force plate frames per mocap frame

#define MAX_PACKETSIZE              65503   // max size of packet in bytes (actual packet size is dynamic)
                                            // (65535 byte IP limit - 20 byte IP header - 8 byte UDP header - 4 byte sPacket header = 65503 bytes)


// Client/server message ids
#define NAT_CONNECT                 0
#define NAT_SERVERINFO              1
#define NAT_REQUEST                 2
#define NAT_RESPONSE                3
#define NAT_REQUEST_MODELDEF        4
#define NAT_MODELDEF                5
#define NAT_REQUEST_FRAMEOFDATA     6
#define NAT_FRAMEOFDATA             7
#define NAT_MESSAGESTRING           8
#define NAT_DISCONNECT              9
#define NAT_KEEPALIVE               10
#define NAT_DISCONNECTBYTIMEOUT     11
#define NAT_ECHOREQUEST             12
#define NAT_ECHORESPONSE            13
#define NAT_DISCOVERY               14
#define NAT_UNRECOGNIZED_REQUEST    100


#define UNDEFINED                    999999.9999

// NatNet uses to set reporting level of messages.
// Clients use to set level of messages to receive.
typedef enum Verbosity
{
    Verbosity_None = 0,
    Verbosity_Debug,
    Verbosity_Info,
    Verbosity_Warning,
    Verbosity_Error,
} Verbosity;


// NatNet error reporting codes
typedef enum ErrorCode
{
    ErrorCode_OK = 0,
    ErrorCode_Internal,
    ErrorCode_External,
    ErrorCode_Network,
    ErrorCode_Other,
    ErrorCode_InvalidArgument,
    ErrorCode_InvalidOperation,
    ErrorCode_InvalidSize,
} ErrorCode;


// NatNet connection types
typedef enum ConnectionType
{
    ConnectionType_Multicast = 0,
    ConnectionType_Unicast
} ConnectionType;


// NatNet data types
typedef enum DataDescriptors
{
    Descriptor_MarkerSet = 0,
    Descriptor_RigidBody,
    Descriptor_Skeleton,
    Descriptor_ForcePlate,
    Descriptor_Device,
    Descriptor_Camera
} DataDescriptors;


typedef float MarkerData[3];                // posX, posY, posZ


#pragma pack(push, 1)

// sender
typedef struct sSender
{
    char szName[MAX_NAMELENGTH];            // host app's name
    uint8_t Version[4];                     // host app's version [major.minor.build.revision]
    uint8_t NatNetVersion[4];               // host app's NatNet version [major.minor.build.revision]
} sSender;


typedef struct sSender_Server
{
    sSender Common;

    uint64_t HighResClockFrequency;         // host's high resolution clock frequency (ticks per second)
    uint16_t DataPort;
    bool IsMulticast;
    uint8_t MulticastGroupAddress[4];
} sSender_Server;


// packet
// Describes basic NatNet packet structure.  
// Only used publicly by clients who are depacketizing NatNet packets directly
typedef struct sPacket
{
    uint16_t iMessage;                      // message ID (e.g. NAT_FRAMEOFDATA)
    uint16_t nDataBytes;                    // Num bytes in payload
    union
    {
        uint8_t         cData[MAX_PACKETSIZE];
        char            szData[MAX_PACKETSIZE];
        uint32_t        lData[MAX_PACKETSIZE/sizeof(uint32_t)];
        float           fData[MAX_PACKETSIZE/sizeof(float)];
        sSender         Sender;
        sSender_Server  SenderServer;
    } Data;                                 // payload - statically allocated for convenience.  Actual packet size is determined by  nDataBytes
} sPacket;

// sConnectionOptions
// Describes optional connection arguments.
// Only used publicly by clients who are depacketizing NatNet packets directly
// NatNetClients use sNatNetClientConnectParams
typedef struct sConnectionOptions
{
    bool subscribedDataOnly;
    uint8_t BitstreamVersion[4];
#if defined(__cplusplus)
    sConnectionOptions() : subscribedDataOnly(false)
    {
        memset(BitstreamVersion, 0, sizeof(BitstreamVersion));
    }
#endif
} sConnectionOptions;

#pragma pack(pop)


// Mocap server application description
typedef struct sServerDescription
{
    bool HostPresent;                       // host is present and accounted for
    char szHostComputerName[MAX_NAMELENGTH];// host computer name
    uint8_t HostComputerAddress[4];         // host IP address
    char szHostApp[MAX_NAMELENGTH];         // name of host app 
    uint8_t HostAppVersion[4];              // version of host app
    uint8_t NatNetVersion[4];               // host app's version of NatNet

    // Clock and connection info is only provided by NatNet 3.0+ servers.
    uint64_t HighResClockFrequency;         // host's high resolution clock frequency (ticks per second)

    bool bConnectionInfoValid;              // If the server predates NatNet 3.0, this will be false, and the other Connection* fields invalid.
    uint16_t ConnectionDataPort;            // The data port this server is configured to use.
    bool ConnectionMulticast;               // Whether this server is streaming in multicast. If false, connect in unicast instead.
    uint8_t ConnectionMulticastAddress[4];  // The multicast group address to use for a multicast connection.
} sServerDescription;


// Marker
typedef struct sMarker
{
    int32_t ID;                             // Unique identifier:
                                            // For active markers, this is the Active ID. For passive markers, this is the PointCloud assigned ID.
                                            // For legacy assets that are created prior to 2.0, this is both AssetID (High-bit) and Member ID (Lo-bit)

    float x;                                // x position
    float y;                                // y position
    float z;                                // z position
    float size;                             // marker size
    int16_t params;                         // host defined parameters.  Bit values:
                                                // 0 : Occluded
                                                // 1 : PointCloudSolved
                                                // 2 : ModelFilled
                                                // 3 : HasModel
                                                // 4 : Unlabeled
                                                // 5 : Active
                                                // 6 : Established
                                                // 7 : Measurement
    float residual;                         // marker error residual, in mm/ray
} sMarker;


// MarkerSet Definition
typedef struct sMarkerSetDescription
{
    char szName[MAX_NAMELENGTH];            // MarkerSet name
    int32_t nMarkers;                       // # of markers in MarkerSet
    char** szMarkerNames;                   // array of marker names
} sMarkerSetDescription;


// MarkerSet Data (single frame of one MarkerSet)
typedef struct sMarkerSetData
{
    char szName[MAX_NAMELENGTH];            // MarkerSet name
    int32_t nMarkers;                       // # of markers in MarkerSet
    MarkerData* Markers;                    // Array of marker data ( [nMarkers][3] )
} sMarkerSetData;


// Rigid Body Definition
typedef struct sRigidBodyDescription
{
    char szName[MAX_NAMELENGTH];            // RigidBody name
    int32_t ID;                             // RigidBody identifier: Streaming ID value for rigid body assets, and Bone index value for skeleton rigid bodies.
    int32_t parentID;                       // ID of parent Rigid Body (in case hierarchy exists; otherwise -1)
    float offsetx, offsety, offsetz;        // offset position relative to parent
    int32_t nMarkers;                       // Number of markers associated with this rigid body
    MarkerData* MarkerPositions;            // Array of marker locations ( [nMarkers][3] )
    int32_t* MarkerRequiredLabels;          // Array of expected marker active labels - 0 if not specified. ( [nMarkers] )
    char** szMarkerNames;                   // Array of marker names  ( [nMarkers][MAX_NAMELENGTH] )
} sRigidBodyDescription;


// Rigid Body Data (single frame of one rigid body)
typedef struct sRigidBodyData
{
    int32_t ID;                             // RigidBody identifier: 
                                            // For rigid body assets, this is the Streaming ID value. 
                                            // For skeleton assets, this combines both skeleton ID (High-bit) and Bone ID (Low-bit).

    float x, y, z;                          // Position
    float qx, qy, qz, qw;                   // Orientation
    float MeanError;                        // Mean measure-to-solve deviation
    int16_t params;                         // Host defined tracking flags

#if defined(__cplusplus)
    sRigidBodyData()
        : ID( 0 ),
        x(0.0),y(0.0),z(0.0),
        qx(0.0),qy(0.0),qz(0.0),qw(1.0),
        MeanError(0.0),
        params( 0 )
    {
    }
#endif
} sRigidBodyData;


// Skeleton Description
typedef struct sSkeletonDescription
{
    char szName[MAX_NAMELENGTH];                            // Skeleton name
    int32_t skeletonID;                                     // Skeleton unqiue identifier
    int32_t nRigidBodies;                                   // # of rigid bodies (bones) in skeleton
    sRigidBodyDescription RigidBodies[MAX_SKELRIGIDBODIES]; // array of rigid body (bone) descriptions 
} sSkeletonDescription;


// Skeleton Data
typedef struct sSkeletonData
{
    int32_t skeletonID;                                     // Skeleton unique identifier
    int32_t nRigidBodies;                                   // # of rigid bodies
    sRigidBodyData* RigidBodyData;                          // Array of RigidBody data
} sSkeletonData;

// FrocePlate description
typedef struct sForcePlateDescription
{
    int32_t ID;                                     // used for order, and for identification in the data stream
    char strSerialNo[128];                          // for unique plate identification
    float fWidth;                                   // plate physical width (manufacturer supplied)
    float fLength;                                  // plate physical length (manufacturer supplied)
    float fOriginX, fOriginY, fOriginZ;             // electrical center offset (from electrical center to geometric center-top of force plate) (manufacturer supplied)
    float fCalMat[12][12];                          // force plate calibration matrix (for raw analog voltage channel type only)
    float fCorners[4][3];                           // plate corners, in world (aka Mocap System) coordinates, clockwise from plate +x,+y (refer to C3D spec for details)
    int32_t iPlateType;                             // force plate 'type' (refer to C3D spec for details) 
    int32_t iChannelDataType;                       // 0=Calibrated force data, 1=Raw analog voltages
    int32_t nChannels;                              // # of channels (signals)
    char szChannelNames[MAX_ANALOG_CHANNELS][MAX_NAMELENGTH];   // channel names
} sForcePlateDescription;

// Peripheral Device description (e.g. NIDAQ)
typedef struct sDeviceDescription
{
    int32_t ID;                                     // used for order, and for identification in the data stream
    char strName[128];                              // device name as appears in Motive
    char strSerialNo[128];                          // for unique device identification
    int32_t iDeviceType;                            // device 'type' code 
    int32_t iChannelDataType;                       // channel data type code
    int32_t nChannels;                              // # of currently enabled/active channels (signals)
    char szChannelNames[MAX_ANALOG_CHANNELS][MAX_NAMELENGTH];   // channel names
} sDeviceDescription;

// Camera description
typedef struct sCameraDescription
{
    char strName[MAX_NAMELENGTH];                   // Camera Name ( can be used with Get/Set property commands )
    float x, y, z;                                  // Position
    float qx, qy, qz, qw;                           // Orientation
} sCameraDescription;


// Tracked Object data description.  
// A Mocap Server application (e.g. Arena or TrackingTools) may contain multiple
// tracked "objects (e.g. RigidBody, MarkerSet).  Each object will have its
// own DataDescription.
typedef struct sDataDescription
{
    int32_t type;
    union
    {
        sMarkerSetDescription*  MarkerSetDescription;
        sRigidBodyDescription*  RigidBodyDescription;
        sSkeletonDescription*   SkeletonDescription;
        sForcePlateDescription* ForcePlateDescription;
        sDeviceDescription*     DeviceDescription;
        sCameraDescription*     CameraDescription;
    } Data;
} sDataDescription;


// All data descriptions for current session (as defined by host app)
typedef struct sDataDescriptions
{
    int32_t nDataDescriptions;
    sDataDescription arrDataDescriptions[MAX_MODELS];
} sDataDescriptions;


typedef struct sAnalogChannelData
{
    int32_t nFrames;                                // # of analog frames of data in this channel data packet (typically # of subframes per mocap frame)
    float Values[MAX_ANALOG_SUBFRAMES];             // values
} sAnalogChannelData;

typedef struct sForcePlateData
{
    int32_t ID;                                         // ForcePlate ID (from data description)
    int32_t nChannels;                                  // # of channels (signals) for this force plate
    sAnalogChannelData ChannelData[MAX_ANALOG_CHANNELS];// Channel (signal) data (e.g. Fx[], Fy[], Fz[])
    int16_t params;                                     // Host defined flags
} sForcePlateData;

typedef struct sDeviceData
{
    int32_t ID;                                         // Device ID (from data description)
    int32_t nChannels;                                  // # of active channels (signals) for this device
    sAnalogChannelData ChannelData[MAX_ANALOG_CHANNELS];// Channel (signal) data (e.g. ai0, ai1, ai2)
    int16_t params;                                     // [b0:trigger bit] [b1:reserved] [b2:is synthetic data?] [b3,b4: mocap frame offset]
} sDeviceData;

// Single frame of data (for all tracked objects)
typedef struct sFrameOfMocapData
{
    int32_t iFrame;                                 // host defined frame number

    int32_t nMarkerSets;                            // # of marker sets in this frame of data
    sMarkerSetData MocapData[MAX_MARKERSETS];       // MarkerSet data

    int32_t nOtherMarkers;                          // # of undefined markers
    MarkerData* OtherMarkers;                       // undefined marker data

    int32_t nRigidBodies;                           // # of rigid bodies
    sRigidBodyData RigidBodies[MAX_RIGIDBODIES];    // Rigid body data

    int32_t nSkeletons;                             // # of Skeletons
    sSkeletonData Skeletons[MAX_SKELETONS];         // Skeleton data

    int32_t nLabeledMarkers;                        // # of Labeled Markers
    sMarker LabeledMarkers[MAX_LABELED_MARKERS];    // Labeled Marker data (labeled markers not associated with a "MarkerSet")

    int32_t nForcePlates;                           // # of force plates
    sForcePlateData ForcePlates[MAX_FORCEPLATES];   // Force plate data

    int32_t nDevices;                               // # of devices
    sDeviceData Devices[MAX_DEVICES];               // Device data

    uint32_t Timecode;                              // SMPTE timecode (if available)
    uint32_t TimecodeSubframe;                      // timecode sub-frame data
    double fTimestamp;                              // timestamp since software start ( software timestamp )
    uint64_t CameraMidExposureTimestamp;            // Given in host's high resolution ticks (from e.g. QueryPerformanceCounter).
    uint64_t CameraDataReceivedTimestamp;           // Given in host's high resolution ticks (from e.g. QueryPerformanceCounter).
    uint64_t TransmitTimestamp;                     // Given in host's high resolution ticks (from e.g. QueryPerformanceCounter).
    int16_t params;                                 // [b0:recording] [b1:model list changed] [b2: Live/Edit mode (0=Live, 1=Edit)]
} sFrameOfMocapData;


typedef struct sNatNetClientConnectParams
{
    ConnectionType connectionType;
    uint16_t serverCommandPort;
    uint16_t serverDataPort;
    const char* serverAddress;
    const char* localAddress;
    const char* multicastAddress;
	bool subscribedDataOnly;
	uint8_t BitstreamVersion[4];					 // client requests specific version of data bitstream (typically packet clients only)

#if defined(__cplusplus)
    sNatNetClientConnectParams()
        : connectionType( ConnectionType_Multicast )
        , serverCommandPort( 0 )
        , serverDataPort( 0 )
        , serverAddress( NULL )
        , localAddress( NULL )
        , multicastAddress( NULL )
		, subscribedDataOnly ( false )
    {
        memset(BitstreamVersion, 0, sizeof(BitstreamVersion));
    }
#endif
} sNatNetClientConnectParams;


// Callback function pointer types
typedef void (NATNET_CALLCONV* NatNetLogCallback)( Verbosity level, const char* message );
typedef void (NATNET_CALLCONV* NatNetFrameReceivedCallback)( sFrameOfMocapData* pFrameOfData, void* pUserData );
typedef void (NATNET_CALLCONV* NatNetUnknownMessageCallback)( sPacket* pPacket, void* pUserData );
