//
// Created by chenqy on 2021/10/29.
//

#ifndef MOWER_WS_OPTITRACKCLIENT_HPP
#define MOWER_WS_OPTITRACKCLIENT_HPP

#include <cinttypes>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <vector>

#include <OptiTrack/NatNetTypes.h>
#include <OptiTrack/NatNetCAPI.h>
#include <OptiTrack/NatNetClient.h>

typedef void (NATNET_CALLCONV* OptiTrackFrameReceivedCallback)( sFrameOfMocapData* pFrameOfData );

static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;

OptiTrackFrameReceivedCallback g_dataCallback;

NatNetClient* g_pClient = nullptr;
FILE* g_outputFile;

sNatNetClientConnectParams g_connectParams; //NOLINT
int g_analogSamplesPerMocapFrame = 0;
sServerDescription g_serverDescription;



class OptiTrackClient {
public:
    explicit OptiTrackClient(OptiTrackFrameReceivedCallback dataCallback, const char *serverAddress, const char *localAddress = nullptr) {
        g_dataCallback = dataCallback;
        // print version info
        unsigned char ver[4];
        NatNet_GetVersion(ver);
        printf("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

        // Install logging callback
        NatNet_SetLogCallback(MessageHandler);

        // create NatNet client
        g_pClient = new NatNetClient();

        // set the frame callback handler
        // this function will receive data from the server
        g_pClient->SetFrameReceivedCallback(DataHandler, g_pClient);

        g_connectParams.connectionType = kDefaultConnectionType;

        g_connectParams.serverAddress = serverAddress;

        if (localAddress) {
            g_connectParams.localAddress = localAddress;
        }

        int iResult;

        // Connect to Motive
        iResult = ConnectClient();
        if (iResult != ErrorCode_OK) {
            printf("Error initializing client. See log for details. Exiting.\n");
            exit(1);
        } else {
            printf("Client initialized and ready.\n");
        }


        // Send/receive test request
        void *response;
        int nBytes;
        printf("[OptiTrackClient] Sending Test Request\n");
        iResult = g_pClient->SendMessageAndWait("TestRequest", &response, &nBytes);
        if (iResult == ErrorCode_OK) {
            printf("[OptiTrackClient] Received: %s\n", (char *) response);
        }

        // Retrieve Data Descriptions from Motive
        printf("\n\n[OptiTrackClient] Requesting Data Descriptions...\n");
        sDataDescriptions *pDataDefs = nullptr;
        iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
        if (iResult != ErrorCode_OK || pDataDefs == nullptr) {
            printf("[OptiTrackClient] Unable to retrieve Data Descriptions.\n");
        } else {
            printf("[OptiTrackClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions);
            for (int i = 0; i < pDataDefs->nDataDescriptions; i++) {
                printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
                if (pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet) {
                    // MarkerSet
                    sMarkerSetDescription *pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
                    printf("MarkerSet Name : %s\n", pMS->szName);
                    for (int j = 0; j < pMS->nMarkers; j++)
                        printf("%s\n", pMS->szMarkerNames[j]);

                } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody) {
                    // RigidBody
                    sRigidBodyDescription *pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                    printf("RigidBody Name : %s\n", pRB->szName);
                    printf("RigidBody ID : %d\n", pRB->ID);
                    printf("RigidBody Parent ID : %d\n", pRB->parentID);
                    printf("Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);

                    if (pRB->MarkerPositions != nullptr && pRB->MarkerRequiredLabels != nullptr) {
                        for (int markerIdx = 0; markerIdx < pRB->nMarkers; ++markerIdx) {
                            const MarkerData &markerPosition = pRB->MarkerPositions[markerIdx];
                            const int markerRequiredLabel = pRB->MarkerRequiredLabels[markerIdx];

                            printf("\tMarker #%d:\n", markerIdx);
                            printf("\t\tPosition: %.2f, %.2f, %.2f\n", markerPosition[0], markerPosition[1],
                                   markerPosition[2]);

                            if (markerRequiredLabel != 0) {
                                printf("\t\tRequired active label: %d\n", markerRequiredLabel);
                            }
                        }
                    }
                } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton) {
                    // Skeleton
                    sSkeletonDescription *pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
                    printf("Skeleton Name : %s\n", pSK->szName);
                    printf("Skeleton ID : %d\n", pSK->skeletonID);
                    printf("RigidBody (Bone) Count : %d\n", pSK->nRigidBodies);
                    for (int j = 0; j < pSK->nRigidBodies; j++) {
                        sRigidBodyDescription *pRB = &pSK->RigidBodies[j];
                        printf("  RigidBody Name : %s\n", pRB->szName);
                        printf("  RigidBody ID : %d\n", pRB->ID);
                        printf("  RigidBody Parent ID : %d\n", pRB->parentID);
                        printf("  Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
                    }
                } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_ForcePlate) {
                    // Force Plate
                    sForcePlateDescription *pFP = pDataDefs->arrDataDescriptions[i].Data.ForcePlateDescription;
                    printf("Force Plate ID : %d\n", pFP->ID);
                    printf("Force Plate Serial : %s\n", pFP->strSerialNo);
                    printf("Force Plate Width : %3.2f\n", pFP->fWidth);
                    printf("Force Plate Length : %3.2f\n", pFP->fLength);
                    printf("Force Plate Electrical Center Offset (%3.3f, %3.3f, %3.3f)\n", pFP->fOriginX, pFP->fOriginY,
                           pFP->fOriginZ);
                    for (int iCorner = 0; iCorner < 4; iCorner++)
                        printf("Force Plate Corner %d : (%3.4f, %3.4f, %3.4f)\n", iCorner, pFP->fCorners[iCorner][0],
                               pFP->fCorners[iCorner][1], pFP->fCorners[iCorner][2]);
                    printf("Force Plate Type : %d\n", pFP->iPlateType);
                    printf("Force Plate Data Type : %d\n", pFP->iChannelDataType);
                    printf("Force Plate Channel Count : %d\n", pFP->nChannels);
                    for (int iChannel = 0; iChannel < pFP->nChannels; iChannel++)
                        printf("\tChannel %d : %s\n", iChannel, pFP->szChannelNames[iChannel]);
                } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Device) {
                    // Peripheral Device
                    sDeviceDescription *pDevice = pDataDefs->arrDataDescriptions[i].Data.DeviceDescription;
                    printf("Device Name : %s\n", pDevice->strName);
                    printf("Device Serial : %s\n", pDevice->strSerialNo);
                    printf("Device ID : %d\n", pDevice->ID);
                    printf("Device Channel Count : %d\n", pDevice->nChannels);
                    for (int iChannel = 0; iChannel < pDevice->nChannels; iChannel++)
                        printf("\tChannel %d : %s\n", iChannel, pDevice->szChannelNames[iChannel]);
                } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Camera) {
                    // Camera
                    sCameraDescription *pCamera = pDataDefs->arrDataDescriptions[i].Data.CameraDescription;
                    printf("Camera Name : %s\n", pCamera->strName);
                    printf("Camera Position (%3.2f, %3.2f, %3.2f)\n", pCamera->x, pCamera->y, pCamera->z);
                    printf("Camera Orientation (%3.2f, %3.2f, %3.2f, %3.2f)\n", pCamera->qx, pCamera->qy, pCamera->qz,
                           pCamera->qw);
                } else {
                    printf("Unknown data type.\n");
                    // Unknown
                }
            }
        }

        // TODO
        // Create data file for writing received stream into
        const char *szFile = "log/Client-output.pts";

        g_outputFile = fopen(szFile, "w");
        if (!g_outputFile) {
            printf("Error opening output file %s.  Exiting.\n", szFile);
            exit(1);
        }

        if (pDataDefs) {
            WriteHeader(g_outputFile, pDataDefs);
            NatNet_FreeDescriptions(pDataDefs);
            pDataDefs = nullptr;
        }

        // Ready to receive marker stream!
        printf("\nClient is connected to server and listening for data...\n");
    }
    ~ OptiTrackClient() {
        // Done - clean up.
        if (g_pClient) {
            g_pClient->Disconnect();
            delete g_pClient;
            g_pClient = nullptr;
        }

        if (g_outputFile) {
            WriteFooter(g_outputFile);
            fclose(g_outputFile);
            g_outputFile = nullptr;
        }
    }

private:
    // DataHandler receives data from the server
// This function is called by NatNet when a frame of mocap data is available
    static void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, __attribute__((unused)) void* pUserData) {
        if (g_outputFile) {
            WriteFrame(g_outputFile, data);
        }

        g_dataCallback(data);
    }
    // MessageHandler receives NatNet error/debug messages
    static void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char *msg) {
        // Optional: Filter out debug messages
        if (msgType < Verbosity_Info) {
            return;
        }

        printf("\n[NatNetLib]");

        switch (msgType) {
            case Verbosity_Debug:
                printf(" [DEBUG]");
                break;
            case Verbosity_Info:
                printf("  [INFO]");
                break;
            case Verbosity_Warning:
                printf("  [WARN]");
                break;
            case Verbosity_Error:
                printf(" [ERROR]");
                break;
            default:
                printf(" [?????]");
                break;
        }

        printf(": %s\n", msg);
    }

/* File writing routines */
    static void WriteHeader(FILE *fp, sDataDescriptions *pBodyDefs) {
        int i;

        if (pBodyDefs->arrDataDescriptions[0].type != Descriptor_MarkerSet)
            return;

        sMarkerSetDescription *pMS = pBodyDefs->arrDataDescriptions[0].Data.MarkerSetDescription;

        fprintf(fp, "<MarkerSet>\n\n");
        fprintf(fp, "<Name>\n%s\n</Name>\n\n", pMS->szName);

        fprintf(fp, "<Markers>\n");
        for (i = 0; i < pMS->nMarkers; i++) {
            fprintf(fp, "%s\n", pMS->szMarkerNames[i]);
        }
        fprintf(fp, "</Markers>\n\n");

        fprintf(fp, "<Data>\n");
        fprintf(fp, "Frame#\t");
        for (i = 0; i < pMS->nMarkers; i++) {
            fprintf(fp, "M%dX\tM%dY\tM%dZ\t", i, i, i);
        }
        fprintf(fp, "\n");

    }


    static void WriteFrame(FILE* fp, sFrameOfMocapData* data)
    {
        fprintf(fp, "%d", data->iFrame);
        for(int i =0; i < data->MocapData->nMarkers; i++)
        {
            fprintf(fp, "\t%.5f\t%.5f\t%.5f", data->MocapData->Markers[i][0], data->MocapData->Markers[i][1], data->MocapData->Markers[i][2]);
        }
        fprintf(fp, "\n");
    }



    static void WriteFooter(FILE *fp) {
        fprintf(fp, "</Data>\n\n");
        fprintf(fp, "</MarkerSet>\n");
    }

// Establish a NatNet Client connection
    static int ConnectClient() {
        // Release previous server
        g_pClient->Disconnect();

        // Init Client and connect to NatNet server
        int retCode = g_pClient->Connect(g_connectParams);
        if (retCode != ErrorCode_OK) {
            printf("Unable to connect to server.  Error code: %d. Exiting.\n", retCode);
            return ErrorCode_Internal;
        } else {
            // connection succeeded

            void *pResult;
            int nBytes = 0;
            ErrorCode ret;

            // print server info
            memset(&g_serverDescription, 0, sizeof(g_serverDescription));
            ret = g_pClient->GetServerDescription(&g_serverDescription);
            if (ret != ErrorCode_OK || !g_serverDescription.HostPresent) {
                printf("Unable to connect to server. Host not present. Exiting.\n");
                return 1;
            }
            printf("\n[OptiTrackClient] Server application info:\n");
            printf("Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp,
                   g_serverDescription.HostAppVersion[0],
                   g_serverDescription.HostAppVersion[1], g_serverDescription.HostAppVersion[2],
                   g_serverDescription.HostAppVersion[3]);
            printf("NatNet Version: %d.%d.%d.%d\n", g_serverDescription.NatNetVersion[0],
                   g_serverDescription.NatNetVersion[1],
                   g_serverDescription.NatNetVersion[2], g_serverDescription.NatNetVersion[3]);
            printf("Client IP:%s\n", g_connectParams.localAddress);
            printf("Server IP:%s\n", g_connectParams.serverAddress);
            printf("Server Name:%s\n", g_serverDescription.szHostComputerName);

            // get mocap frame rate
            ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
            if (ret == ErrorCode_OK) {
                float fRate = *((float *) pResult);
                printf("Mocap Framerate : %3.2f\n", fRate);
            } else
                printf("Error getting frame rate.\n");

            // get # of analog samples per mocap frame of data
            ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
            if (ret == ErrorCode_OK) {
                g_analogSamplesPerMocapFrame = *((int *) pResult);
                printf("Analog Samples Per Mocap Frame : %d\n", g_analogSamplesPerMocapFrame);
            } else
                printf("Error getting Analog frame rate.\n");
        }

        return ErrorCode_OK;
    }

};
#endif //MOWER_WS_OPTITRACKCLIENT_HPP
