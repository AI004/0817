#include "NokovClinet.h"
#include <arpa/inet.h>
#include <dirent.h>
#include <fcntl.h>
#include <linux/unistd.h>
#include <net/if.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <Eigen/Dense>
#include <iostream>
#include <unordered_map>
#include <vector>
#define MAX_PATH 256

#include <mutex>
#include <thread>
#include "NokovSDKClient.h"
#include "NokovSDKTypes.h"
#include "Utility.h"
#include "basicfunction.h"
#include "data_nokov.h"

using namespace std;
// 这段代码的主要目的是在不同的平台上使用不同的互斥锁类型和函数声明方式。如果没有定义
// _LINUX 宏，则使用 C++ 标准库中的 std::mutex 和标准的 C 调用约定。如果定义了
// _LINUX 宏，则使用 POSIX 线程库中的 pthread_mutex_t 和 GCC 特定的
// __attribute__((__cdecl__)) 来指定 C 调用约定

void __attribute__((__cdecl__)) ForcePlateHandler(sForcePlates* data,
                                                  void* pUserData);  // receives force plate data from the server
void __attribute__((__cdecl__)) DataHandler(sFrameOfMocapData* data,
                                            void* pUserData);  // receives data from the server
void __attribute__((__cdecl__)) MessageHandler(int msgType,
                                               char* msg);  // receives NokovSDK error messages
void __attribute__((__cdecl__)) NotifyHandler(sNotifyMsg* data, void* pUserData);
void __attribute__((__cdecl__)) AnlogChHandler(sFrameOfAnalogChannelData* data, void* pUserData);
pthread_mutex_t m_mutex;

int bExit = 0;
NokovSDKClient* theClient;
char szMyIPAddress[128] = "";
char szServerIPAddress[128] = "";

typedef std::vector<std::vector<SlideFrameArray>> TrackerArray;
// Used to calculate velocity and acceleration
TrackerArray MarkerVelocityTrackerArray;
TrackerArray MarkerAccelerationTrackerArray;
std::vector<SlideFrameArray> BoneVelocityTrackerArray;
std::vector<SlideFrameArray> BoneAccelerationTrackerArray;
typedef std::unordered_map<int, std::string> IdNameMap;
IdNameMap g_RigidBodyIdMap;
IdNameMap g_SkeletonIdMap;
std::unordered_map<int, IdNameMap> g_SkeletonRigidsIdMp;
std::unordered_map<std::string, IdNameMap> g_mapMarkersSetDesc;
IdNameMap g_mapForcePlateDesc;
int g_iFrameRate = 200;  // 镜头型号1.3HW,最大支持240Hz的通讯频率
char g_cSelect = 0;
std::mutex g_v_body_mtx;
std::mutex g_euler_world_mtx;

// 这个函数通过创建一个 UDP 套接字并使用 ioctl 系统调用来获取指定网络接口的本地
// IP 地址。
int get_localip(const char* eth_name, char* local_ip_addr) {
  int ret = -1;
  register int fd;
  struct ifreq ifr;
  if (local_ip_addr == NULL || eth_name == NULL) {
    return ret;
  }
  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) > 0) {
    strcpy(ifr.ifr_name, eth_name);
    if (!(ioctl(fd, SIOCGIFADDR, &ifr))) {
      ret = 0;
      strcpy(local_ip_addr, inet_ntoa(((struct sockaddr_in*)&ifr.ifr_addr)->sin_addr));
      printf("Found %s:%s\n", eth_name, local_ip_addr);
    }
  }
  if (fd > 0) {
    close(fd);
  }
  return ret;
}

// 这段代码的主要目的是在不同的平台上使用不同的主函数声明，并且在定义了
// LOST_FRAME_TEST 宏的情况下，初始化 s_Record
// 结构体。这种做法常见于跨平台开发，以确保代码在不同环境下的兼容性和正确性。
int NokovClient() {
  int iResult = -1;

#ifdef LOST_FRAME_TEST
  memset(&s_Record, 0x00, sizeof(s_Record));
#endif

  // 1.init wsa
  pthread_mutex_init(&m_mutex, NULL);

  // printf("Client gethostbyname szMyIPAddress[%s] .\n", szMyIPAddress);
  // char ipBuf[100] = {0};
  // printf("Please input the server IP\n");
  // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  // std::cin >> ipBuf;

  // 4.Create NokovSDK Client
  iResult = CreateClient("192.168.31.230");
  if (iResult != ErrorCode_OK) {
    printf("Error initializing client.  Exiting");

    getchar();
    getchar();

    return 1;
  } else {
    printf("Client initialized and ready.\n");
  }

  // getchar();
  // getchar();
  // pthread_mutex_destroy(&m_mutex);

  // // Done - clean up.
  // theClient->Uninitialize();

  return ErrorCode_OK;
}

void clearData() {
  for (auto vel : MarkerVelocityTrackerArray) {
    for (auto v : vel) {
      v.clear();
    }
  }
  MarkerVelocityTrackerArray.clear();

  for (auto vel : MarkerAccelerationTrackerArray) {
    for (auto v : vel) {
      v.clear();
    }
  }
  MarkerAccelerationTrackerArray.clear();

  for (auto vel : BoneVelocityTrackerArray) {
    vel.clear();
  }
  BoneVelocityTrackerArray.clear();

  for (auto vel : BoneAccelerationTrackerArray) {
    vel.clear();
  }
  BoneAccelerationTrackerArray.clear();

  for (auto m : g_mapMarkersSetDesc) {
    m.second.clear();
  }
  g_mapMarkersSetDesc.clear();
  for (auto m : g_SkeletonRigidsIdMp) {
    m.second.clear();
  }
  g_SkeletonRigidsIdMp.clear();
  g_SkeletonIdMap.clear();
  g_mapForcePlateDesc.clear();
  g_RigidBodyIdMap.clear();
}

void readData(void* param) {
  if (param == NULL) {
    return;
  }
  NokovSDKClient* client = (NokovSDKClient*)param;
  sFrameOfMocapData* data = NULL;
  while (!bExit) {
    data = client->GetLastFrameOfMocapData();
    if (data) {
      // printData(data, client);
      printData_pnd(data, client);
    }

    client->NokovFreeFrame(data);
  }

  return;
}

void readDataEx(void* param) {
  if (param == NULL) {
    return;
  }
  int ret = -1;
  NokovSDKClient* client = (NokovSDKClient*)param;
  sFrameOfMocapData* data = NULL;
  while (!bExit) {
    data = client->GetLastFrameOfMocapData();
    if (!data) {
      continue;
    }
    pthread_mutex_lock(&m_mutex);

    // type://1帧号，2markerset数量
    // ，3刚体数量，4骨骼数量，5命名点数量，6未命名点数量，7模拟通道数，8参数
    ret = client->GetLastFrameDataByType(MocapDataParam);  // 参数
    bool bIsRecording = ret & 0x01;
    bool bTrackedModelsChanged = ret & 0x02;
    if (bIsRecording) {
      printf("RECORDING\n");
    }

    if (bTrackedModelsChanged) {
      printf("Models Changed.\n");
    }
    // timecode - for systems with an eSync and SMPTE timecode generator -
    // decode to values
    int hour, minute, second, frame, subframe;
    unsigned int timeCode = client->GetLastFrameTimecode();
    unsigned int timeCodeSubFrame = client->GetLastFrameSubframe();
    bool bValid = client->DecodeTimecode(timeCode, timeCodeSubFrame, &hour, &minute, &second, &frame, &subframe);

    // decode to friendly string
    char szTimecode[128] = "";
    client->TimecodeStringify(timeCode, timeCodeSubFrame, szTimecode, 128);
    int iFrame = client->GetLastFrameDataByType(MocapDataFrameNO);
    long long iTimeStamp = client->GetLastFrameTimeStamp();
    printf("\nFrameNO:%d\tTimeStamp:%lld\t  Timecode:%s\n", iFrame, iTimeStamp, szTimecode);

    int nMarkerSets = client->GetLastFrameDataByType(MocapDataMarkerSetNum);
    printf("MarkerSet [Count=%d]\n", nMarkerSets);  // Number of Markerset

    MarkerVelocityTrackerArray.resize(nMarkerSets);
    MarkerAccelerationTrackerArray.resize(nMarkerSets);

    float* markerData = NULL;
    int i = 0;
    for (auto markerset : g_mapMarkersSetDesc) {
      // sMarkerSetData* markerset = &data->MocapData[i];
      string markerSetName = markerset.first;
      int markerCount = markerset.second.size();
      if (i >= nMarkerSets) {
        break;
      }
      MarkerVelocityTrackerArray[i].resize(markerCount);
      MarkerAccelerationTrackerArray[i].resize(markerCount);
      printf("Markerset%d: %s [nMarkers Count=%d]\n", i + 1, markerSetName.c_str(),
             markerCount);  // Output the id and name of the i-th Markerset and the
                            // total number of named markers in the Markerset
      printf("{\n");
      for (auto marker : markerset.second) {
        int markerIndex = marker.first;
        string markerName = marker.second;
        if (0 != client->GetLastFrameMarkerByName(markerSetName.c_str(), markerName.c_str(), &markerData)) {
          continue;
        }
        printf("\tMarker%d(mm)\tx:%6.2f\ty:%6.2f\tz:%6.2f\n", markerIndex + 1, markerData[0], markerData[1],
               markerData[2]);

        // calculate the velocity and acceleration
        MarkerVelocityTrackerArray[i][markerIndex].Cache(markerData[0], markerData[1], markerData[2], iFrame);
        MarkerAccelerationTrackerArray[i][markerIndex].Cache(markerData[0], markerData[1], markerData[2], iFrame);

        // Caution: Actually, you cat get velocity of frame 2 after you get
        // frame 3's position, But it just has a little difference
        static CalculateVelocity method(g_iFrameRate,
                                        3);  // FPS:60 FrameFactor:3
        static CalculateAcceleration method2(g_iFrameRate,
                                             3);  // FPS:60 FrameFactor:5

        Vel MarkerVelocity;
        MarkerVelocityTrackerArray[i][markerIndex].tryToCalculate(MarkerVelocity, method);

        Accel MarkerAccleration;
        MarkerAccelerationTrackerArray[i][markerIndex].tryToCalculate(MarkerAccleration, method2);

        MarkerVelocity.output(MarkerVelocity);
        MarkerAccleration.output(MarkerAccleration);
      }
      ++i;
      printf("}\n");  // The data output of the i-th Markerset is completed
    }

    int nRigidBodies = client->GetLastFrameDataByType(MocapDataRigidBodyNum);
    BoneVelocityTrackerArray.resize(nRigidBodies);
    BoneAccelerationTrackerArray.resize(nRigidBodies);

    // Print RigidBodies
    sRigidBodyData* rigidData;
    printf("Markerset.RigidBodies [Count=%d]\n",
           nRigidBodies);  // Number of Markerset.Skeleton(skeleton)
    i = 0;
    for (auto rigidBody : g_RigidBodyIdMap) {
      if (i >= nRigidBodies) {
        break;
      }
      string rigidBodyName = rigidBody.second;
      if (0 != client->GetLastFrameRigidBodyByName(rigidBodyName.c_str(), &rigidData)) {
        continue;
      }
      printf("{\n");
      printf("\tname:%s\tid:%02d\n", rigidBodyName.c_str(), rigidData->ID);
      printf("\t     (mm)\tx:%6.2f\ty:%6.2f\tz:%6.2f\n", rigidData->x, rigidData->y, rigidData->z);
      printf("\t\t\tqx:%6.2f\tqy:%6.2f\tqz:%6.2f\tqw:%6.2f\n", rigidData->qx, rigidData->qy, rigidData->qz,
             rigidData->qw);

      // calculate the velocity and acceleration
      BoneVelocityTrackerArray[i].Cache(rigidData->x, rigidData->y, rigidData->z, iFrame);
      BoneAccelerationTrackerArray[i].Cache(rigidData->x, rigidData->y, rigidData->z, iFrame);

      // Caution: Actually, you cat get velocity of frame 2 after you get frame
      // 3's position, But it just has a little difference
      static CalculateVelocity method1(g_iFrameRate, 3);  // FPS:60
                                                          // FrameFactor:3
      static CalculateAcceleration method2(g_iFrameRate,
                                           3);  // FPS:60 FrameFactor:3

      Vel boneVelocity;
      BoneVelocityTrackerArray[i].tryToCalculate(boneVelocity, method1);

      Accel boneAccleration;
      BoneAccelerationTrackerArray[i].tryToCalculate(boneAccleration, method2);

      boneVelocity.output(boneVelocity);
      boneAccleration.output(boneAccleration);
      printf("\tRigidBody markers [Count=%d]\n", rigidData->nMarkers);
      for (int iMarker = 0; iMarker < rigidData->nMarkers;
           iMarker++)  // Output the id and information (x, y, z) of the marker
                       // associated with the j-th RigidBody (RigidBody)
      {
        if (rigidData->MarkerIDs) {
          printf("\tMarker%d(mm)", rigidData->MarkerIDs[iMarker]);
        }

        if (rigidData->Markers) {
          printf("\tx:%6.2f\ty:%6.2f\tz:%6.2f\n", rigidData->Markers[iMarker][0], rigidData->Markers[iMarker][1],
                 rigidData->Markers[iMarker][2]);
        }
      }

      printf("}\n");  // The data output of the j-th RigidBody (RigidBody) is
                      // completed
      ++i;
    }

    // Print Skeletons
    i = 0;
    sRigidBodyData* skeData = NULL;
    int nSkeletons = client->GetLastFrameDataByType(MocapDataSkeletonNum);
    printf("Markerset.Skeletons [Count=%d]\n",
           nSkeletons);  // Number of Markerset.Skeleton(skeleton)
    for (auto rigids : g_SkeletonRigidsIdMp) {
      int skeletonID = rigids.first;
      int nRigidBodies = rigids.second.size();
      string markerSetName = g_SkeletonIdMap[skeletonID];
      printf("Markerset%d [Name=%s].Skeleton [nRigidBodies Count=%d]\n", skeletonID, markerSetName.c_str(),
             nRigidBodies);
      printf("{\n");
      for (auto skeletons : rigids.second) {
        string skeName = skeletons.second;
        if (0 != client->GetLastFrameSkeletonByName(markerSetName.c_str(), skeName.c_str(), &skeData)) {
          continue;
        }
        printf("\tname:%s\tid:%02d\n", skeName.c_str(), skeData->ID);
        printf("\t     (mm)\tx:%6.2f\ty:%6.2f\tz:%6.2f\n", skeData->x, skeData->y, skeData->z);
        printf("\t\t\tqx:%6.2f\tqy:%6.2f\tqz:%6.2f\tqw:%6.2f\n", skeData->qx, skeData->qy, skeData->qz, skeData->qw);

        printf("\tRigidBody markers [Count=%d]\n", skeData->nMarkers);
        for (int iMarker = 0; iMarker < skeData->nMarkers; iMarker++)  // Output the id and information (x, y, z) of the
                                                                       // marker associated with the j-th RigidBody
                                                                       // (RigidBody)
        {
          if (skeData->MarkerIDs) {
            printf("\tMarker%02d(mm)", skeData->MarkerIDs[iMarker]);
          }

          if (skeData->Markers)
            printf("\tx:%6.2f\ty:%6.2f\tz:%6.2f\n", skeData->Markers[iMarker][0], skeData->Markers[iMarker][1],
                   skeData->Markers[iMarker][2]);
        }
      }
      printf("}\n");  // The data output of the j-th RigidBody (RigidBody) is
                      // completed
    }

    int nOtherMarkers = client->GetLastFrameDataByType(MocapDataOtherMarkerNum);
    printf("Other Markers [Count=%d]\n",
           nOtherMarkers);  // Output the total number of unnamed markers contained
    printf("{\n");
    for (i = 0; i < nOtherMarkers; i++)  // Output the id and information (x, y,
                                         // z) of the unnamed marker included
    {
      if (0 != client->GetLastFrameUndefined(i, &markerData)) {
        continue;
      }
      printf("\tMarker%02d(mm)\tx:%6.2f\ty:%6.2f\tz:%6.2f\n", i, markerData[0], markerData[1], markerData[2]);
    }
    printf("}\n");  // The data output of unnamed marker is completed, and the
                    // data output of one frame is completed

    i = 0;
    int nAnalogdatas = client->GetLastFrameDataByType(MocapDataAnalogChNum);
    float fFpData = 0.0;
    printf("Analog [Count=%d]\n",
           nAnalogdatas);  // Output the total number of analog data contained
    printf("{\n");
    if (nAnalogdatas > 0) {
      for (auto fpCh : g_mapForcePlateDesc) {
        string chName = fpCh.second;
        if (0 != client->GetLastFrameAnalogdata(chName.c_str(), &fFpData)) {
          continue;
        }
        printf("\tAnalogData %02d: %6.3f\n", i, fFpData);
        ++i;
      }
    }

    printf("}\n");  // The data output of analog data is completed, and the data
                    // output of one frame is completed

    int nLabeledMarkers = client->GetLastFrameDataByType(MocapDataLabeledMarkerNum);

    client->NokovFreeFrame(data);

    pthread_mutex_unlock(&m_mutex);
  }

  return;
}

// Establish a NokovSDK Client connection
int CreateClient(char* szServerIP) {
  // release previous server
  if (theClient) {
    theClient->Uninitialize();
    delete theClient;
  }

  // create NokovSDK client
  theClient = new NokovSDKClient();

  // print version info
  unsigned char ver[4];
  theClient->NokovSDKVersion(ver);
  printf("NokovSDK Sample Client 2.4.0.5418(NokovSDK ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

  // Set callback handlers
  // theClient->SetMessageCallback(MessageHandler);
  // theClient->SetVerbosityLevel(Verbosity_Error);
  // Init Client and connect to NokovSDK server
  int retCode = -1;
  retCode = theClient->Initialize(szServerIP);  // szMyIPAddress：Client IP (local IP)；szServerIP：Server
                                                // IP (computer IP running motion capture software)
  // Sleep(10);
  memcpy(szServerIPAddress, szServerIP, sizeof(szServerIP));

  if (retCode != ErrorCode_OK) {
    printf("Unable to connect to server.Error code: %d. Exiting\n", retCode);
    return ErrorCode_Internal;
  }

  // print server info
  sServerDescription ServerDescription;
  memset(&ServerDescription, 0, sizeof(ServerDescription));
  theClient->GetServerDescription(&ServerDescription);
  if (!ServerDescription.HostPresent) {
    printf("Unable to connect to server. Host not present. Exiting.\n");
    return 1;
  }
  printf("Successfully connected to server\n");
  printf(
      "\n1: Callback passive receiving data\n2: Host reading data\n3: Read "
      "Data By Name\nEnter 1,2,3 select: ");

  getchar();
  char ch = '2';

  sDataDescriptions* ps = nullptr;
  theClient->GetDataDescriptions(&ps);
  if (ps) {
    printPacketDescriptions(ps);
    theClient->FreeDataDescriptions(ps);
    ps = nullptr;
  }

  // getTposeDescriptions();
  theClient->SetNotifyMsgCallback(NotifyHandler, theClient);
  theClient->WaitForForcePlateInit();
  // theClient->SetDataCallback(DataHandler, theClient);	// this function
  // will receive data from the server
  theClient->SetForcePlateCallback(ForcePlateHandler, theClient);

  if (ch == '2') {  //
    printf("\nselect: %c\n", ch);
    std::thread t(readData, (void*)theClient);
    t.detach();
  } else if (ch == '3') {
    printf("\nselect: %c\n", ch);
    std::thread t(readDataEx, (void*)theClient);
    t.detach();
  } else {  //(ch == '1')
    printf("\nselect: %c\n", ch);
    theClient->SetDataCallback(DataHandler,
                               theClient);  // this function will receive data from the server
    theClient->SetAnalogChCallback(AnlogChHandler, theClient);
  }
  g_cSelect = ch;

  return ErrorCode_OK;
}

void printPacketDescriptions(sDataDescriptions* pData) {
  if (!pData) {
    return;
  }

  pthread_mutex_lock(&m_mutex);

  clearData();

  for (int dsIndex = 0; dsIndex < pData->nDataDescriptions; ++dsIndex) {
    const auto& dataDescription = pData->arrDataDescriptions[dsIndex];

    switch (dataDescription.type) {
      case Descriptor_MarkerSetEx:
        printf("MarkerSetExName : %s\n", dataDescription.Data.MarkerSetData->szName);

        for (int markerIndex = 0; markerIndex < dataDescription.Data.MarkerSetData->nMarkers; ++markerIndex) {
          MarkerData& md = dataDescription.Data.MarkerSetData->Markers[markerIndex];
          printf("Marker[%d] : %fmm,%fmm,%fmm\n", markerIndex, md[0], md[1], md[2]);
        }
        break;
      case Descriptor_MarkerSet: {
        printf("MarkerSetName : %s\n", dataDescription.Data.MarkerSetDescription->szName);
#ifdef LOST_FRAME_TEST
        s_Record.m_MarkerNums += dataDescription.Data.MarkerSetDescription->nMarkers;
#endif
        IdNameMap tmpmap;
        for (int markerIndex = 0; markerIndex < dataDescription.Data.MarkerSetDescription->nMarkers; ++markerIndex) {
          printf("Marker[%d] : %s\n", markerIndex,
                 dataDescription.Data.MarkerSetDescription->szMarkerNames[markerIndex]);
          tmpmap.emplace(markerIndex, dataDescription.Data.MarkerSetDescription->szMarkerNames[markerIndex]);
        }
        g_mapMarkersSetDesc.emplace(dataDescription.Data.MarkerSetData->szName, tmpmap);
        break;
      }
      case Descriptor_RigidBody:
        printf("RigidBody:%s ID:%d\n", dataDescription.Data.RigidBodyDescription->szName,
               dataDescription.Data.RigidBodyDescription->ID);
        g_RigidBodyIdMap.emplace(dataDescription.Data.RigidBodyDescription->ID,
                                 dataDescription.Data.RigidBodyDescription->szName);
#ifdef LOST_FRAME_TEST
        ++s_Record.m_RigNums;
#endif
        break;
      case Descriptor_Skeleton:
        printf("Skeleton name:%s, id:%d, rigids:%d\n", dataDescription.Data.SkeletonDescription->szName,
               dataDescription.Data.SkeletonDescription->skeletonID,
               dataDescription.Data.SkeletonDescription->nRigidBodies);
        {
          IdNameMap rm;
          for (int boneIndex = 0; boneIndex < dataDescription.Data.SkeletonDescription->nRigidBodies; ++boneIndex) {
            auto boneDescription = dataDescription.Data.SkeletonDescription->RigidBodies[boneIndex];
            rm.emplace(boneDescription.ID, boneDescription.szName);
            printf("[%d] %s %d %fmm %fmm %fmm %f %f %f %f\n", boneDescription.ID, boneDescription.szName,
                   boneDescription.parentID, boneDescription.offsetx, boneDescription.offsety, boneDescription.offsetz,
                   boneDescription.qx, boneDescription.qy, boneDescription.qz, boneDescription.qw);
          }

          g_SkeletonRigidsIdMp.emplace(dataDescription.Data.SkeletonDescription->skeletonID, rm);
          g_SkeletonIdMap.emplace(dataDescription.Data.SkeletonDescription->skeletonID,
                                  dataDescription.Data.SkeletonDescription->szName);
        }
        break;
      case Descriptor_ForcePlate:
        for (int channelIdx = 0; channelIdx < dataDescription.Data.ForcePlateDescription->nChannels; ++channelIdx) {
          printf("Channel:%d %s\n", channelIdx, dataDescription.Data.ForcePlateDescription->szChannelNames[channelIdx]);
          g_mapForcePlateDesc.emplace(channelIdx,
                                      dataDescription.Data.ForcePlateDescription->szChannelNames[channelIdx]);
        }
        break;
      case Descriptor_Param: {
        printf("FrameRate: %d\n", dataDescription.Data.DataParam->nFrameRate);
        g_iFrameRate = dataDescription.Data.DataParam->nFrameRate;
      }
      default:
        break;
    }
  }
  pthread_mutex_unlock(&m_mutex);
}

void getTposeDescriptions() {
  char name[100] = "HYK";
  sDataDescriptions* pData = nullptr;

  theClient->GetTposeDataDescriptions(name, &pData);
  // theClient->GetDataDescriptions(&pData);
  if (pData) {
    printPacketDescriptions(pData);
    theClient->FreeDataDescriptions(pData);
    pData = nullptr;
  }

  return;
}

// ForcePlateHandler receives data from the server
void __attribute__((__cdecl__)) ForcePlateHandler(sForcePlates* pForcePlate, void* pUserData) {
  if (nullptr != pForcePlate) {
    for (int plateIdx = 0; plateIdx < pForcePlate->nForcePlates; ++plateIdx) {
      printf(
          "Frame[%d]:ForcePlate[%d]\nFxyz:(%lf,%lf,%lf)\nxyz:(%lf,%lf,%lf)"
          "\nMFree:%lf\n",
          pForcePlate->iFrame, plateIdx, pForcePlate->ForcePlates[plateIdx].Fxyz[0],
          pForcePlate->ForcePlates[plateIdx].Fxyz[1], pForcePlate->ForcePlates[plateIdx].Fxyz[2],
          pForcePlate->ForcePlates[plateIdx].xyz[0], pForcePlate->ForcePlates[plateIdx].xyz[1],
          pForcePlate->ForcePlates[plateIdx].xyz[2], pForcePlate->ForcePlates[plateIdx].Mfree);
    }
  }
}

void printData_pnd(sFrameOfMocapData* data, NokovSDKClient* pClient) {
  int i = 0;

  // FrameOfMocapData params
  bool bIsRecording = data->params & 0x01;
  bool bTrackedModelsChanged = data->params & 0x02;
  if (bIsRecording) printf("RECORDING\n");
  if (bTrackedModelsChanged) printf("Models Changed.\n");

  // timecode - for systems with an eSync and SMPTE timecode generator - decode
  // to values
  int hour, minute, second, frame, subframe;

  bool bValid =
      pClient->DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);

  // decode to friendly string
  char szTimecode[128] = "";
  int iFrame = data->iFrame;
  pClient->TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);

  BoneVelocityTrackerArray.resize(data->nRigidBodies);
  BoneAccelerationTrackerArray.resize(data->nRigidBodies);

  // Print RigidBodies
  // printf("-----------------\r\n");
  for (i = 0; i < data->nRigidBodies; i++) {
    // printf("x:%6.2f\ty:%6.2f\tz:%6.2f\n", data->RigidBodies[i].x,
    //        data->RigidBodies[i].y, data->RigidBodies[i].z);
    // printf("qx:%6.2f\tqy:%6.2f\tqz:%6.2f\tqw:%6.2f\n", data->RigidBodies[i].qx,
    //        data->RigidBodies[i].qy, data->RigidBodies[i].qz,
    //        data->RigidBodies[i].qw);

    // calculate the velocity and acceleration
    BoneVelocityTrackerArray[i].Cache(data->RigidBodies[i].x, data->RigidBodies[i].y, data->RigidBodies[i].z, iFrame);
    BoneAccelerationTrackerArray[i].Cache(data->RigidBodies[i].x, data->RigidBodies[i].y, data->RigidBodies[i].z,
                                          iFrame);

    // Caution: Actually, you cat get velocity of frame 2 after you get frame
    // 3's position, But it just has a little difference
    static CalculateVelocity method1(g_iFrameRate, 3);      // FPS:60 FrameFactor:3
    static CalculateAcceleration method2(g_iFrameRate, 3);  // FPS:60 FrameFactor:3

    Vel boneVelocity;
    BoneVelocityTrackerArray[i].tryToCalculate(boneVelocity, method1);

    Accel boneAccleration;
    BoneAccelerationTrackerArray[i].tryToCalculate(boneAccleration, method2);

    // printf("Vx:%6.2F\tVy:%6.2F\tVz:%6.2F\tVr:%6.2F\r\n", boneVelocity.Vx,
    //        boneVelocity.Vy, boneVelocity.Vz, boneVelocity.Vr);
    // printf("Ax:%6.2F\tAy:%6.2F\tAz:%6.2F\tAr:%6.2F\r\n", boneAccleration.Ax,
    //        boneAccleration.Ay, boneAccleration.Az, boneAccleration.Ar);

    Eigen::Vector3d V;
    Eigen::Vector3d V_;
    Eigen::Matrix3d R;
    Eigen::Vector3d euler_xyz;
    Eigen::Vector4d quaternion;

    V << boneVelocity.Vx, boneVelocity.Vy, boneVelocity.Vz;
    quaternion << data->RigidBodies[i].qx, data->RigidBodies[i].qy, data->RigidBodies[i].qz, data->RigidBodies[i].qw;
    basicfunction::quaternionToEuler(quaternion(3), quaternion(0), quaternion(1), quaternion(2), euler_xyz);
    // euler_xyz.head(2).setZero();
    basicfunction::Euler_XYZToMatrix(R, euler_xyz);
    V_ = R.transpose() * V;  // world to local
    // std::cout << "V:" << V.transpose() << std::endl
    //           <<  "quaternion:" << quaternion.transpose() << std::endl
    //           <<  "euler_xyz:" << euler_xyz.transpose() << std::endl
    //           <<  "R_inv:" << R.transpose() << std::endl
    //           <<  "V_:" << V_.transpose() << std::endl;

    g_v_body_mtx.lock();  // 锁定互斥量
    v_body = V_;
    g_v_body_mtx.unlock();  // 解锁互斥量

    g_euler_world_mtx.lock();  // 锁定互斥量
    euler_world = euler_xyz;
    g_euler_world_mtx.unlock();  // 解锁互斥量
  }
}

void printData(sFrameOfMocapData* data, NokovSDKClient* pClient) {
  int i = 0;

  // FrameOfMocapData params
  bool bIsRecording = data->params & 0x01;
  bool bTrackedModelsChanged = data->params & 0x02;
  if (bIsRecording) printf("RECORDING\n");
  if (bTrackedModelsChanged) printf("Models Changed.\n");

  // timecode - for systems with an eSync and SMPTE timecode generator - decode
  // to values
  int hour, minute, second, frame, subframe;

  bool bValid =
      pClient->DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);

  // decode to friendly string
  char szTimecode[128] = "";
  int iFrame = data->iFrame;
  pClient->TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);
  printf("\nFrameNO:%d\tTimeStamp:%lld\t  Timecode:%s\n", iFrame, data->iTimeStamp, szTimecode);

  printf("MarkerSet [Count=%d]\n", data->nMarkerSets);  // Number of Markerset

  MarkerVelocityTrackerArray.resize(data->nMarkerSets);
  MarkerAccelerationTrackerArray.resize(data->nMarkerSets);

  // Print Markers
  for (i = 0; i < data->nMarkerSets; i++) {
    sMarkerSetData* markerset = &data->MocapData[i];
    MarkerVelocityTrackerArray[i].resize(markerset->nMarkers);
    MarkerAccelerationTrackerArray[i].resize(markerset->nMarkers);

    printf("Markerset%d: %s [nMarkers Count=%d]\n", i + 1, markerset->szName,
           markerset->nMarkers);  // Output the id and name of the i-th Markerset and
                                  // the total number of named markers in the Markerset
    printf("{\n");
    for (int i_Marker = 0; i_Marker < markerset->nMarkers;
         i_Marker++)  // Output the id and information (x, y, z) of the Marker
                      // point contained in the i-th Markerset
    {
      printf("\tMarker%d(mm) \tx:%6.2f\ty:%6.2f\tz:%6.2f\n", i_Marker + 1, markerset->Markers[i_Marker][0],
             markerset->Markers[i_Marker][1], markerset->Markers[i_Marker][2]);

      // calculate the velocity and acceleration
      MarkerVelocityTrackerArray[i][i_Marker].Cache(markerset->Markers[i_Marker][0], markerset->Markers[i_Marker][1],
                                                    markerset->Markers[i_Marker][2], iFrame);
      MarkerAccelerationTrackerArray[i][i_Marker].Cache(
          markerset->Markers[i_Marker][0], markerset->Markers[i_Marker][1], markerset->Markers[i_Marker][2], iFrame);

      // Caution: Actually, you cat get velocity of frame 2 after you get frame
      // 3's position, But it just has a little difference
      static CalculateVelocity method(g_iFrameRate, 3);  // FPS:60 FrameFactor:3
      static CalculateAcceleration method2(g_iFrameRate,
                                           3);  // FPS:60 FrameFactor:5

      Vel MarkerVelocity;
      MarkerVelocityTrackerArray[i][i_Marker].tryToCalculate(MarkerVelocity, method);

      Accel MarkerAccleration;
      MarkerAccelerationTrackerArray[i][i_Marker].tryToCalculate(MarkerAccleration, method2);

      MarkerVelocity.output(MarkerVelocity);
      MarkerAccleration.output(MarkerAccleration);
    }
    printf("}\n");  // The data output of the i-th Markerset is completed
  }

  BoneVelocityTrackerArray.resize(data->nRigidBodies);
  BoneAccelerationTrackerArray.resize(data->nRigidBodies);

  // Print RigidBodies
  printf("Markerset.RigidBodies [Count=%d]\n",
         data->nRigidBodies);  // Number of Markerset.Skeleton(skeleton)
  for (i = 0; i < data->nRigidBodies; i++) {
    printf("{\n");
    printf("\tname:%s\tid:%02d\n", g_RigidBodyIdMap[data->RigidBodies[i].ID].c_str(), data->RigidBodies[i].ID);
    printf("\t     (mm)\tx:%6.2f\ty:%6.2f\tz:%6.2f\n", data->RigidBodies[i].x, data->RigidBodies[i].y,
           data->RigidBodies[i].z);
    printf("\t\t\tqx:%6.2f\tqy:%6.2f\tqz:%6.2f\tqw:%6.2f\n", data->RigidBodies[i].qx, data->RigidBodies[i].qy,
           data->RigidBodies[i].qz, data->RigidBodies[i].qw);

    // calculate the velocity and acceleration
    BoneVelocityTrackerArray[i].Cache(data->RigidBodies[i].x, data->RigidBodies[i].y, data->RigidBodies[i].z, iFrame);
    BoneAccelerationTrackerArray[i].Cache(data->RigidBodies[i].x, data->RigidBodies[i].y, data->RigidBodies[i].z,
                                          iFrame);

    // Caution: Actually, you cat get velocity of frame 2 after you get frame
    // 3's position, But it just has a little difference
    static CalculateVelocity method1(g_iFrameRate, 3);  // FPS:60 FrameFactor:3
    static CalculateAcceleration method2(g_iFrameRate,
                                         3);  // FPS:60 FrameFactor:3

    Vel boneVelocity;
    BoneVelocityTrackerArray[i].tryToCalculate(boneVelocity, method1);

    Accel boneAccleration;
    BoneAccelerationTrackerArray[i].tryToCalculate(boneAccleration, method2);

    boneVelocity.output(boneVelocity);
    boneAccleration.output(boneAccleration);

    printf("\tRigidBody markers [Count=%d]\n", data->RigidBodies[i].nMarkers);
    for (int iMarker = 0; iMarker < data->RigidBodies[i].nMarkers;
         iMarker++)  // Output the id and information (x, y, z) of the marker
                     // associated with the j-th RigidBody (RigidBody)
    {
      // printf("\t\t");
      if (data->RigidBodies[i].MarkerIDs) printf("\tMarker%d(mm)", data->RigidBodies[i].MarkerIDs[iMarker]);
      if (data->RigidBodies[i].Markers)
        printf("\tx:%6.2f\ty:%6.2f\tz:%6.2f\n", data->RigidBodies[i].Markers[iMarker][0],
               data->RigidBodies[i].Markers[iMarker][1], data->RigidBodies[i].Markers[iMarker][2]);
    }

    printf("}\n");  // The data output of the j-th RigidBody (RigidBody) is
                    // completed
  }

  // Print Skeletons
  printf("Markerset.Skeletons [Count=%d]\n",
         data->nSkeletons);  // Number of Markerset.Skeleton(skeleton)
  for (i = 0; i < data->nSkeletons; i++) {
    printf("Markerset%d [Name=%s].Skeleton [nRigidBodies Count=%d]\n", data->Skeletons[i].skeletonID,
           g_SkeletonIdMap[data->Skeletons[i].skeletonID].c_str(),
           data->Skeletons[i].nRigidBodies);  // Skeleton (Skeleton) of the i-th Markerset of
                                              // the motion capture data, including the number
                                              // of RigidBody (RigidBody)
    printf("{\n");
    for (int j = 0; j < data->Skeletons[i].nRigidBodies; j++)  // Output id and information(x, y, z, qx, qy, qz, qw) of
                                                               // the j-th RigidBody (RigidBody)
    {
      printf("\tname:%s\tid:%02d\n",
             g_SkeletonRigidsIdMp[data->Skeletons[i].skeletonID][data->Skeletons[i].RigidBodyData[j].ID].c_str(),
             data->Skeletons[i].RigidBodyData[j].ID);
      printf("\t     (mm)\tx:%6.2f\ty:%6.2f\tz:%6.2f\n", data->Skeletons[i].RigidBodyData[j].x,
             data->Skeletons[i].RigidBodyData[j].y, data->Skeletons[i].RigidBodyData[j].z);
      printf("\t\t\tqx:%6.2f\tqy:%6.2f\tqz:%6.2f\tqw:%6.2f\n", data->Skeletons[i].RigidBodyData[j].qx,
             data->Skeletons[i].RigidBodyData[j].qy, data->Skeletons[i].RigidBodyData[j].qz,
             data->Skeletons[i].RigidBodyData[j].qw);

      printf("\tRigidBody markers [Count=%d]\n", data->Skeletons[i].RigidBodyData[j].nMarkers);
      for (int iMarker = 0; iMarker < data->Skeletons[i].RigidBodyData[j].nMarkers;
           iMarker++)  // Output the id and information (x, y, z) of the marker
                       // associated with the j-th RigidBody (RigidBody)
      {
        if (data->Skeletons[i].RigidBodyData[j].MarkerIDs)
          printf("\tMarker%d(mm)", data->Skeletons[i].RigidBodyData[j].MarkerIDs[iMarker]);
        if (data->Skeletons[i].RigidBodyData[j].Markers)
          printf("\tx:%6.2f\ty:%6.2f\tz:%6.2f\n", data->Skeletons[i].RigidBodyData[j].Markers[iMarker][0],
                 data->Skeletons[i].RigidBodyData[j].Markers[iMarker][1],
                 data->Skeletons[i].RigidBodyData[j].Markers[iMarker][2]);
      }
    }
    printf("}\n");  // The data output of the j-th RigidBody (RigidBody) is
                    // completed
  }
  printf("Other Markers [Count=%d]\n",
         data->nOtherMarkers);  // Output the total number of unnamed markers
                                // contained
  printf("{\n");
  for (i = 0; i < data->nOtherMarkers; i++)  // Output the id and information (x, y, z) of the unnamed marker
                                             // included
  {
    printf("\tMarker%02d(mm)\tx:%6.2f\ty:%6.2f\tz:%6.2f\n", i, data->OtherMarkers[i][0], data->OtherMarkers[i][1],
           data->OtherMarkers[i][2]);
  }
  printf("}\n\n");  // The data output of unnamed marker is completed, and the
                    // data output of one frame is completed
  printf("Analog [Count=%d]\n",
         data->nAnalogdatas);  // Output the total number of analog data contained
  printf("{\n");
  for (int i = 0; i < data->nAnalogdatas; ++i) {
    printf("\tAnalogData %02d: %6.3f\n", i, data->Analogdata[i]);
  }
  printf("}\n\n");  // The data output of analog data is completed, and the data
                    // output of one frame is completed
}

// DataHandler receives data from the server
void __attribute__((__cdecl__)) DataHandler(sFrameOfMocapData* data, void* pUserData) {
  NokovSDKClient* pClient = (NokovSDKClient*)pUserData;

  // printData(data, pClient);
  printData_pnd(data, pClient);
  return;
}

// MessageHandler receives NokovSDK error/debug messages
void __attribute__((__cdecl__)) MessageHandler(int msgType, char* msg) { printf("\n%s\n", msg); }

// NotifyHandler receives NokovSDK notify
void __attribute__((__cdecl__)) NotifyHandler(sNotifyMsg* data, void* pUserData) {
  printf(
      "Notify Type: %d, Value: %d, timestamp:%llu, msg: %s, param1:%d, "
      "param2:%d, param3:%d, param4:%d\n",
      data->nType, data->nValue, data->nTimeStamp, data->sMsg, data->nParam1, data->nParam2, data->nParam3,
      data->nParam4);
  switch (data->nType) {
    case eStartPlay:
    case eDataChange: {
      if (g_cSelect != '3') {
        break;
      }
      sDataDescriptions* ps = nullptr;
      theClient->GetDataDescriptions(&ps);
      if (ps) {
        printPacketDescriptions(ps);
        theClient->FreeDataDescriptions(ps);
        ps = nullptr;
      }
      break;
    }
    case eFrameRateChange: {
      g_iFrameRate = data->nParam1;
    }
    default:
      break;
  }

  // getTposeDescriptions();
  return;
}

void __attribute__((__cdecl__)) AnlogChHandler(sFrameOfAnalogChannelData* anData, void* pUserData) {
  NokovSDKClient* pClient = (NokovSDKClient*)pUserData;
  // FrameOfMocapData params
  bool bIsRecording = anData->params & 0x01;
  bool bTrackedModelsChanged = anData->params & 0x02;
  if (bIsRecording) printf("RECORDING\n");
  if (bTrackedModelsChanged) printf("Models Changed.\n");

  // decode to friendly string
  char szTimecode[128] = "";
  int iFrame = anData->iFrame;
  pClient->TimecodeStringify(anData->Timecode, anData->TimecodeSubframe, szTimecode, 128);
  printf("FrameNO:%d\tTimeStamp:%lld\t Timecode:%s\n", iFrame, anData->iTimeStamp, szTimecode);
  printf("Analog Channel Number:%02d , SubFrame: %02d\n", anData->nAnalogdatas, anData->nSubFrame);

  for (int i = 0; i < anData->nAnalogdatas; i++) {
    printf("Channel %02d ", i);
    for (int j = 0; j < anData->nSubFrame; j++) {
      printf(",%6.3f ", anData->Analogdata[i][j]);
    }
    printf("\n");
  }
  return;
}