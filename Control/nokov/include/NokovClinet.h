#ifndef NOKOVCLIENT_H
#define NOKOVCLIENT_H
#include "NokovSDKClient.h"

int CreateClient(char* serverIp);
void printData(sFrameOfMocapData* data, NokovSDKClient* pClient);
void printData_pnd(sFrameOfMocapData* data, NokovSDKClient* pClient);
void printPacketDescriptions(sDataDescriptions* pData);
void getTposeDescriptions();
int NokovClient();

#endif