//
// 7/3/2023.
//

#include "RobotInterface.h"

#include <openssl/md5.h>

#include <fstream>
#include <iomanip>
#include <sstream>

#include "../RobotInterfaceImpl.h"

void VerifyPndCppSdkHash() {
  std::ifstream filename("libpnd.so.1.5.1", std::ios::binary);
  if (!filename) {
    exit(-1);
  }
  MD5_CTX ctx;
  MD5_Init(&ctx);
  char DataBuff[1024];
  while (!filename.eof()) {
    filename.read(DataBuff, 1024);  // 读文件
    int length = filename.gcount();
    if (length) {
      MD5_Update(&ctx, DataBuff, length);  // 将当前文件块加入并更新MD5
    }
  }
  unsigned char hash[MD5_DIGEST_LENGTH] = {0};
  MD5_Final(hash, &ctx);
  std::stringstream ss;
  ss << std::hex << std::setfill('0');
  for (int i = 0; i < MD5_DIGEST_LENGTH; i++) {
    ss << std::setw(2) << static_cast<int>(hash[i]);
  }
  if (ss.str() != "66d780e39403140e83515340bee0ccb2") {
    std::cerr << "no no no!!!" << std::endl;
    exit(-1);
  }
}

RobotInterface* get_robot_interface() {
  VerifyPndCppSdkHash();
  return new RobotInterfaceImpl();
};
