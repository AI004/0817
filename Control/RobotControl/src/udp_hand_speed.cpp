#include <udp_hand_speed.h>

void HandController::writeRegister(int ser, const char* addr, int id, int add, int num,
                                   const std::vector<uint8_t>& val) {
  sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  inet_pton(AF_INET, addr, &server_addr.sin_addr);

  std::vector<uint8_t> sbytes = {0xEB, 0x90};
  sbytes.push_back(id);       // id
  sbytes.push_back(num + 3);  // len
  sbytes.push_back(0x12);     // cmd
  sbytes.push_back(add & 0xFF);
  sbytes.push_back((add >> 8) & 0xFF);  // add

  for (int i = 0; i < num; i++) {
    sbytes.push_back(val[i]);
  }

  uint8_t checksum = 0x00;
  for (int i = 2; i < sbytes.size(); i++) {
    checksum += sbytes[i];
  }
  checksum &= 0xFF;
  sbytes.push_back(checksum);

  sendto(ser, sbytes.data(), sbytes.size(), 0, (sockaddr*)&server_addr, sizeof(server_addr));
  usleep(10000);
  int flags = fcntl(ser, F_SETFL, flags | O_NONBLOCK);
  if (flags < 0) {
    close(ser);
  }
  if (fcntl(ser, F_SETFL, flags | O_NONBLOCK) < 0) {
    close(ser);
  }
  uint8_t buffer[1024];
  recv(ser, buffer, sizeof(buffer), 0);
}

std::vector<uint8_t> HandController::readRegister(int ser, const char* addr, int id, int add, int num, bool mute) {
  sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  inet_pton(AF_INET, addr, &server_addr.sin_addr);

  std::vector<uint8_t> sbytes = {0xEB, 0x90};
  sbytes.push_back(id);    // id
  sbytes.push_back(0x04);  // len
  sbytes.push_back(0x11);  // cmd
  sbytes.push_back(add & 0xFF);
  sbytes.push_back((add >> 8) & 0xFF);  // add
  sbytes.push_back(num);
  uint8_t checksum = 0x00;
  for (int i = 2; i < sbytes.size(); i++) {
    checksum += sbytes[i];
  }
  checksum &= 0xFF;
  sbytes.push_back(checksum);

  sendto(ser, sbytes.data(), sbytes.size(), 0, (sockaddr*)&server_addr, sizeof(server_addr));
  usleep(10000);

  int flags = fcntl(ser, F_GETFL);
  if (flags < 0) {
    perror("fcntl(F_GETFL) failed");
    close(ser);
    exit(EXIT_FAILURE);
  }

  // 设置非阻塞标志位
  flags |= O_NONBLOCK;
  if (fcntl(ser, F_SETFL, flags) < 0) {
    perror("fcntl(F_SETFL) failed");
    close(ser);
    exit(EXIT_FAILURE);
  }

  uint8_t buffer[1024];
  auto getData = recv(ser, buffer, sizeof(buffer), 0);

  std::vector<uint8_t> val;
  if (getData == -1) {
    return val;
  }

  num = (buffer[3] & 0xFF) - 3;
  for (int i = 0; i < num; i++) {
    val.push_back(buffer[7 + i]);
  }

  if (!mute) {
    std::cout << "read register:";
    for (int i = 0; i < num; i++) {
      std::cout << (int)val[i] << " ";
    }
    std::cout << std::endl;
  }
  return val;
}

void HandController::write6(int ser, const char* addr, int id, const std::string& str, const std::vector<int>& val) {
  if (str == "angleSet" || str == "forceSet" || str == "speedSet") {
    std::vector<uint8_t> val_reg;
    for (int i = 0; i < 6; i++) {
      val_reg.push_back(val[i] & 0xFF);
      val_reg.push_back((val[i] >> 8) & 0xFF);
    }
    writeRegister(ser, addr, id, regdict[str], 12, val_reg);
  }
}

std::vector<int> HandController::read6(int ser, const char* addr, int id, const std::string& str) {
  std::vector<int> val_act;  // 定义返回值类型为 std::vector<int>

  if (str == "angleSet" || str == "forceSet" || str == "speedSet" || str == "angleAct" || str == "forceAct") {
    std::vector<uint8_t> val = readRegister(ser, addr, id, regdict[str], 12, true);
    if (val.size() < 12) {
      // 返回一个空的 std::vector<int> 表示读取失败
      val_act.assign(6, -1);
      return val_act;
    }

    val_act.resize(6);  // 确保 val_act 大小为 6
    for (int i = 0; i < 6; i++) {
      val_act[i] = (val[2 * i] & 0xFF) + (val[1 + 2 * i] << 8);
    }
    // std::cout << "read close value:";
    // for (int i = 0; i < 6; i++)
    // {
    //     std::cout << val_act[i] << " ";
    // }
    // std::cout << std::endl;
  } else if (str == "errCode" || str == "statusCode" || str == "temp") {
    std::vector<uint8_t> val_act_raw = readRegister(ser, addr, id, regdict[str], 6, true);
    if (val_act_raw.size() < 6) {
      // 返回一个空的 std::vector<int> 表示读取失败
      val_act.assign(6, -1);
      return val_act;
    }

    val_act.resize(6);  // 确保 val_act 大小为 6
    for (int i = 0; i < 6; i++) {
      val_act[i] = static_cast<int>(val_act_raw[i]);
    }
  }

  return val_act;
}

void HandController::controlHands() {
  while (running) {
    if (!left_values.empty() && !right_values.empty()) {
      values_mutex.lock();
      // hand_ctrl("l", "angleSet", left_values);
      // hand_ctrl("r", "angleSet", right_values);
      write6(sok_l, addr_l, 1, "angleSet", left_values);
      write6(sok_r, addr_r, 1, "angleSet", right_values);
      values_mutex.unlock();
    }
    getHandAngles();
    sleep(0.02);
  }
}

void HandController::getHandAngles() {
  // 更新左右手的关节信息
  left_hand_angles = read6(sok_l, addr_l, 1, "angleAct");
  right_hand_angles = read6(sok_r, addr_r, 1, "angleAct");
  for (int i = 0; i < 12; i++) {
    if (i < 6) {
      val_act_buff[i] = left_hand_angles[i];
    } else {
      val_act_buff[i] = right_hand_angles[i - 6];
    }
  }
}

void HandController::updateHandControlValues(const std::vector<int>& values) {
  if (values.size() != 12) {
    throw std::invalid_argument("Expected 12 integers for hand control");
  }
  values_mutex.lock();
  left_values.assign(values.begin(), values.begin() + 6);
  right_values.assign(values.begin() + 6, values.end());
  values_mutex.unlock();
}

// void HandController::hand_ctrl(const std::string &lr, const std::string &str, const std::vector<int> &val)
// {
//     if (lr == "l")
//     {
//         write6(sok_l, addr_l, 1, str, val);
//     }
//     else if (lr == "r")
//     {
//         write6(sok_r, addr_r, 1, str, val);
//     }
// }

// void HandController::hand_get_angle(const std::string &lr)
// {
//     if (lr == "l")
//     {
//         read6(sok_l, addr_l, 1, "angleAct");
//     }
//     else if (lr == "r")
//     {
//         read6(sok_r, addr_r, 1, "angleAct");
//     }
// }