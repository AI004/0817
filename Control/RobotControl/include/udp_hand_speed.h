#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

class HandController {
 private:
  void writeRegister(int ser, const char* addr, int id, int add, int num, const std::vector<uint8_t>& val);
  std::vector<uint8_t> readRegister(int ser, const char* addr, int id, int add, int num, bool mute = false);

  void write6(int ser, const char* addr, int id, const std::string& str, const std::vector<int>& val);
  std::vector<int> read6(int ser, const char* addr, int id, const std::string& str);
  void getHandAngles();
  void controlHands();
  void hand_ctrl(const std::string& lr, const std::string& str, const std::vector<int>& val);

  // int set_socket_nonblocking(int sockfd);
  std::vector<int> left_hand_angles;
  std::vector<int> right_hand_angles;
  std::vector<int> val_act;
  std::vector<int> left_values;
  std::vector<int> right_values;
  std::thread control_thread;
  std::mutex values_mutex;
  std::atomic<bool> running;
  bool get_data = true;

  int port = 2562;
  const char* addr_l = "10.10.10.18";
  const char* addr_r = "10.10.10.38";

  int sok_l, sok_r;
  int data_count = 0;

  std::map<std::string, int> regdict = {
      {"ID", 1000},         {"baudrate", 1001}, {"clearErr", 1004},  {"forceClb", 1009}, {"angleSet", 1486},
      {"forceSet", 1498},   {"speedSet", 1522}, {"angleAct", 1546},  {"forceAct", 1582}, {"errCode", 1606},
      {"statusCode", 1612}, {"temp", 1618},     {"actionSeq", 2320}, {"actionRun", 2322}};

 public:
  HandController(const std::vector<int>& values)
      : left_values(values.begin(), values.begin() + 6), right_values(values.begin() + 6, values.end()), running(true) {
    if (values.size() != 12) {
      throw std::invalid_argument("Expected 12 integers for hand control");
    }
    val_act.assign(6, 0);
    left_hand_angles.assign(6, 0);
    right_hand_angles.assign(6, 0);
    val_act_buff.assign(12, 0);
    sok_l = socket(AF_INET, SOCK_DGRAM, 0);
    sok_r = socket(AF_INET, SOCK_DGRAM, 0);
    if (sok_l < 0 || sok_r < 0) {
      throw std::runtime_error("Failed to create socket");
    }
    control_thread = std::thread([this]() { this->controlHands(); });
  }
  ~HandController() {
    running = false;
    if (control_thread.joinable()) {
      control_thread.join();
    }
    close(sok_l);
    close(sok_r);
    std::cout << "HandController destructed, thread terminated." << std::endl;
  }
  void updateHandControlValues(const std::vector<int>& values);
  void hand_get_angle(const std::string& lr);
  std::vector<int> getHandsState() { return val_act_buff; };
  std::vector<int> val_act_buff;
};