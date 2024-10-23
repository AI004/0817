#ifndef KEYBOARD_H_
#define KEYBOARD_H_

#include <termios.h>
#include <unistd.h>
#include <atomic>
#include <condition_variable>
#include <mutex>

class keyboard {
 public:
  keyboard();
  ~keyboard();

  bool kbhit();
  int readch();

  void init_keyboard();
  void close_keyboard();

  // 用于控制线程的退出
  std::atomic<bool> running;

  // 共享变量用于存储键盘值
  char shared_key;
  std::mutex mtx;
  std::condition_variable cv;

 private:
  struct termios initial_settings, new_settings;
  int peek_character;
};

// 将 keyboard_thread 函数声明为 inline
inline void keyboard_thread(keyboard& key) {
  while (key.running) {
    if (key.kbhit()) {  // 检查是否有按键输入
      int ch = key.readch();
      {
        std::lock_guard<std::mutex> lock(key.mtx);
        key.shared_key = ch;
      }
      key.cv.notify_one();  // 通知主线程有新数据

      if (ch == 'q') {  // 按下 'q' 键时退出循环
        key.running = false;
      }
    }
  }
}

#endif  // KEYBOARD_H_