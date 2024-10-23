#include <stdio.h>
#include <thread>
#include "keyboard.h"

int main() {
  keyboard key;  // 使用栈上分配的对象
  key.init_keyboard();

  // 创建键盘监听线程
  std::thread kb_thread(keyboard_thread, std::ref(key));

  while (key.running) {
    std::unique_lock<std::mutex> lock(key.mtx);
    key.cv.wait(lock, [&key] { return key.shared_key != 0; });  // 等待新数据

    printf("\n%d\n", key.shared_key);
    key.shared_key = 0;  // 重置共享变量
  }

  // 主线程等待键盘线程结束
  kb_thread.join();

  key.close_keyboard();
  return 0;
}