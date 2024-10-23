#include "keyboard.h"
#include <stdio.h>

keyboard::keyboard() : peek_character(-1), running(true), shared_key(0) { init_keyboard(); }

keyboard::~keyboard() { close_keyboard(); }

void keyboard::init_keyboard() {
  tcgetattr(0, &initial_settings);
  new_settings = initial_settings;
  new_settings.c_lflag &= ~(ICANON | ECHO);  // 设置为非规范模式，关闭回显
  new_settings.c_cc[VMIN] = 1;
  new_settings.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &new_settings);
}

void keyboard::close_keyboard() { tcsetattr(0, TCSANOW, &initial_settings); }

bool keyboard::kbhit() {
  unsigned char ch;
  int nread;

  if (peek_character != -1) return true;
  new_settings.c_cc[VMIN] = 0;
  tcsetattr(0, TCSANOW, &new_settings);
  nread = read(0, &ch, 1);
  new_settings.c_cc[VMIN] = 1;
  tcsetattr(0, TCSANOW, &new_settings);
  if (nread == 1) {
    peek_character = ch;
    return true;
  }
  return false;
}

int keyboard::readch() {
  char ch;

  if (peek_character != -1) {
    ch = peek_character;
    peek_character = -1;
    return ch;
  }
  read(0, &ch, 1);
  return ch;
}