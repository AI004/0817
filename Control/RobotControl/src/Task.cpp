#include "../include/Task.h"
// #include "Task.h"
Task::Task() {
  controller = new Controller_Lib;
  IG.setZero(6, 6);
}
Task::~Task() { delete controller; }
