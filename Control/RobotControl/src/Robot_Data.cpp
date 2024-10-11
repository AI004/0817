#include "../include/Robot_Data.h"
// #include "Robot_Data.h"
Robot_Data::Robot_Data() { carryBoxFirstStand = true; }

Robot_Data::~Robot_Data() {
  for (std::vector<Task*>::iterator iter = task_card_set.begin(); iter != task_card_set.end(); iter++) {
    delete (*iter);
  }
}

int Robot_Data::addlog(std::string log_) {
  log_buffer.push_back(log_);
  return log_buffer.size();
}

int Robot_Data::clearlog() {
  if (log_buffer.size() > 0) {
    log_buffer.clear();
  }
  return log_buffer.size();
}
