#include "../include/DataPackage.h"
// #include "DataPackage.h"
DataPackage::DataPackage() {}

int DataPackage::addlog(std::string log_) {
  log_buffer.push_back(log_);
  return log_buffer.size();
}

int DataPackage::clearlog() {
  if (log_buffer.size() > 0) {
    log_buffer.clear();
  }
  return log_buffer.size();
}
