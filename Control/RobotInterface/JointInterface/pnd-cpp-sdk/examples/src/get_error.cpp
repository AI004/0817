#include <iostream>
#include <thread>
#include <vector>

#include "groupCommand.cpp"
#include "lookup.hpp"

using namespace Pnd;

int main() {
  std::string str("10.10.10.255");
  Lookup lookup(&str);  // After construction,start the background thread lookup actuator
  // Wait 1 seconds for the module list to populate, and then print out its
  // contents
  std::this_thread::sleep_for(std::chrono::seconds(1));
  lookup.setLookupFrequencyHz(0);  // set lookup stop

  std::shared_ptr<Group> group = lookup.getGroupFromFamily("Default");
  std::cout << std::endl << "group size: " << group->size() << std::endl;
  auto entry_list = lookup.getEntryList();
  for (const auto &entry : *entry_list) {
    std::cout << "Name: " << entry.name_ << std::endl;
    std::cout << "Family: " << entry.family_ << std::endl;
  }

  GroupCommand group_command(group->size());

  group_command.getError(std::vector<bool>(group->size(), true));
  group->sendCommand(group_command);

  return 0;
}