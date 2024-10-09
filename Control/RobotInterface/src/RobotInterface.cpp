//
// 7/3/2023.
//

#include "RobotInterface.h"
#include "../RobotInterfaceImpl.h"

RobotInterface *get_robot_interface() { return new RobotInterfaceImpl(); };