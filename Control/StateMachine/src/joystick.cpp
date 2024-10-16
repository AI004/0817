#include "../include/joystick.h"
#include <iostream>
Joystick_humanoid::Joystick_humanoid() { exit_flag_.store(false); }

Joystick_humanoid::~Joystick_humanoid() {
  exit_flag_.store(true);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "joystick end" << std::endl;
}

int Joystick_humanoid::xbox_open(const char* file_name) {
  int xbox_fd;

  xbox_fd = open(file_name, O_RDONLY);
  if (xbox_fd < 0) {
    perror("open");
    return -1;
  }

  return xbox_fd;
}

int Joystick_humanoid::xbox_map_read(xbox_map_t* map) {
  int len, type, number, value;
  struct js_event js;

  len = read(xbox_fd, &js, sizeof(struct js_event));
  if (len < 0) {
    perror("read");
    return -1;
  }

  type = js.type;
  number = js.number;
  value = js.value;

  map->time = js.time;

  if (type == JS_EVENT_BUTTON) {
    switch (number) {
      case XBOX_BUTTON_A:
        map->a = value;
        break;

      case XBOX_BUTTON_B:
        map->b = value;
        break;

      case XBOX_BUTTON_X:
        map->x = value;
        break;

      case XBOX_BUTTON_Y:
        map->y = value;
        break;

      case XBOX_BUTTON_LB:
        map->lb = value;
        break;

      case XBOX_BUTTON_RB:
        map->rb = value;
        break;

      case XBOX_BUTTON_START:
        map->start = value;
        break;

      case XBOX_BUTTON_BACK:
        map->back = value;
        break;

      case XBOX_BUTTON_HOME:
        map->home = value;
        break;

      case XBOX_BUTTON_LO:
        map->lo = value;
        break;

      case XBOX_BUTTON_RO:
        map->ro = value;
        break;

      case XBOX_BUTTON_SEL:
        map->sel = value;
        break;

      default:
        break;
    }
  } else if (type == JS_EVENT_AXIS) {
    switch (number) {
      case XBOX_AXIS_LX:
        map->lx = value;
        break;

      case XBOX_AXIS_LY:
        map->ly = value;
        break;

      case XBOX_AXIS_RX:
        map->rx = value;
        break;

      case XBOX_AXIS_RY:
        map->ry = value;
        break;

      case XBOX_AXIS_LT:
        map->lt = value;
        break;

      case XBOX_AXIS_RT:
        map->rt = value;
        break;

      case XBOX_AXIS_XX:
        map->xx = value;
        break;

      case XBOX_AXIS_YY:
        map->yy = value;
        break;

      default:
        break;
    }
  } else {
    /* Init do nothing */
  }

  return len;
}

void Joystick_humanoid::xbox_close(void) {
  close(xbox_fd);
  return;
}

int Joystick_humanoid::xbox_init(void) {
  int len, type;
  int axis_value, button_value;
  int number_of_axis, number_of_buttons;

  xbox_fd = xbox_open("/dev/input/js0");
  if (xbox_fd < 0) {
    return -1;
  }

  return 0;
}

int Joystick_humanoid::xbox_read(xbox_map_t* xbox_m) {
  int len = xbox_map_read(xbox_m);
  if (len < 0) {
    return -1;
  }
  return 0;
}

void Joystick_humanoid::init() {
  int ret = -1;
  xbox_m.a = 0.0;
  xbox_m.b = 0.0;
  xbox_m.x = 0.0;
  xbox_m.y = 0.0;
  xbox_m.lx = 0.0;
  xbox_m.ly = 0.0;
  xbox_m.rx = 0.0;
  xbox_m.ry = 0.0;
  xbox_m.xx = 0.0;
  xbox_m.yy = 0.0;
  xbox_m.rb = 0.0;
  xbox_m.lb = 0.0;
  xbox_m.lt = -32767;
  xbox_m.rt = -32767;
  xbox_m.ro = 0.0;
  xbox_m.lo = 0.0;
  xbox_m.sel = 0;
  current_fsmstate_command = "";
  // ret = pthread_create(&Joystick::xbox_thread, NULL, Joystick::xbox_run,
  // NULL);
  xbox_thread = std::thread(&Joystick_humanoid::xbox_run, this);
  xbox_thread.detach();
}

void Joystick_humanoid::xbox_run() {
  int len, ret;
  while (1) {
    ret = xbox_init();
    if (ret < 0) {
      printf("xbox init fail\n");
      usleep(1000 * 1000);
    } else {
      std::cout << "xbox connect!" << std::endl;
    }

    while (ret == 0) {
      len = xbox_read(&xbox_m);
      if (len < 0) {
        std::cout << "xbox disconnect!" << std::endl;
        break;
      }

      // printf("A:%d B:%d X:%d Y:%d\n", xbox_m.a, xbox_m.b, xbox_m.x,
      // xbox_m.y); printf("RB:%d ST:%d BK:%d HM:%d\n", xbox_m.rb, xbox_m.start,
      // xbox_m.back, xbox_m.home); printf("LX:%d LY:%d RX:%d RY:%d\n",
      // xbox_m.lx, xbox_m.ly, xbox_m.rx, xbox_m.ry); printf("LB:%d LT:%d RB:%d
      // RT:%d\n", xbox_m.lb, xbox_m.lt, xbox_m.rb, xbox_m.rt); printf("LO:%d
      // RO:%d SEL:%d\n", xbox_m.lo, xbox_m.ro, xbox_m.sel);

      // printf("Time:%8d A:%d B:%d X:%d Y:%d LB:%  m.b, xbox_m.x, xbox_m.y,
      // xbox_m.lb, xbox_m.rb, xbox_m.start, xbox_m.back, xbox_m.home,
      // xbox_m.lo, xbox_m.ro,
      //         xbox_m.xx, xbox_m.yy, xbox_m.lx, xbox_m.ly, xbox_m.rx,
      //         xbox_m.ry, xbox_m.lt, xbox_m.rt);
      // fflush(stdout);

      usleep(10 * 1000);
      if (exit_flag_.load()) {
        break;
      }
    }
  }
  std::cout << "xbox run end" << std::endl;
}

std::string Joystick_humanoid::get_state_change() {
  if (xbox_m.a == 1.0) {
    current_fsmstate_command = "gotoZero";
    return "gotoZero";
  } else if (xbox_m.b == 1.0) {
    // current_fsmstate_command = "gotoDual2Single";
    // return "gotoDual2Single";
    current_fsmstate_command = "gotoStop";
    return "gotoStop";
    // current_fsmstate_command = "gotoSwing";
    // return "gotoSwing";
  } else if (xbox_m.x == 1.0) {
    current_fsmstate_command = "gotoZ2S";
    return "gotoZ2S";
  } else if (xbox_m.y == 1.0) {
    // return "gotoWalk";
    current_fsmstate_command = "gotoS2W";
    return "gotoS2W";
  } else {
    return "";
    // return "gotoZero";
  }
}

std::string Joystick_humanoid::get_current_state_command() { return current_fsmstate_command; }

double Joystick_humanoid::get_walk_x_direction_speed() {
  int lt_value = xbox_m.lt;
  if (lt_value < 1000) {
    int x_value = xbox_m.ly;
    if ((abs(x_value) > deadarea) && (abs(x_value) <= maxvalue)) {
      if (x_value > 0) {
        return ly_dir * maxspeed_x * ((double)(abs(x_value) - deadarea) / (maxvalue - deadarea));
      } else {
        return ly_dir * minspeed_x * ((double)(abs(x_value) - deadarea) / (maxvalue - deadarea));
      }
    } else {
      return 0.0;
    }
  } else {
    return 0.5;
  }
}

double Joystick_humanoid::get_walk_y_direction_speed() {
  int y_value = xbox_m.lx;
  if ((abs(y_value) > deadarea) && (abs(y_value) <= maxvalue)) {
    if (y_value > 0) {
      return lx_dir * maxspeed_y * ((double)(abs(y_value) - deadarea) / (maxvalue - deadarea));
    } else {
      return lx_dir * minspeed_y * ((double)(abs(y_value) - deadarea) / (maxvalue - deadarea));
    }
  } else {
    return 0.0;
  }
}

double Joystick_humanoid::get_walk_yaw_direction_speed() {
  int yaw_value = xbox_m.rx;
  if ((abs(yaw_value) > deadarea) && (abs(yaw_value) <= maxvalue)) {
    if (yaw_value > 0) {
      return rx_dir * maxspeed_yaw * ((double)(abs(yaw_value) - deadarea) / (maxvalue - deadarea));
    } else {
      return rx_dir * minspeed_yaw * ((double)(abs(yaw_value) - deadarea) / (maxvalue - deadarea));
    }
  } else {
    return 0.0;
  }
}

double Joystick_humanoid::get_walk_x_direction_speed_offset() {
  int x_value = xbox_m.yy;
  if ((x_value < -30000) && (abs(last_value_x) < 500)) {
    last_value_x = x_value;
    return deltoffset_x;
  } else if ((x_value > 30000) && (abs(last_value_x) < 500)) {
    last_value_x = x_value;
    return -deltoffset_x;
  } else {
    last_value_x = x_value;
    return 0.0;
  }
}

double Joystick_humanoid::get_walk_y_direction_speed_offset() {
  int y_value = xbox_m.xx;
  if ((y_value < -30000) && (abs(last_value_y) < 500)) {
    last_value_y = y_value;
    return deltoffset_y;
  } else if ((y_value > 30000) && (abs(last_value_y) < 500)) {
    last_value_y = y_value;
    return -deltoffset_y;
  } else {
    last_value_y = y_value;
    return 0.0;
  }
}

// get command position for stand
double Joystick_humanoid::get_stand_x_direction_position() {
  int x_value = xbox_m.ly;
  if ((abs(x_value) > deadarea) && (abs(x_value) <= maxvalue)) {
    if (x_value > 0) {
      return ly_dir * maxposition_x * ((double)(abs(x_value) - deadarea) / (maxvalue - deadarea));
    } else {
      return ly_dir * minposition_x * ((double)(abs(x_value) - deadarea) / (maxvalue - deadarea));
    }
  } else {
    return 0.0;
  }
}

double Joystick_humanoid::get_stand_y_direction_posiiton() {
  int y_value = xbox_m.lx;
  if ((abs(y_value) > deadarea) && (abs(y_value) <= maxvalue)) {
    if (y_value > 0) {
      return lx_dir * maxposition_y * ((double)(abs(y_value) - deadarea) / (maxvalue - deadarea));
    } else {
      return lx_dir * minposition_y * ((double)(abs(y_value) - deadarea) / (maxvalue - deadarea));
    }
  } else {
    return 0.0;
  }
}

double Joystick_humanoid::get_stand_z_direction_posiiton() {
  int z_value = xbox_m.ry;
  if ((abs(z_value) > deadarea) && (abs(z_value) <= maxvalue)) {
    if (z_value > 0) {
      return ry_dir * maxposition_z * ((double)(abs(z_value) - deadarea) / (maxvalue - deadarea));
    } else {
      return ry_dir * minposition_z * ((double)(abs(z_value) - deadarea) / (maxvalue - deadarea));
    }
  } else {
    return 0.0;
  }
}

double Joystick_humanoid::get_stand_roll_direction_position() {
  int x_value = xbox_m.yy;
  if ((x_value < -30000)) {
    if (roll_cmd < maxposition_roll) {
      roll_cmd += rolldelt_cmd;
    } else {
      roll_cmd = maxposition_roll;
    }
    return roll_cmd;
  } else if ((x_value > 30000)) {
    if (roll_cmd > minposition_roll) {
      roll_cmd -= rolldelt_cmd;
    } else {
      roll_cmd = minposition_roll;
    }
    return roll_cmd;
  } else {
    return roll_cmd;
  }
}

double Joystick_humanoid::get_stand_pitch_direction_posiiton() {
  int x_value = xbox_m.xx;
  if ((x_value < -30000)) {
    if (pitch_cmd < maxposition_pitch) {
      pitch_cmd += pitchdelt_cmd;
    } else {
      pitch_cmd = maxposition_pitch;
    }
    return pitch_cmd;
  } else if ((x_value > 30000)) {
    if (pitch_cmd > minposition_pitch) {
      pitch_cmd -= pitchdelt_cmd;
    } else {
      pitch_cmd = minposition_pitch;
    }
    return pitch_cmd;
  } else {
    return pitch_cmd;
  }
}

double Joystick_humanoid::get_stand_yaw_direction_posiiton() {
  int yaw_value = xbox_m.rx;
  if ((abs(yaw_value) > deadarea) && (abs(yaw_value) <= maxvalue)) {
    if (yaw_value > 0) {
      return rx_dir * maxposition_yaw * ((double)(abs(yaw_value) - deadarea) / (maxvalue - deadarea));
    } else {
      return rx_dir * minposition_yaw * ((double)(abs(yaw_value) - deadarea) / (maxvalue - deadarea));
    }
  } else {
    return 0.0;
  }
}

bool Joystick_humanoid::get_motion_state() { return xbox_m.rb; }

bool Joystick_humanoid::get_carry_box_state() { return xbox_m.lb; }
bool Joystick_humanoid::get_if_stand_carry() { return xbox_m.lt > 1000; }
bool Joystick_humanoid::get_gait_mode() { return xbox_m.rb; }
bool Joystick_humanoid::get_foot_rotate() {
  if (xbox_m.rt < 1000)
    return false;
  else
    return true;
}

bool Joystick_humanoid::disableJoints() {
  if (xbox_m.lt > 1000 && xbox_m.rt > 1000) {
    return true;
  } else {
    return false;
  }
}
bool Joystick_humanoid::get_calibration_flag() { return xbox_m.sel; }

bool Joystick_humanoid::get_momentumController_on() {
  double ro = xbox_m.ro;

  if (!button_processed_ && ro == 1.0) {
    get_momentumController_on_flag_ = !get_momentumController_on_flag_;
    std::cout << "momentumTurnOn changed to: " << get_momentumController_on_flag_ << std::endl;
    button_processed_ = true;
  } else if (ro == 0.0) {
    button_processed_ = false;
  }

  return get_momentumController_on_flag_;
};