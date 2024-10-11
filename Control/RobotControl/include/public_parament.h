#ifndef PUBLIC_PARAMENT_H
#define PUBLIC_PARAMENT_H
#include <QString>
#pragma once
extern int adam_type;
extern int floating_base_dof;
extern int actor_num;
extern int generalized_coordinates;

extern int leg_l_actor_num;
extern int leg_r_actor_num;

extern int waist_actor_num;

extern int arm_l_actor_num;
extern int hand_l_actor_num;

extern int arm_r_actor_num;
extern int hand_r_actor_num;
extern int head_actor_num;

extern int adam_upper_actor_num;
extern int adam_upper_except_waist_actor_num;

enum ADAM_TYPE { AdamLite = 0, AdamStandard, StandardPlus23, StandardPlus29, StandardPlus53, AdamLiteSimple, DuckDuck };
#endif