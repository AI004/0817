#include "webotsInterface.h"
#include "public_parament.h"
using namespace webots;

void WebotsRobot::initWebots() {
  // motors
  legMotor.resize(nJoint);
  int i = 0;
  legMotor[i] = robot->getMotor("hipPitch_Left");i++;
  legMotor[i] = robot->getMotor("hipRoll_Left");i++;
  legMotor[i] = robot->getMotor("hipYaw_Left");i++;
  legMotor[i] = robot->getMotor("kneePitch_Left");i++;
  legMotor[i] = robot->getMotor("anklePitch_Left");i++;
  legMotor[i] = robot->getMotor("ankleRoll_Left");i++;
  legMotor[i] = robot->getMotor("hipPitch_Right");i++;
  legMotor[i] = robot->getMotor("hipRoll_Right");i++;
  legMotor[i] = robot->getMotor("hipYaw_Right");i++;
  legMotor[i] = robot->getMotor("kneePitch_Right");i++;
  legMotor[i] = robot->getMotor("anklePitch_Right");i++;
  legMotor[i] = robot->getMotor("ankleRoll_Right");i++;
  
  //waist
  if(adam_type==ADAM_TYPE::AdamLite){
    legMotor[i] = robot->getMotor("waistRoll");i++;
    legMotor[i] = robot->getMotor("waistPitch");i++;
    legMotor[i] = robot->getMotor("waistYaw");i++;
  }else if(adam_type==ADAM_TYPE::AdamLiteSimple){
  }else if(adam_type==ADAM_TYPE::AdamStandard){
    legMotor[i] = robot->getMotor("waistRoll");i++;
    legMotor[i] = robot->getMotor("waistPitch");i++;
    legMotor[i] = robot->getMotor("waistYaw");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus23){
    legMotor[i] = robot->getMotor("waistRoll");i++;
    legMotor[i] = robot->getMotor("waistPitch");i++;
    legMotor[i] = robot->getMotor("waistYaw");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus29){
    legMotor[i] = robot->getMotor("waistRoll");i++;
    legMotor[i] = robot->getMotor("waistPitch");i++;
    legMotor[i] = robot->getMotor("waistYaw");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus53){
    legMotor[i] = robot->getMotor("waistRoll");i++;
    legMotor[i] = robot->getMotor("waistPitch");i++;
    legMotor[i] = robot->getMotor("waistYaw");i++;
  }

  // left arm and hand
  if(adam_type==ADAM_TYPE::AdamLite){
    legMotor[i] = robot->getMotor("shoulderPitch_Left");i++;
    legMotor[i] = robot->getMotor("shoulderRoll_Left");i++;
    legMotor[i] = robot->getMotor("shoulderYaw_Left");i++;
    legMotor[i] = robot->getMotor("elbow_Left");i++;
  }else if(adam_type==ADAM_TYPE::AdamLiteSimple){
  }else if(adam_type==ADAM_TYPE::AdamStandard){
    legMotor[i] = robot->getMotor("shoulderPitch_Left");i++;
    legMotor[i] = robot->getMotor("shoulderRoll_Left");i++;
    legMotor[i] = robot->getMotor("shoulderYaw_Left");i++;
    legMotor[i] = robot->getMotor("elbow_Left");i++;

    legMotor[i] = robot->getMotor("wristYaw_Left");i++;
    legMotor[i] = robot->getMotor("wristPitch_Left");i++;
    legMotor[i] = robot->getMotor("wristRoll_Left");i++;
    legMotor[i] = robot->getMotor("gripper_Left");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus23){
    legMotor[i] = robot->getMotor("shoulderPitch_Left");i++;
    legMotor[i] = robot->getMotor("shoulderRoll_Left");i++;
    legMotor[i] = robot->getMotor("shoulderYaw_Left");i++;
    legMotor[i] = robot->getMotor("elbow_Left");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus29){
    legMotor[i] = robot->getMotor("shoulderPitch_Left");i++;
    legMotor[i] = robot->getMotor("shoulderRoll_Left");i++;
    legMotor[i] = robot->getMotor("shoulderYaw_Left");i++;
    legMotor[i] = robot->getMotor("elbow_Left");i++;
    
    legMotor[i] = robot->getMotor("wristYaw_Left");i++;
    legMotor[i] = robot->getMotor("wristPitch_Left");i++;
    legMotor[i] = robot->getMotor("wristRoll_Left");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus53){
    legMotor[i] = robot->getMotor("shoulderPitch_Left");i++;
    legMotor[i] = robot->getMotor("shoulderRoll_Left");i++;
    legMotor[i] = robot->getMotor("shoulderYaw_Left");i++;
    legMotor[i] = robot->getMotor("elbow_Left");i++;
    
    legMotor[i] = robot->getMotor("wristYaw_Left");i++;
    legMotor[i] = robot->getMotor("wristPitch_Left");i++;
    legMotor[i] = robot->getMotor("wristRoll_Left");i++;

    legMotor[i] = robot->getMotor("L_thumb_MCP_joint1");i++;
    legMotor[i] = robot->getMotor("L_thumb_MCP_joint2");i++;
    legMotor[i] = robot->getMotor("L_thumb_PIP_joint");i++;
    legMotor[i] = robot->getMotor("L_thumb_DIP_joint");i++;
    legMotor[i] = robot->getMotor("L_index_MCP_joint");i++;
    legMotor[i] = robot->getMotor("L_index_DIP_joint");i++;
    legMotor[i] = robot->getMotor("L_middle_MCP_joint");i++;
    legMotor[i] = robot->getMotor("L_middle_DIP_joint");i++;
    legMotor[i] = robot->getMotor("L_ring_MCP_joint");i++;
    legMotor[i] = robot->getMotor("L_ring_DIP_joint");i++;
    legMotor[i] = robot->getMotor("L_pinky_MCP_joint");i++;
    legMotor[i] = robot->getMotor("L_pinky_DIP_joint");i++;
  }

  // right arm and hand
  if(adam_type==ADAM_TYPE::AdamLite){
    legMotor[i] = robot->getMotor("shoulderPitch_Right");i++;
    legMotor[i] = robot->getMotor("shoulderRoll_Right");i++;
    legMotor[i] = robot->getMotor("shoulderYaw_Right");i++;
    legMotor[i] = robot->getMotor("elbow_Right");i++;
  }else if(adam_type==ADAM_TYPE::AdamLiteSimple){
  }else if(adam_type==ADAM_TYPE::AdamStandard){
    legMotor[i] = robot->getMotor("shoulderPitch_Right");i++;
    legMotor[i] = robot->getMotor("shoulderRoll_Right");i++;
    legMotor[i] = robot->getMotor("shoulderYaw_Right");i++;
    legMotor[i] = robot->getMotor("elbow_Right");i++;

    legMotor[i] = robot->getMotor("wristYaw_Right");i++;
    legMotor[i] = robot->getMotor("wristPitch_Right");i++;
    legMotor[i] = robot->getMotor("wristRoll_Right");i++;
    legMotor[i] = robot->getMotor("gripper_Right");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus23){
    legMotor[i] = robot->getMotor("shoulderPitch_Right");i++;
    legMotor[i] = robot->getMotor("shoulderRoll_Right");i++;
    legMotor[i] = robot->getMotor("shoulderYaw_Right");i++;
    legMotor[i] = robot->getMotor("elbow_Right");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus29){
    legMotor[i] = robot->getMotor("shoulderPitch_Right");i++;
    legMotor[i] = robot->getMotor("shoulderRoll_Right");i++;
    legMotor[i] = robot->getMotor("shoulderYaw_Right");i++;
    legMotor[i] = robot->getMotor("elbow_Right");i++;
    
    legMotor[i] = robot->getMotor("wristYaw_Right");i++;
    legMotor[i] = robot->getMotor("wristPitch_Right");i++;
    legMotor[i] = robot->getMotor("wristRoll_Right");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus53){
    legMotor[i] = robot->getMotor("shoulderPitch_Right");i++;
    legMotor[i] = robot->getMotor("shoulderRoll_Right");i++;
    legMotor[i] = robot->getMotor("shoulderYaw_Right");i++;
    legMotor[i] = robot->getMotor("elbow_Right");i++;
    
    legMotor[i] = robot->getMotor("wristYaw_Right");i++;
    legMotor[i] = robot->getMotor("wristPitch_Right");i++;
    legMotor[i] = robot->getMotor("wristRoll_Right");i++;

    legMotor[i] = robot->getMotor("R_thumb_MCP_joint1");i++;
    legMotor[i] = robot->getMotor("R_thumb_MCP_joint2");i++;
    legMotor[i] = robot->getMotor("R_thumb_PIP_joint");i++;
    legMotor[i] = robot->getMotor("R_thumb_DIP_joint");i++;
    legMotor[i] = robot->getMotor("R_index_MCP_joint");i++;
    legMotor[i] = robot->getMotor("R_index_DIP_joint");i++;
    legMotor[i] = robot->getMotor("R_middle_MCP_joint");i++;
    legMotor[i] = robot->getMotor("R_middle_DIP_joint");i++;
    legMotor[i] = robot->getMotor("R_ring_MCP_joint");i++;
    legMotor[i] = robot->getMotor("R_ring_DIP_joint");i++;
    legMotor[i] = robot->getMotor("R_pinky_MCP_joint");i++;
    legMotor[i] = robot->getMotor("R_pinky_DIP_joint");
  }

  // legMotor[29] = robot->getMotor("headYaw");
  // legMotor[30] = robot->getMotor("headRoll");
  // legMotor[31] = robot->getMotor("headPitch");

  // motor sensors
  legSensor.resize(nJoint);
  i = 0;
  legSensor[i] = robot->getPositionSensor("hipPitch_Left_sensor");i++;
  legSensor[i] = robot->getPositionSensor("hipRoll_Left_sensor");i++;
  legSensor[i] = robot->getPositionSensor("hipYaw_Left_sensor");i++;
  legSensor[i] = robot->getPositionSensor("kneePitch_Left_sensor");i++;
  legSensor[i] = robot->getPositionSensor("anklePitch_Left_sensor");i++;
  legSensor[i] = robot->getPositionSensor("ankleRoll_Left_sensor");i++;
  legSensor[i] = robot->getPositionSensor("hipPitch_Right_sensor");i++;
  legSensor[i] = robot->getPositionSensor("hipRoll_Right_sensor");i++;
  legSensor[i] = robot->getPositionSensor("hipYaw_Right_sensor");i++;
  legSensor[i] = robot->getPositionSensor("kneePitch_Right_sensor");i++;
  legSensor[i] = robot->getPositionSensor("anklePitch_Right_sensor");i++;
  legSensor[i] = robot->getPositionSensor("ankleRoll_Right_sensor");i++;

  //waist
  if(adam_type==ADAM_TYPE::AdamLite){
    legSensor[i] = robot->getPositionSensor("waistRoll_sensor");i++;
    legSensor[i] = robot->getPositionSensor("waistPitch_sensor");i++;
    legSensor[i] = robot->getPositionSensor("waistYaw_sensor");i++;
  }else if(adam_type==ADAM_TYPE::AdamLiteSimple){
  }else if(adam_type==ADAM_TYPE::AdamStandard){
    legSensor[i] = robot->getPositionSensor("waistRoll_sensor");i++;
    legSensor[i] = robot->getPositionSensor("waistPitch_sensor");i++;
    legSensor[i] = robot->getPositionSensor("waistYaw_sensor");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus23){
    legSensor[i] = robot->getPositionSensor("waistRoll_sensor");i++;
    legSensor[i] = robot->getPositionSensor("waistPitch_sensor");i++;
    legSensor[i] = robot->getPositionSensor("waistYaw_sensor");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus29){
    legSensor[i] = robot->getPositionSensor("waistRoll_sensor");i++;
    legSensor[i] = robot->getPositionSensor("waistPitch_sensor");i++;
    legSensor[i] = robot->getPositionSensor("waistYaw_sensor");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus53){
    legSensor[i] = robot->getPositionSensor("waistRoll_sensor");i++;
    legSensor[i] = robot->getPositionSensor("waistPitch_sensor");i++;
    legSensor[i] = robot->getPositionSensor("waistYaw_sensor");i++;
  }

  // left arm and hand
  if(adam_type==ADAM_TYPE::AdamLite){
    legSensor[i] = robot->getPositionSensor("shoulderPitch_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderRoll_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderYaw_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("elbow_Left_sensor");i++;
  }else if(adam_type==ADAM_TYPE::AdamLiteSimple){
  }else if(adam_type==ADAM_TYPE::AdamStandard){
    legSensor[i] = robot->getPositionSensor("shoulderPitch_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderRoll_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderYaw_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("elbow_Left_sensor");i++;

    legSensor[i] = robot->getPositionSensor("wristYaw_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("wristPitch_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("wristRoll_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("gripper_Left_sensor");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus23){
    legSensor[i] = robot->getPositionSensor("shoulderPitch_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderRoll_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderYaw_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("elbow_Left_sensor");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus29){
    legSensor[i] = robot->getPositionSensor("shoulderPitch_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderRoll_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderYaw_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("elbow_Left_sensor");i++;

    legSensor[i] = robot->getPositionSensor("wristYaw_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("wristPitch_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("wristRoll_Left_sensor");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus53){
    legSensor[i] = robot->getPositionSensor("shoulderPitch_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderRoll_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderYaw_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("elbow_Left_sensor");i++;

    legSensor[i] = robot->getPositionSensor("wristYaw_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("wristPitch_Left_sensor");i++;
    legSensor[i] = robot->getPositionSensor("wristRoll_Left_sensor");i++;

    legSensor[i] = robot->getPositionSensor("L_thumb_MCP_joint1_sensor");i++;
    legSensor[i] = robot->getPositionSensor("L_thumb_MCP_joint2_sensor");i++;
    legSensor[i] = robot->getPositionSensor("L_thumb_PIP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("L_thumb_DIP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("L_index_MCP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("L_index_DIP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("L_middle_MCP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("L_middle_DIP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("L_ring_MCP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("L_ring_DIP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("L_pinky_MCP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("L_pinky_DIP_joint_sensor");i++;
  }

  // right arm and hand
  if(adam_type==ADAM_TYPE::AdamLite){
    legSensor[i] = robot->getPositionSensor("shoulderPitch_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderRoll_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderYaw_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("elbow_Right_sensor");i++;
  }else if(adam_type==ADAM_TYPE::AdamLiteSimple){
  }else if(adam_type==ADAM_TYPE::AdamStandard){
    legSensor[i] = robot->getPositionSensor("shoulderPitch_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderRoll_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderYaw_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("elbow_Right_sensor");i++;

    legSensor[i] = robot->getPositionSensor("wristYaw_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("wristPitch_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("wristRoll_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("gripper_Right_sensor");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus23){
    legSensor[i] = robot->getPositionSensor("shoulderPitch_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderRoll_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderYaw_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("elbow_Right_sensor");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus29){
    legSensor[i] = robot->getPositionSensor("shoulderPitch_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderRoll_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderYaw_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("elbow_Right_sensor");i++;

    legSensor[i] = robot->getPositionSensor("wristYaw_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("wristPitch_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("wristRoll_Right_sensor");i++;
  }else if(adam_type==ADAM_TYPE::StandardPlus53){
    legSensor[i] = robot->getPositionSensor("shoulderPitch_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderRoll_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("shoulderYaw_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("elbow_Right_sensor");i++;

    legSensor[i] = robot->getPositionSensor("wristYaw_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("wristPitch_Right_sensor");i++;
    legSensor[i] = robot->getPositionSensor("wristRoll_Right_sensor");i++;

    legSensor[i] = robot->getPositionSensor("R_thumb_MCP_joint1_sensor");i++;
    legSensor[i] = robot->getPositionSensor("R_thumb_MCP_joint2_sensor");i++;
    legSensor[i] = robot->getPositionSensor("R_thumb_PIP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("R_thumb_DIP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("R_index_MCP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("R_index_DIP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("R_middle_MCP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("R_middle_DIP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("R_ring_MCP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("R_ring_DIP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("R_pinky_MCP_joint_sensor");i++;
    legSensor[i] = robot->getPositionSensor("R_pinky_DIP_joint_sensor");
  }
  
  // torque sensors
  // torqueSensor.resize(6);
  // torqueSensor[0] = robot->getMotor("Left_torqueX_sensor");
  // torqueSensor[1] = robot->getMotor("Left_torqueY_sensor");
  // torqueSensor[2] = robot->getMotor("Left_torqueZ_sensor");
  // torqueSensor[3] = robot->getMotor("Right_torqueX_sensor");
  // torqueSensor[4] = robot->getMotor("Right_torqueY_sensor");
  // torqueSensor[5] = robot->getMotor("Right_torqueZ_sensor");
  // touch sensors
  // forceSensor.resize(2);
  // forceSensor[0] = robot->getTouchSensor("Left_3Dforces_sensor");
  // forceSensor[1] = robot->getTouchSensor("Right_3Dforces_sensor");

  // other sensors
  imu = robot->getInertialUnit("inertial unit");
  gyro = robot->getGyro("gyro");
  accelerometer = robot->getAccelerometer("accelerometer");
  // Waistgps = robot->getGPS("gps_upperBody");
  // LFootGps = robot->getGPS("gps_LeftFoot");
  // RFootGps = robot->getGPS("gps_RightFoot");
  Waist = robot->getFromDef("Adam"); //all types are call Adam
  // SoleLeft = robot->getFromDef("Sole_Left");
  // SoleRight = robot->getFromDef("Sole_Right");
  // enable
  for (int i = 0; i < nJoint; i++) {
    legMotor[i]->enableTorqueFeedback(TIME_STEP);
  }
  for (int i = 0; i < nJoint; i++) {
    legSensor[i]->enable(TIME_STEP);
  }
  // for (int i = 0; i < 6; i++) {
  //     torqueSensor[i]->enableTorqueFeedback(TIME_STEP);
  // }
  // for (int i = 0; i < 2; i++) {
  //     forceSensor[i]->enable(TIME_STEP);
  // }
  imu->enable(TIME_STEP);
  gyro->enable(TIME_STEP);
  accelerometer->enable(TIME_STEP);
  // Waistgps->enable(TIME_STEP);
  // LFootGps->enable(TIME_STEP);
  // RFootGps->enable(TIME_STEP);
  // Derivative
  dRpy.resize(3);
  for (int i = 0; i < 3; i++) {
    dRpy.at(i).init(SAMPLE_TIME, 1e-3, 0.);
  }
  dJnt.resize(nJoint);
  for (int i = 0; i < nJoint; i++) {
    dJnt.at(i).init(SAMPLE_TIME, 1e-3, 0.);
  }
}

void WebotsRobot::deleteRobot() { delete robot; }

bool WebotsRobot::readData(double simTime, webotState &robotStateSim) {
  // Motor pos
  robotStateSim.jointPosAct = getMotorPos();

  // Motor vel
  if (simTime > SAMPLE_TIME - 1e-6 && simTime < SAMPLE_TIME + 1e-6) {
    for (int i = 0; i < nJoint; i++) {
      dJnt.at(i).init(SAMPLE_TIME, 1e-3, robotStateSim.jointPosAct(i));
    }
  }
  for (int i = 0; i < nJoint; i++) {
    robotStateSim.jointVelAct(i) =
        dJnt.at(i).mSig(robotStateSim.jointPosAct(i), SAMPLE_TIME);
  }

  // Motor torque
  robotStateSim.jointTorAct = getMotorTau();

  // IMU Data 9-dof
  const double *rotmArray = Waist->getOrientation();
  Eigen::Matrix3d rotm;
  rotm << rotmArray[0], rotmArray[1], rotmArray[2], rotmArray[3], rotmArray[4],
      rotmArray[5], rotmArray[6], rotmArray[7], rotmArray[8];
  // robotStateSim.waistRpyAct = rotm2Rpy(rotm);
  // if (simTime > SAMPLE_TIME - 1e-6 && simTime < SAMPLE_TIME + 1e-6){
  //     for (int i = 0; i < 3; i++) {
  //         dRpy.at(i).init(SAMPLE_TIME, 1e-3, robotStateSim.waistRpyAct(i));
  //     }
  // }
  // for (int i = 0; i < 3; i++) {
  //     robotStateSim.waistRpyVelAct(i) =
  //     dRpy.at(i).mSig(robotStateSim.waistRpyAct(i));
  // }
  Eigen::Vector3d rpy = rotm2Rpy(rotm);
  robotStateSim.waistRpyAct << rpy(2), rpy(1), rpy(0);

  const double *waistVel =
      Waist->getVelocity(); // the first three is linear velocity the second
                            // three is angular velocity
  Eigen::Vector3d angularRate;
  angularRate << waistVel[3], waistVel[4], waistVel[5];
  robotStateSim.waistRpyVelAct = rotm.transpose() * angularRate;

  robotStateSim.waistXyzAccAct = getWaistAcc();
  robotStateSim.imu9DAct << robotStateSim.waistRpyAct,
      robotStateSim.waistRpyVelAct, robotStateSim.waistXyzAccAct;

  // External Force
  // robotStateSim.footGrfAct = getFootForce12D();

  return true;
}

bool WebotsRobot::setMotorPos(const Eigen::VectorXd &jointPosTar) {
  for (int i = 0; i < nJoint; i++) {
    if (jointPosTar(i, 0) < 50000)
      legMotor[i]->setPosition(jointPosTar(i, 0));
  }
  return true;
}

bool WebotsRobot::setMotorTau(const Eigen::VectorXd &jointTauTar) {
  for (int i = 0; i < nJoint; i++) {
    if (jointTauTar(i, 0) < 50000)
      legMotor[i]->setTorque(jointTauTar(i, 0));
  }
  return true;
}

Eigen::VectorXd WebotsRobot::getMotorPos() {
  Eigen::VectorXd Q = Eigen::VectorXd::Zero(nJoint);
  for (int i = 0; i < nJoint; i++) {
    Q(i, 0) = legSensor[i]->getValue();
  }
  return Q;
}

Eigen::VectorXd WebotsRobot::getMotorTau() {
  Eigen::VectorXd Tau = Eigen::VectorXd::Zero(nJoint);
  for (int i = 0; i < nJoint; i++) {
    Tau(i, 0) = legMotor[i]->getTorqueFeedback();
  }
  return Tau;
}

Eigen::Vector3d WebotsRobot::getWaistAcc() {
  const double *data = accelerometer->getValues();
  Eigen::Vector3d acceleration(data[0], data[1], data[2]);
  return acceleration;
}

// Eigen::VectorXd WebotsRobot::getFootForce6D(const int& footFlag) {
//     const double* force;
//     double torqueX;
//     double torqueY;
//     double torqueZ;
//     switch (footFlag) {
//         case LEFTFOOT: {
//             force = forceSensor[0]->getValues();
//             torqueX = torqueSensor[0]->getTorqueFeedback();
//             torqueY = torqueSensor[1]->getTorqueFeedback();
//             torqueZ = torqueSensor[2]->getTorqueFeedback();
//             break;
//         }
//         case RIGHTFOOT: {
//             force = forceSensor[1]->getValues();
//             torqueX = torqueSensor[3]->getTorqueFeedback();
//             torqueY = torqueSensor[4]->getTorqueFeedback();
//             torqueZ = torqueSensor[5]->getTorqueFeedback();
//             break;
//         }
//         default: {
//             std::cout << " footFlag is wrong, return the values of LEFTFOOT
//             by default ! " << std::endl; force = forceSensor[0]->getValues();
//             torqueX = torqueSensor[0]->getTorqueFeedback();
//             torqueY = torqueSensor[1]->getTorqueFeedback();
//             torqueZ = torqueSensor[2]->getTorqueFeedback();
//             break;
//         }
//     }
//     Eigen::Vector3d forceVec = Eigen::Vector3d::Zero();
//     forceVec << force[0], force[1], force[2];
//     Eigen::Vector3d torqueVec = Eigen::Vector3d::Zero();
//     torqueVec << torqueX, torqueY, torqueZ;
//     Eigen::VectorXd footForce = Eigen::VectorXd::Zero(6);
//     footForce << forceVec, torqueVec;
//     return footForce;
// }

// Eigen::VectorXd WebotsRobot::getFootForce12D() {
//     Eigen::VectorXd LFootForce = getFootForce6D(LEFTFOOT);
//     Eigen::VectorXd RFootForce = getFootForce6D(RIGHTFOOT);
//     Eigen::VectorXd FootForce = Eigen::VectorXd::Zero(12);
//     FootForce << LFootForce,  RFootForce;
//     return FootForce;
// }

Eigen::Vector3d WebotsRobot::rotm2Rpy(const Eigen::Matrix3d &rotm) {
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  rpy(0) = atan2(rotm(2, 1), rotm(2, 2));
  rpy(1) = atan2(-rotm(2, 0),
                 sqrt(rotm(2, 1) * rotm(2, 1) + rotm(2, 2) * rotm(2, 2)));
  rpy(2) = atan2(rotm(1, 0), rotm(0, 0));
  return rpy;
}

Eigen::Vector3d WebotsRobot::rotm2xyz(const Eigen::Matrix3d &R) {
  Eigen::Vector3d euler = Eigen::Vector3d::Zero();

  euler.setZero();
  // y (-pi/2 pi/2)
  euler(1) = asin(R(0, 2));
  // z [-pi pi]
  double sinz = -R(0, 1) / cos(euler(1));
  double cosz = R(0, 0) / cos(euler(1));
  euler(2) = atan2(sinz, cosz);
  // x [-pi pi]
  double sinx = -R(1, 2) / cos(euler(1));
  double cosx = R(2, 2) / cos(euler(1));
  euler(0) = atan2(sinx, cosx);

  return euler;
}

Eigen::Matrix3d WebotsRobot::rotx(const double theta) {
  Eigen::Matrix3d matRes;
  matRes << 1., 0., 0., 0., std::cos(theta), -std::sin(theta), 0.,
      std::sin(theta), std::cos(theta);
  return matRes;
}

// **************************************************************************************************************//

Derivative::Derivative() {}

Derivative::Derivative(double dT, double c) {
  double alpha(2. / dT);
  this->a0 = c * alpha + 1.;
  this->a1 = (1. - c * alpha) / this->a0;
  this->b0 = alpha / this->a0;
  this->b1 = -alpha / this->a0;
  this->a0 = 1.;
  this->sigInPrev = 0.;
  this->sigOutPrev = 0.;
}

void Derivative::init(double dT, double c, double initValue) {
  double alpha(2. / dT);
  this->a0 = c * alpha + 1.;
  this->a1 = (1. - c * alpha) / this->a0;
  this->b0 = alpha / this->a0;
  this->b1 = -alpha / this->a0;
  this->a0 = 1.;
  this->sigInPrev = initValue;
  this->sigOutPrev = initValue;
}

double Derivative::mSig(double sigIn, double dT) {
  double sigOut(0.);
  if (sigIn - sigInPrev < -PI) {
    sigOut = (2. * PI + sigIn - sigInPrev) / dT;
  } else if (sigIn - sigInPrev > PI) {
    sigOut = (sigIn - 2. * PI - sigInPrev) / dT;
  } else {
    sigOut = (sigIn - sigInPrev) / dT;
  }
  // sigOut = b0 * sigIn + b1 * sigInPrev - a1 * sigOutPrev;

  sigOutPrev = sigOut;
  sigInPrev = sigIn;
  return sigOut;
}
