//
// Created by Gang on 2021-11-04//
//
#ifndef S2P_H_
#define S2P_H_
#ifndef PI
#define PI 3.141592654
#endif

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <sstream>
#include <string>
#include <time.h>
#include <vector>

#include <Eigen/Dense>
// #include <rbdl/rbdl.h>

class functionIKID_S2P {
public:
  functionIKID_S2P();
  ~functionIKID_S2P();

  bool calcJointPosRef();
  bool calcJointPosRefLeft();
  bool calcJointPosRefRight();
  bool calcIK();
  bool calcJLeft();
  bool calcJRight();
  bool calcJDotLeft();
  bool calcJDotRight();
  bool calcFK();
  bool calcAnkleEstLeft();
  bool calcAnkleEstRight();
  bool calcJointTorDes();
  bool calcJointTorDesPKMDynLeft();
  bool setEst(const Eigen::VectorXd &qEst, const Eigen::VectorXd &qDotEst,
              const Eigen::VectorXd &qTorEst);

  bool setDes(const Eigen::VectorXd &ankleOrienRef,
              const Eigen::VectorXd &ankleOmegaRef);
  bool setDesTorque(const Eigen::VectorXd &tauSDes);
  bool getAnkleState(Eigen::VectorXd &ankleOrienEst,
                     Eigen::VectorXd &ankleOmegaEst,
                     Eigen::VectorXd &ankleTorEst);
  bool getDes(Eigen::VectorXd &qDes, Eigen::VectorXd &qDotDes,
              Eigen::VectorXd &tauDes);

  Eigen::Matrix3d Skew(const Eigen::Vector3d &omg);

  double leftAnkleALimb; // Length of
  double leftAnkleZLimb1;
  double leftAnkleZLimb2;
  double leftAnklePLimb1;
  double leftAnklePLimb2;
  double leftAnkleBaseDis;
  double rightAnkleALimb; // Length of
  double rightAnkleZLimb1;
  double rightAnkleZLimb2;
  double rightAnklePLimb1;
  double rightAnklePLimb2;
  double rightAnkleBaseDis;

  double IRightAnkleALimb;
  double mRightAnkleALimb;
  double rightAnkleALimbCom;
  double mRightAnklePLimb1;
  double mRightAnklePLimb2;
  double ILeftAnkleALimb;
  double mLeftAnkleALimb;
  double leftAnkleALimbCom;
  double mLeftAnklePLimb1;
  double mLeftAnklePLimb2;

  Eigen::Matrix3d IFoot;
  double mFoot;
  Eigen::Vector3d footComPos;

  Eigen::Matrix3d ILeg;
  double mLeg;
  Eigen::Vector3d legComPos;

  Eigen::Matrix3d ILoad;
  double mLoad;
  Eigen::Vector3d loadComPos;

  Eigen::MatrixXd rotXLeft;
  Eigen::MatrixXd rotYLeft;
  Eigen::MatrixXd rotYXLeft;
  Eigen::MatrixXd rotXLeftRef;
  Eigen::MatrixXd rotYLeftRef;
  Eigen::MatrixXd rotYXLeftRef;
  Eigen::MatrixXd rotXRight;
  Eigen::MatrixXd rotYRight;
  Eigen::MatrixXd rotYXRight;
  Eigen::MatrixXd rotXRightRef;
  Eigen::MatrixXd rotYRightRef;
  Eigen::MatrixXd rotYXRightRef;

  Eigen::Vector3d oP1LeftFoot;
  Eigen::Vector3d oP2LeftFoot;
  Eigen::Vector3d oP1RightFoot;
  Eigen::Vector3d oP2RightFoot;
  Eigen::Vector3d oP1BodyLeft;
  Eigen::Vector3d oP2BodyLeft;
  Eigen::Vector3d oP1BodyLeftRef;
  Eigen::Vector3d oP2BodyLeftRef;
  Eigen::Vector3d footComPosBodyLeft;
  Eigen::Vector3d legComPosBodyLeft;
  Eigen::Vector3d loadComPosBodyLeft;
  Eigen::Vector3d oP1BodyRight;
  Eigen::Vector3d oP2BodyRight;
  Eigen::Vector3d oP1BodyRightRef;
  Eigen::Vector3d oP2BodyRightRef;
  Eigen::Vector3d footComPosBodyRight;
  Eigen::Vector3d legComPosBodyRight;
  Eigen::Vector3d loadComPosBodyRight;

  double rollLeftRef;
  double pitchLeftRef;
  double alpha1LeftRef;
  double alpha2LeftRef;
  Eigen::Vector2d qLeftRef;
  Eigen::Vector2d qDotLeftRef;
  Eigen::Vector2d omegaLeftRef;
  double rollRightRef;
  double pitchRightRef;
  double alpha1RightRef;
  double alpha2RightRef;
  Eigen::Vector2d qRightRef;
  Eigen::Vector2d qDotRightRef;
  Eigen::Vector2d omegaRightRef;
  Eigen::Vector4d qRef;
  Eigen::Vector4d qDotRef;
  double rollLeftEst;
  double pitchLeftEst;
  double alpha1LeftEst;
  double alpha2LeftEst;
  Eigen::Vector2d qLeftEst;
  Eigen::Vector2d qDotLeftEst;
  Eigen::Vector2d qTorLeftEst;
  Eigen::Vector2d omegaLeftEst;
  Eigen::Vector2d omegaLeftEstTwoOrder;
  Eigen::Vector2d omegaLeftEstAver;
  Eigen::Vector2d omegaLeftEstBeforeFilt;
  Eigen::Vector2d torLeftEst;
  Eigen::VectorXd pitchLeftVelVec;
  Eigen::VectorXd rollLeftVelVec;
  Eigen::Matrix3d RBaseLeftEst;
  Eigen::Vector3d baseOmegaLeftEst;
  Eigen::VectorXd baseAccLeftDes;
  double rollRightEst;
  double pitchRightEst;
  double alpha1RightEst;
  double alpha2RightEst;
  Eigen::Vector2d qRightEst;
  Eigen::Vector2d qDotRightEst;
  Eigen::Vector2d qTorRightEst;
  Eigen::Vector2d omegaRightEst;
  Eigen::Vector2d omegaRightEstTwoOrder;
  Eigen::Vector2d omegaRightEstAver;
  Eigen::Vector2d omegaRightEstBeforeFilt;
  Eigen::Vector2d torRightEst;
  Eigen::VectorXd pitchRightVelVec;
  Eigen::VectorXd rollRightVelVec;
  Eigen::Matrix3d RBaseRightEst;
  Eigen::Vector3d baseOmegaRightEst;
  Eigen::VectorXd baseAccRightDes;
  Eigen::Vector4d qEst;
  Eigen::Vector4d qDotEst;

  Eigen::Vector3d C1P1Left;
  Eigen::Vector3d C2P2Left;
  Eigen::Vector3d C1P1Right;
  Eigen::Vector3d C2P2Right;

  Eigen::MatrixXd ROmegaLeft;
  Eigen::MatrixXd ROmegaLegLeft;
  Eigen::MatrixXd JP1Left;
  Eigen::MatrixXd JP2Left;
  Eigen::MatrixXd JFootComLeft;
  Eigen::MatrixXd JLegComLeft;
  Eigen::MatrixXd JLoadComLeft;
  Eigen::MatrixXd ROmegaRight;
  Eigen::MatrixXd ROmegaLegRight;
  Eigen::MatrixXd JP1Right;
  Eigen::MatrixXd JP2Right;
  Eigen::MatrixXd JFootComRight;
  Eigen::MatrixXd JLegComRight;
  Eigen::MatrixXd JLoadComRight;

  Eigen::Vector3d B1C1Left;
  Eigen::Vector3d B2C2Left;
  Eigen::MatrixXd JLeft;
  Eigen::MatrixXd JC1Left;
  Eigen::MatrixXd JC2Left;
  Eigen::Vector3d vP1Left;
  Eigen::Vector3d vP2Left;
  Eigen::MatrixXd JP1DotLeft;
  Eigen::MatrixXd JP2DotLeft;
  Eigen::MatrixXd JC1DotLeft;
  Eigen::MatrixXd JC2DotLeft;
  Eigen::MatrixXd ROmegaDotLeft;
  Eigen::MatrixXd ROmegaLegDotLeft;
  Eigen::MatrixXd JDotLeft;
  Eigen::Vector2d epsilonLeftDes;
  Eigen::Vector2d ankleTauSLeftDes;
  Eigen::Vector2d tauLeftDes;
  Eigen::Vector2d tauLeftDesjointFB;
  Eigen::Vector3d B1C1Right;
  Eigen::Vector3d B2C2Right;
  Eigen::MatrixXd JRight;
  Eigen::MatrixXd JC1Right;
  Eigen::MatrixXd JC2Right;
  Eigen::Vector3d vP1Right;
  Eigen::Vector3d vP2Right;
  Eigen::MatrixXd JP1DotRight;
  Eigen::MatrixXd JP2DotRight;
  Eigen::MatrixXd JC1DotRight;
  Eigen::MatrixXd JC2DotRight;
  Eigen::MatrixXd ROmegaDotRight;
  Eigen::MatrixXd ROmegaLegDotRight;
  Eigen::MatrixXd JDotRight;
  Eigen::Vector2d epsilonRightDes;
  Eigen::Vector2d ankleTauSRightDes;
  Eigen::Vector2d tauRightDes;
  Eigen::Vector2d tauRightDesjointFB;
  Eigen::Vector4d tauDes;
  Eigen::Vector4d tauDesjointFB;
};
#endif
