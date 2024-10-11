//
#include "functionIKID_S2P.h"
functionIKID_S2P::functionIKID_S2P() {
  leftAnkleALimb = 40.0;
  leftAnkleZLimb1 = 275.0;
  leftAnkleZLimb2 = 202.0;
  leftAnklePLimb1 = 280.0;
  leftAnklePLimb2 = 208.0;
  leftAnkleBaseDis = 20.5;
  rightAnkleALimb = 40.0;
  rightAnkleZLimb1 = 275.0;
  rightAnkleZLimb2 = 202.0;
  rightAnklePLimb1 = 280.0;
  rightAnklePLimb2 = 208.0;
  rightAnkleBaseDis = 20.5;
  oP1LeftFoot << 37.08, 20.5, -15.0;
  oP2LeftFoot << 37.084, -20.5, -15.0;
  oP1RightFoot << 37.08, -20.5, -15.0;
  oP2RightFoot << 37.08, 20.5, -15.0;

  rollLeftRef = 0.0;
  pitchLeftRef = 0.0;
  alpha1LeftRef = -0.5071;
  alpha2LeftRef = -0.5656;
  qLeftRef.setZero();
  qDotLeftRef.setZero();
  rollRightRef = 0.0;
  pitchRightRef = 0.0;
  alpha1RightRef = -0.5071;
  alpha2RightRef = -0.5656;
  qRightRef.setZero();
  qDotRightRef.setZero();

  qLeftEst.setZero();
  rollLeftEst = 0.0;
  pitchLeftEst = 0.0;
  alpha1LeftEst = -0.5071;
  alpha2LeftEst = -0.5656;
  qDotLeftEst.setZero();
  qTorLeftEst.setZero();
  omegaLeftEst.setZero();
  omegaLeftEstTwoOrder.setZero();
  omegaLeftEstAver.setZero();
  omegaLeftEstBeforeFilt.setZero();
  pitchLeftVelVec = Eigen::VectorXd::Zero(60);
  rollLeftVelVec = Eigen::VectorXd::Zero(60);
  RBaseLeftEst.setZero();
  baseOmegaLeftEst.setZero();
  baseAccLeftDes = Eigen::VectorXd::Zero(6);
  qRightEst.setZero();
  rollRightEst = 0.0;
  pitchRightEst = 0.0;
  alpha1RightEst = -0.5071;
  alpha2RightEst = -0.5656;
  qDotRightEst.setZero();
  qTorRightEst.setZero();
  torLeftEst.setZero();
  torRightEst.setZero();
  omegaRightEst.setZero();
  omegaRightEstTwoOrder.setZero();
  omegaRightEstAver.setZero();
  omegaRightEstBeforeFilt.setZero();
  pitchRightVelVec = Eigen::VectorXd::Zero(60);
  rollRightVelVec = Eigen::VectorXd::Zero(60);
  RBaseRightEst.setZero();
  baseOmegaRightEst.setZero();
  baseAccRightDes = Eigen::VectorXd::Zero(6);

  rotXLeft = Eigen::MatrixXd::Zero(3, 3);
  rotYLeft = Eigen::MatrixXd::Zero(3, 3);
  rotYXLeft = Eigen::MatrixXd::Zero(3, 3);
  rotXLeftRef = Eigen::MatrixXd::Zero(3, 3);
  rotYLeftRef = Eigen::MatrixXd::Zero(3, 3);
  rotYXLeftRef = Eigen::MatrixXd::Zero(3, 3);
  rotXRight = Eigen::MatrixXd::Zero(3, 3);
  rotYRight = Eigen::MatrixXd::Zero(3, 3);
  rotYXRight = Eigen::MatrixXd::Zero(3, 3);
  rotXRightRef = Eigen::MatrixXd::Zero(3, 3);
  rotYRightRef = Eigen::MatrixXd::Zero(3, 3);
  rotYXRightRef = Eigen::MatrixXd::Zero(3, 3);

  C1P1Left.setZero();
  C2P2Left.setZero();
  C1P1Right.setZero();
  C2P2Right.setZero();
  ROmegaLeft = Eigen::MatrixXd::Zero(3, 2);
  ROmegaLegLeft = Eigen::MatrixXd::Zero(3, 2);
  ROmegaRight = Eigen::MatrixXd::Zero(3, 2);
  ROmegaLegRight = Eigen::MatrixXd::Zero(3, 2);
  footComPosBodyLeft.setZero();
  legComPosBodyLeft.setZero();
  loadComPosBodyLeft.setZero();
  footComPosBodyRight.setZero();
  legComPosBodyRight.setZero();
  loadComPosBodyRight.setZero();

  JP1Left = Eigen::MatrixXd::Zero(3, 2);
  JP2Left = Eigen::MatrixXd::Zero(3, 2);
  JFootComLeft = Eigen::MatrixXd::Zero(3, 2);
  JLegComLeft = Eigen::MatrixXd::Zero(3, 2);
  JLoadComLeft = Eigen::MatrixXd::Zero(3, 2);
  B1C1Left.setZero();
  B2C2Left.setZero();
  JLeft = Eigen::MatrixXd::Zero(2, 2);
  JP1Right = Eigen::MatrixXd::Zero(3, 2);
  JP2Right = Eigen::MatrixXd::Zero(3, 2);
  JFootComRight = Eigen::MatrixXd::Zero(3, 2);
  JLegComRight = Eigen::MatrixXd::Zero(3, 2);
  JLoadComRight = Eigen::MatrixXd::Zero(3, 2);
  B1C1Right.setZero();
  B2C2Right.setZero();
  JRight = Eigen::MatrixXd::Zero(2, 2);

  epsilonLeftDes.setZero();
  ankleTauSLeftDes.setZero();
  tauLeftDes.setZero();
  tauLeftDesjointFB.setZero();
  JC1Left = Eigen::MatrixXd::Zero(3, 2);
  JC2Left = Eigen::MatrixXd::Zero(3, 2);
  vP1Left.setZero();
  vP2Left.setZero();
  JP1DotLeft = Eigen::MatrixXd::Zero(3, 2);
  JP2DotLeft = Eigen::MatrixXd::Zero(3, 2);
  JC1DotLeft = Eigen::MatrixXd::Zero(3, 2);
  JC2DotLeft = Eigen::MatrixXd::Zero(3, 2);
  JDotLeft = Eigen::MatrixXd::Zero(2, 2);
  ROmegaDotLeft = Eigen::MatrixXd::Zero(3, 2);
  ROmegaLegDotLeft = Eigen::MatrixXd::Zero(3, 2);
  epsilonRightDes.setZero();
  ankleTauSRightDes.setZero();
  tauRightDes.setZero();
  tauRightDesjointFB.setZero();
  JC1Right = Eigen::MatrixXd::Zero(3, 2);
  JC2Right = Eigen::MatrixXd::Zero(3, 2);
  vP1Right.setZero();
  vP2Right.setZero();
  JP1DotRight = Eigen::MatrixXd::Zero(3, 2);
  JP2DotRight = Eigen::MatrixXd::Zero(3, 2);
  JC1DotRight = Eigen::MatrixXd::Zero(3, 2);
  JC2DotRight = Eigen::MatrixXd::Zero(3, 2);
  JDotRight = Eigen::MatrixXd::Zero(2, 2);
  ROmegaDotRight = Eigen::MatrixXd::Zero(3, 2);
  ROmegaLegDotRight = Eigen::MatrixXd::Zero(3, 2);
  qRef.setZero();
  qEst.setZero();
  qDotRef.setZero();
  qDotEst.setZero();
}
functionIKID_S2P::~functionIKID_S2P() {}
bool functionIKID_S2P::setEst(const Eigen::VectorXd& qEst, const Eigen::VectorXd& qDotEst,
                              const Eigen::VectorXd& qTorEst) {
  alpha1LeftEst = qEst(0) - 0.5071;
  alpha2LeftEst = qEst(1) - 0.5656;
  alpha1RightEst = qEst(2) - 0.5071;
  alpha2RightEst = qEst(3) - 0.5656;

  qLeftEst = qEst.head(2);
  qRightEst = qEst.tail(2);
  qDotLeftEst = qDotEst.head(2);
  qDotRightEst = qDotEst.tail(2);
  qTorLeftEst = qTorEst.head(2);
  qTorRightEst = qTorEst.tail(2);
  return 1;
}

bool functionIKID_S2P::setDes(const Eigen::VectorXd& ankleOrienRef, const Eigen::VectorXd& ankleOmegaRef) {
  rollLeftRef = ankleOrienRef(1);
  pitchLeftRef = ankleOrienRef(0);
  rollRightRef = ankleOrienRef(3);
  pitchRightRef = ankleOrienRef(2);
  omegaLeftRef = ankleOmegaRef.head(2);
  omegaRightRef = ankleOmegaRef.tail(2);
  return 1;
}
bool functionIKID_S2P::setDesTorque(const Eigen::VectorXd& tauSDes) {
  ankleTauSLeftDes = tauSDes.head(2);
  ankleTauSRightDes = tauSDes.tail(2);
  return 1;
}
bool functionIKID_S2P::getDes(Eigen::VectorXd& qDes, Eigen::VectorXd& qDotDes, Eigen::VectorXd& tauDes) {
  qDes = qRef;
  qDotDes = qDotRef;
  tauDes = tauDesjointFB;
  return 1;
}

bool functionIKID_S2P::calcJointPosRef() {
  calcJointPosRefLeft();
  calcJointPosRefRight();
  return 1;
}
bool functionIKID_S2P::calcJointPosRefLeft() {
  rotXLeftRef << 1.0, 0.0, 0.0, 0.0, cos(rollLeftRef), -sin(rollLeftRef), 0, sin(rollLeftRef), cos(rollLeftRef);
  rotYLeftRef << cos(pitchLeftRef), 0.0, sin(pitchLeftRef), 0.0, 1.0, 0.0, -sin(pitchLeftRef), 0.0, cos(pitchLeftRef);
  oP1BodyLeftRef = rotYLeftRef * rotXLeftRef * oP1LeftFoot;
  oP2BodyLeftRef = rotYLeftRef * rotXLeftRef * oP2LeftFoot;
  double a1 = -2.0 * oP1BodyLeftRef(0) * leftAnkleALimb;
  double b1 = 2.0 * (oP1BodyLeftRef(2) - leftAnkleZLimb1) * leftAnkleALimb;
  double c1 = oP1BodyLeftRef(0) * oP1BodyLeftRef(0) +
              (oP1BodyLeftRef(1) - leftAnkleBaseDis) * (oP1BodyLeftRef(1) - leftAnkleBaseDis) +
              (oP1BodyLeftRef(2) - leftAnkleZLimb1) * (oP1BodyLeftRef(2) - leftAnkleZLimb1) +
              leftAnkleALimb * leftAnkleALimb - leftAnklePLimb1 * leftAnklePLimb1;

  double a2 = -2.0 * oP2BodyLeftRef(0) * leftAnkleALimb;
  double b2 = 2.0 * (oP2BodyLeftRef(2) - leftAnkleZLimb2) * leftAnkleALimb;
  double c2 = oP2BodyLeftRef(0) * oP2BodyLeftRef(0) +
              (oP2BodyLeftRef(1) + leftAnkleBaseDis) * (oP2BodyLeftRef(1) + leftAnkleBaseDis) +
              (oP2BodyLeftRef(2) - leftAnkleZLimb2) * (oP2BodyLeftRef(2) - leftAnkleZLimb2) +
              leftAnkleALimb * leftAnkleALimb - leftAnklePLimb2 * leftAnklePLimb2;

  if (a1 * a1 + b1 * b1 - c1 * c1 < 0.0) {
    std::cout << "ErrorID00 : Cannot solved !" << std::endl;
    return false;
  } else if (a2 * a2 + b2 * b2 - c2 * c2 < 0.0) {
    std::cout << "ErrorID01 : Cannot solved !" << std::endl;
    return false;
  } else {
    double csalpha11 = (a1 * c1 + b1 * sqrt(a1 * a1 + b1 * b1 - c1 * c1)) / (a1 * a1 + b1 * b1);
    double csalpha12 = (a1 * c1 - b1 * sqrt(a1 * a1 + b1 * b1 - c1 * c1)) / (a1 * a1 + b1 * b1);
    double csalpha21 = (a2 * c2 + b2 * sqrt(a2 * a2 + b2 * b2 - c2 * c2)) / (a2 * a2 + b2 * b2);
    double csalpha22 = (a2 * c2 - b2 * sqrt(a2 * a2 + b2 * b2 - c2 * c2)) / (a2 * a2 + b2 * b2);
    if (abs(csalpha11) > 1.0) {
      std::cout << "ErrorID02 : !!!!!!limb1 !" << std::endl;
      return false;
    } else if (abs(csalpha12) > 1.0) {
      std::cout << "ErrorID03 : !!!!!!limb1 !" << std::endl;
      return false;
    } else if (abs(csalpha21) > 1.0) {
      std::cout << "ErrorID04 : !!!!!!limb2 !" << std::endl;
      return false;
    } else if (abs(csalpha22) > 1.0) {
      std::cout << "ErrorID05 : !!!!!!limb2 !" << std::endl;
      return false;
    } else {
      double alpha11 = acos(csalpha11);
      double alpha12 = -acos(csalpha11);
      double alpha13 = acos(csalpha12);
      double alpha14 = -acos(csalpha12);
      Eigen::Vector4d alpha1_p = Eigen::Vector4d::Zero();
      alpha1_p << alpha11, alpha12, alpha13, alpha14;
      double alpha21 = acos(csalpha21);
      double alpha22 = -acos(csalpha21);
      double alpha23 = acos(csalpha22);
      double alpha24 = -acos(csalpha22);
      Eigen::Vector4d alpha2_p = Eigen::Vector4d::Zero();
      alpha2_p << alpha21, alpha22, alpha23, alpha24;
      for (int i = 0; i < 4; i++) {
        if (alpha1_p(i) >= -PI / 2 && alpha1_p(i) <= PI / 2) {
          if (abs(a1 * cos(alpha1_p(i)) + b1 * sin(alpha1_p(i)) - c1) <= 0.0000001) {
            alpha1LeftRef = alpha1_p(i);
          }
        }
      }
      for (int i = 0; i < 4; i++) {
        if (alpha2_p(i) >= -PI / 2 && alpha2_p(i) <= PI / 2) {
          if (abs(a2 * cos(alpha2_p(i)) + b2 * sin(alpha2_p(i)) - c2) <= 0.0000001) {
            alpha2LeftRef = alpha2_p(i);
          }
        }
      }
      qLeftRef << alpha1LeftRef + 0.5071, alpha2LeftRef + 0.5656;

      return true;
    }
  }

  return 1;
}
bool functionIKID_S2P::calcJointPosRefRight() {
  rotXRightRef << 1.0, 0.0, 0.0, 0.0, cos(rollRightRef), -sin(rollRightRef), 0, sin(rollRightRef), cos(rollRightRef);
  rotYRightRef << cos(pitchRightRef), 0.0, sin(pitchRightRef), 0.0, 1.0, 0.0, -sin(pitchRightRef), 0.0,
      cos(pitchRightRef);
  oP1BodyRightRef = rotYRightRef * rotXRightRef * oP1RightFoot;
  oP2BodyRightRef = rotYRightRef * rotXRightRef * oP2RightFoot;
  double a1 = -2.0 * oP1BodyRightRef(0) * rightAnkleALimb;
  double b1 = 2.0 * (oP1BodyRightRef(2) - rightAnkleZLimb1) * rightAnkleALimb;
  double c1 = oP1BodyRightRef(0) * oP1BodyRightRef(0) +
              (oP1BodyRightRef(1) + rightAnkleBaseDis) * (oP1BodyRightRef(1) + rightAnkleBaseDis) +
              (oP1BodyRightRef(2) - rightAnkleZLimb1) * (oP1BodyRightRef(2) - rightAnkleZLimb1) +
              rightAnkleALimb * rightAnkleALimb - rightAnklePLimb1 * rightAnklePLimb1;

  double a2 = -2.0 * oP2BodyRightRef(0) * rightAnkleALimb;
  double b2 = 2.0 * (oP2BodyRightRef(2) - rightAnkleZLimb2) * rightAnkleALimb;
  double c2 = oP2BodyRightRef(0) * oP2BodyRightRef(0) +
              (oP2BodyRightRef(1) - rightAnkleBaseDis) * (oP2BodyRightRef(1) - rightAnkleBaseDis) +
              (oP2BodyRightRef(2) - rightAnkleZLimb2) * (oP2BodyRightRef(2) - rightAnkleZLimb2) +
              rightAnkleALimb * rightAnkleALimb - rightAnklePLimb2 * rightAnklePLimb2;

  if (a1 * a1 + b1 * b1 - c1 * c1 < 0.0) {
    std::cout << "ErrorID00 : Cannot solved !" << std::endl;
    return false;
  } else if (a2 * a2 + b2 * b2 - c2 * c2 < 0.0) {
    std::cout << "ErrorID01 : Cannot solved !" << std::endl;
    return false;
  } else {
    double csalpha11 = (a1 * c1 + b1 * sqrt(a1 * a1 + b1 * b1 - c1 * c1)) / (a1 * a1 + b1 * b1);
    double csalpha12 = (a1 * c1 - b1 * sqrt(a1 * a1 + b1 * b1 - c1 * c1)) / (a1 * a1 + b1 * b1);
    double csalpha21 = (a2 * c2 + b2 * sqrt(a2 * a2 + b2 * b2 - c2 * c2)) / (a2 * a2 + b2 * b2);
    double csalpha22 = (a2 * c2 - b2 * sqrt(a2 * a2 + b2 * b2 - c2 * c2)) / (a2 * a2 + b2 * b2);
    if (abs(csalpha11) > 1.0) {
      std::cout << "ErrorID02 : !!!!!!limb1 !" << std::endl;
      return false;
    } else if (abs(csalpha12) > 1.0) {
      std::cout << "ErrorID03 : !!!!!!limb1 !" << std::endl;
      return false;
    } else if (abs(csalpha21) > 1.0) {
      std::cout << "ErrorID04 : !!!!!!limb2 !" << std::endl;
      return false;
    } else if (abs(csalpha22) > 1.0) {
      std::cout << "ErrorID05 : !!!!!!limb2 !" << std::endl;
      return false;
    } else {
      double alpha11 = acos(csalpha11);
      double alpha12 = -acos(csalpha11);
      double alpha13 = acos(csalpha12);
      double alpha14 = -acos(csalpha12);
      Eigen::Vector4d alpha1_p = Eigen::Vector4d::Zero();
      alpha1_p << alpha11, alpha12, alpha13, alpha14;
      double alpha21 = acos(csalpha21);
      double alpha22 = -acos(csalpha21);
      double alpha23 = acos(csalpha22);
      double alpha24 = -acos(csalpha22);
      Eigen::Vector4d alpha2_p = Eigen::Vector4d::Zero();
      alpha2_p << alpha21, alpha22, alpha23, alpha24;
      for (int i = 0; i < 4; i++) {
        if (alpha1_p(i) >= -PI / 2 && alpha1_p(i) <= PI / 2) {
          if (abs(a1 * cos(alpha1_p(i)) + b1 * sin(alpha1_p(i)) - c1) <= 0.0000001) {
            alpha1RightRef = alpha1_p(i);
          }
        }
      }
      for (int i = 0; i < 4; i++) {
        if (alpha2_p(i) >= -PI / 2 && alpha2_p(i) <= PI / 2) {
          if (abs(a2 * cos(alpha2_p(i)) + b2 * sin(alpha2_p(i)) - c2) <= 0.0000001) {
            alpha2RightRef = alpha2_p(i);
          }
        }
      }
      qRightRef << alpha1RightRef + 0.5071, alpha2RightRef + 0.5656;

      return true;
    }
  }

  return 1;
}
bool functionIKID_S2P::calcFK() {
  calcAnkleEstLeft();
  calcAnkleEstRight();
  return 1;
}

bool functionIKID_S2P::calcAnkleEstLeft() {
  static const float b_a[50]{3.59178543F,  0.775940239F, 1.24691415F,  -5.07508802F,  2.40616512F,   -0.604540408F,
                             -5.0010376F,  1.61057806F,  -3.09902048F, 1.94651365F,   -0.211533889F, -0.129851863F,
                             2.49764037F,  0.129169494F, -1.95933187F, -0.201830804F, -4.24206257F,  3.38318372F,
                             0.62031883F,  5.58123398F,  -4.23221731F, 6.32177687F,   1.35723114F,   -3.84512877F,
                             -2.89399F,    -5.53990841F, 4.66575718F,  5.66879654F,   4.68981028F,   -1.29733133F,
                             -3.07611585F, -1.73929477F, -3.75354934F, 3.33007216F,   -1.3362695F,   -1.24159634F,
                             -6.16725826F, -1.67880929F, 6.17626429F,  1.27029467F,   -1.35327601F,  3.944767F,
                             -2.71123481F, 2.58899856F,  -3.3408823F,  5.64190865F,   -2.57107902F,  3.83001328F,
                             -1.91180086F, -5.4636426F};
  static const float c_a[50]{
      -0.571474791F,  -0.306457788F,   0.00565536972F, -0.0678443089F, 0.0100264493F,  0.0219776984F,   -0.624789774F,
      0.166214675F,   -0.35164389F,    0.00229097856F, -0.026138939F,  0.0251090359F,  0.000309568713F, 0.000186517354F,
      0.00285909977F, 0.000529525511F, 0.201675698F,   -0.0258218329F, -0.663394928F,  -0.0903202072F,  -0.301209271F,
      0.43761003F,    -0.337631851F,   0.537240088F,   -0.171232209F,  -0.0424098745F, -0.336149663F,   0.534753382F,
      0.740822732F,   0.246673137F,    -0.394871175F,  0.603340268F,   -0.19152014F,   -0.0679072589F,  -0.97987324F,
      -0.367859572F,  0.0732798278F,   -0.137250915F,  0.044957146F,   0.0476667322F,  0.0107350014F,   0.0179120507F,
      0.0508709699F,  -0.00389487064F, 0.048437871F,   -0.0956749767F, 0.0588298F,     -0.0262468122F,  -0.0575751F,
      0.0991663188F};
  static const float fv[25]{-7.08717394F,  -5.17271185F, -6.17403316F,  5.37523031F,   -2.1499157F,
                            2.72721028F,   3.00402284F,  -1.33928549F,  2.60191917F,   -0.79004544F,
                            0.628515601F,  0.228955984F, -0.112595975F, -0.229212746F, -0.714729667F,
                            -0.659370422F, -2.54766536F, 2.55712104F,   2.23827672F,   4.14827299F,
                            -4.30017614F,  5.56251049F,  4.31973553F,   -5.77351F,     -7.7286644F};
  static const float fv1[2]{0.200858429F, 0.00381205324F};
  float a[25];
  float xp1_idx_0;
  float xp1_idx_1;
  //  ===== NEURAL NETWORK CONSTANTS =====
  //  Input 1
  //  Layer 1
  //  Layer 2
  //  Output 1
  //  ===== SIMULATION ========
  //  Dimensions
  //  samples
  //  Input 1
  //  ===== MODULE FUNCTIONS ========
  //  Map Minimum and Maximum Input Processing Function
  xp1_idx_0 = (float(alpha1LeftEst) - -1.04082251F) * 1.05235863F - 1.0F;
  xp1_idx_1 = (float(alpha2LeftEst) - -1.05159509F) * 1.06206155F - 1.0F;
  //  Layer 1
  //  Sigmoid Symmetric Transfer Function
  //  Layer 2
  for (int k{0}; k < 25; k++) {
    a[k] = 2.0F / (std::exp(-2.0F * ((b_a[k] * xp1_idx_0 + b_a[k + 25] * xp1_idx_1) + fv[k])) + 1.0F) - 1.0F;
  }
  //  Output 1
  //  Map Minimum and Maximum Output Reverse-Processing Function
  float b_y1[2];
  for (int k{0}; k < 2; k++) {
    xp1_idx_0 = 0.0F;
    for (int i{0}; i < 25; i++) {
      xp1_idx_0 += c_a[k + (i << 1)] * a[i];
    }
    b_y1[k] = ((xp1_idx_0 + fv1[k]) - -1.0F) / (-0.943695307F * static_cast<float>(k) + 2.29183125F) +
              (-0.610865235F * static_cast<float>(k) - 0.436332315F);
  }

  rollLeftEst = double(-b_y1[0]);
  pitchLeftEst = double(b_y1[1]);
  return 1;
}
bool functionIKID_S2P::calcAnkleEstRight() {
  static const float b_a[50]{3.59178543F,  0.775940239F, 1.24691415F,  -5.07508802F,  2.40616512F,   -0.604540408F,
                             -5.0010376F,  1.61057806F,  -3.09902048F, 1.94651365F,   -0.211533889F, -0.129851863F,
                             2.49764037F,  0.129169494F, -1.95933187F, -0.201830804F, -4.24206257F,  3.38318372F,
                             0.62031883F,  5.58123398F,  -4.23221731F, 6.32177687F,   1.35723114F,   -3.84512877F,
                             -2.89399F,    -5.53990841F, 4.66575718F,  5.66879654F,   4.68981028F,   -1.29733133F,
                             -3.07611585F, -1.73929477F, -3.75354934F, 3.33007216F,   -1.3362695F,   -1.24159634F,
                             -6.16725826F, -1.67880929F, 6.17626429F,  1.27029467F,   -1.35327601F,  3.944767F,
                             -2.71123481F, 2.58899856F,  -3.3408823F,  5.64190865F,   -2.57107902F,  3.83001328F,
                             -1.91180086F, -5.4636426F};
  static const float c_a[50]{
      -0.571474791F,  -0.306457788F,   0.00565536972F, -0.0678443089F, 0.0100264493F,  0.0219776984F,   -0.624789774F,
      0.166214675F,   -0.35164389F,    0.00229097856F, -0.026138939F,  0.0251090359F,  0.000309568713F, 0.000186517354F,
      0.00285909977F, 0.000529525511F, 0.201675698F,   -0.0258218329F, -0.663394928F,  -0.0903202072F,  -0.301209271F,
      0.43761003F,    -0.337631851F,   0.537240088F,   -0.171232209F,  -0.0424098745F, -0.336149663F,   0.534753382F,
      0.740822732F,   0.246673137F,    -0.394871175F,  0.603340268F,   -0.19152014F,   -0.0679072589F,  -0.97987324F,
      -0.367859572F,  0.0732798278F,   -0.137250915F,  0.044957146F,   0.0476667322F,  0.0107350014F,   0.0179120507F,
      0.0508709699F,  -0.00389487064F, 0.048437871F,   -0.0956749767F, 0.0588298F,     -0.0262468122F,  -0.0575751F,
      0.0991663188F};
  static const float fv[25]{-7.08717394F,  -5.17271185F, -6.17403316F,  5.37523031F,   -2.1499157F,
                            2.72721028F,   3.00402284F,  -1.33928549F,  2.60191917F,   -0.79004544F,
                            0.628515601F,  0.228955984F, -0.112595975F, -0.229212746F, -0.714729667F,
                            -0.659370422F, -2.54766536F, 2.55712104F,   2.23827672F,   4.14827299F,
                            -4.30017614F,  5.56251049F,  4.31973553F,   -5.77351F,     -7.7286644F};
  static const float fv1[2]{0.200858429F, 0.00381205324F};
  float a[25];
  float xp1_idx_0;
  float xp1_idx_1;
  //  ===== NEURAL NETWORK CONSTANTS =====
  //  Input 1
  //  Layer 1
  //  Layer 2
  //  Output 1
  //  ===== SIMULATION ========
  //  Dimensions
  //  samples
  //  Input 1
  //  ===== MODULE FUNCTIONS ========
  //  Map Minimum and Maximum Input Processing Function
  xp1_idx_0 = (float(alpha1RightEst) - -1.04082251F) * 1.05235863F - 1.0F;
  xp1_idx_1 = (float(alpha2RightEst) - -1.05159509F) * 1.06206155F - 1.0F;
  //  Layer 1
  //  Sigmoid Symmetric Transfer Function
  //  Layer 2
  for (int k{0}; k < 25; k++) {
    a[k] = 2.0F / (std::exp(-2.0F * ((b_a[k] * xp1_idx_0 + b_a[k + 25] * xp1_idx_1) + fv[k])) + 1.0F) - 1.0F;
  }
  //  Output 1
  //  Map Minimum and Maximum Output Reverse-Processing Function
  float b_y1[2];
  for (int k{0}; k < 2; k++) {
    xp1_idx_0 = 0.0F;
    for (int i{0}; i < 25; i++) {
      xp1_idx_0 += c_a[k + (i << 1)] * a[i];
    }
    b_y1[k] = ((xp1_idx_0 + fv1[k]) - -1.0F) / (-0.943695307F * static_cast<float>(k) + 2.29183125F) +
              (-0.610865235F * static_cast<float>(k) - 0.436332315F);
  }

  rollRightEst = double(b_y1[0]);
  pitchRightEst = double(b_y1[1]);
  return 1;
}
bool functionIKID_S2P::calcIK() {
  calcJLeft();
  calcJRight();
  calcJDotLeft();
  calcJDotRight();
  return 1;
}
bool functionIKID_S2P::calcJLeft() {
  rotXLeft << 1.0, 0.0, 0.0, 0.0, cos(rollLeftEst), -sin(rollLeftEst), 0, sin(rollLeftEst), cos(rollLeftEst);
  rotYLeft << cos(pitchLeftEst), 0.0, sin(pitchLeftEst), 0.0, 1.0, 0.0, -sin(pitchLeftEst), 0.0, cos(pitchLeftEst);

  rotYXLeft = rotYLeft * rotXLeft;
  oP1BodyLeft = rotYXLeft * oP1LeftFoot;
  oP2BodyLeft = rotYXLeft * oP2LeftFoot;
  // footComPosBodyLeft = rotYXLeft*footComPos;

  C1P1Left << oP1BodyLeft(0) + leftAnkleALimb * cos(alpha1LeftEst), oP1BodyLeft(1) - leftAnkleBaseDis,
      oP1BodyLeft(2) - leftAnkleZLimb1 - leftAnkleALimb * sin(alpha1LeftEst);
  C2P2Left << oP2BodyLeft(0) + leftAnkleALimb * cos(alpha2LeftEst), oP2BodyLeft(1) + leftAnkleBaseDis,
      oP2BodyLeft(2) - leftAnkleZLimb2 - leftAnkleALimb * sin(alpha2LeftEst);

  ROmegaLeft << 0.0, rotYXLeft(0, 0), 1.0, rotYXLeft(1, 0), 0.0, rotYXLeft(2, 0);

  JP1Left = -Skew(oP1BodyLeft) * ROmegaLeft;
  JP2Left = -Skew(oP2BodyLeft) * ROmegaLeft;
  // JFootComLeft = -Skew(footComPosBodyLeft) * ROmegaLeft;

  B1C1Left << -leftAnkleALimb * cos(alpha1LeftEst), 0, leftAnkleALimb * sin(alpha1LeftEst),
      B2C2Left << -leftAnkleALimb * cos(alpha2LeftEst), 0, leftAnkleALimb * sin(alpha2LeftEst);

  JLeft << C1P1Left.transpose() * JP1Left / (B1C1Left(2) * C1P1Left(0) - B1C1Left(0) * C1P1Left(2)),
      C2P2Left.transpose() * JP2Left / (B2C2Left(2) * C2P2Left(0) - B2C2Left(0) * C2P2Left(2));
  JC1Left << B1C1Left(2) * JLeft.block(0, 0, 1, 2), 0.0, 0.0, -B1C1Left(0) * JLeft.block(0, 0, 1, 2);
  JC2Left << B2C2Left(2) * JLeft.block(1, 0, 1, 2), 0.0, 0.0, -B2C2Left(0) * JLeft.block(1, 0, 1, 2);
  // std::cout<<"JLeft:"<<std::endl<<JLeft<<std::endl;

  omegaLeftEstBeforeFilt = JLeft.completeOrthogonalDecomposition().pseudoInverse() * qDotLeftEst;
  omegaLeftEst = omegaLeftEstBeforeFilt;
  torLeftEst = JLeft.transpose() * qTorLeftEst;
  //     for (int i = 0; i < 2; i++)
  //         {
  //           omegaLeftEst[i] = filter(omegaLeftEst[i],
  //           &omegaLeftEstFilter[i],8);
  //         }
  return 1;
}

bool functionIKID_S2P::calcJDotLeft() {
  vP1Left = JP1Left * omegaLeftEst;
  vP2Left = JP2Left * omegaLeftEst;
  JP1DotLeft = -Skew(vP1Left) * ROmegaLeft;
  JP2DotLeft = -Skew(vP2Left) * ROmegaLeft;

  JDotLeft << (C1P1Left.transpose() *
                   (JP1DotLeft + B1C1Left * JLeft.block(0, 0, 1, 2) * omegaLeftEst * JLeft.block(0, 0, 1, 2)) +
               omegaLeftEst.transpose() * (JP1Left - JC1Left).transpose() * (JP1Left - JC1Left)) /
                  (B1C1Left(2) * C1P1Left(0) - B1C1Left(0) * C1P1Left(2)),
      (C2P2Left.transpose() *
           (JP2DotLeft + B2C2Left * JLeft.block(1, 0, 1, 2) * omegaLeftEst * JLeft.block(1, 0, 1, 2)) +
       omegaLeftEst.transpose() * (JP2Left - JC2Left).transpose() * (JP2Left - JC2Left)) /
          (B2C2Left(2) * C2P2Left(0) - B2C2Left(0) * C2P2Left(2));

  Eigen::MatrixXd JC1Dot_ = Eigen::MatrixXd::Zero(3, 2);
  Eigen::MatrixXd JC2Dot_ = Eigen::MatrixXd::Zero(3, 2);
  JC1Dot_ << B1C1Left(2) * JDotLeft.block(0, 0, 1, 2), 0.0, 0.0, -B1C1Left(0) * JDotLeft.block(0, 0, 1, 2);
  JC2Dot_ << B2C2Left(2) * JDotLeft.block(1, 0, 1, 2), 0.0, 0.0, -B2C2Left(0) * JDotLeft.block(1, 0, 1, 2);

  JC1DotLeft = -qDotLeftEst(0) * B1C1Left * JLeft.block(0, 0, 1, 2) + JC1Dot_;
  JC2DotLeft = -qDotLeftEst(1) * B2C2Left * JLeft.block(1, 0, 1, 2) + JC2Dot_;

  ROmegaDotLeft << 0.0, -sin(pitchLeftEst) * omegaLeftEst(0), 0.0, 0.0, 0.0, -cos(pitchLeftEst) * omegaLeftEst(0);
  return 1;
}
bool functionIKID_S2P::calcJRight() {
  rotXRight << 1.0, 0.0, 0.0, 0.0, cos(rollRightEst), -sin(rollRightEst), 0, sin(rollRightEst), cos(rollRightEst);
  rotYRight << cos(pitchRightEst), 0.0, sin(pitchRightEst), 0.0, 1.0, 0.0, -sin(pitchRightEst), 0.0, cos(pitchRightEst);

  rotYXRight = rotYRight * rotXRight;
  oP1BodyRight = rotYXRight * oP1RightFoot;
  oP2BodyRight = rotYXRight * oP2RightFoot;
  footComPosBodyRight = rotYXRight * footComPos;

  C1P1Right << oP1BodyRight(0) + rightAnkleALimb * cos(alpha1RightEst), oP1BodyRight(1) + rightAnkleBaseDis,
      oP1BodyRight(2) - rightAnkleZLimb1 - rightAnkleALimb * sin(alpha1RightEst);
  C2P2Right << oP2BodyRight(0) + rightAnkleALimb * cos(alpha2RightEst), oP2BodyRight(1) - rightAnkleBaseDis,
      oP2BodyRight(2) - rightAnkleZLimb2 - rightAnkleALimb * sin(alpha2RightEst);

  ROmegaRight << 0.0, rotYXRight(0, 0), 1.0, rotYXRight(1, 0), 0.0, rotYXRight(2, 0);

  JP1Right = -Skew(oP1BodyRight) * ROmegaRight;
  JP2Right = -Skew(oP2BodyRight) * ROmegaRight;
  JFootComRight = -Skew(footComPosBodyRight) * ROmegaRight;

  B1C1Right << -rightAnkleALimb * cos(alpha1RightEst), 0, rightAnkleALimb * sin(alpha1RightEst),
      B2C2Right << -rightAnkleALimb * cos(alpha2RightEst), 0, rightAnkleALimb * sin(alpha2RightEst);
  JRight << C1P1Right.transpose() * JP1Right / (B1C1Right(2) * C1P1Right(0) - B1C1Right(0) * C1P1Right(2)),
      C2P2Right.transpose() * JP2Right / (B2C2Right(2) * C2P2Right(0) - B2C2Right(0) * C2P2Right(2));

  omegaRightEstBeforeFilt = JRight.completeOrthogonalDecomposition().pseudoInverse() * qDotRightEst;
  omegaRightEst = omegaRightEstBeforeFilt;
  //     for (int i = 0; i < 2; i++)
  //         {
  //           omegaLeftEst[i] = filter(omegaLeftEst[i],
  //           &omegaLeftEstFilter[i],8);
  //         }
  JC1Right << B1C1Right(2) * JRight.block(0, 0, 1, 2), 0.0, 0.0, -B1C1Right(0) * JRight.block(0, 0, 1, 2);
  JC2Right << B2C2Right(2) * JRight.block(1, 0, 1, 2), 0.0, 0.0, -B2C2Right(0) * JRight.block(1, 0, 1, 2);
  torRightEst = JRight.transpose() * qTorRightEst;
  return 1;
}
bool functionIKID_S2P::calcJDotRight() {
  vP1Right = JP1Right * omegaRightEst;
  vP2Right = JP2Right * omegaRightEst;
  JP1DotRight = -Skew(vP1Right) * ROmegaRight;
  JP2DotRight = -Skew(vP2Right) * ROmegaRight;
  // std::cout<<"JP1DotLeft:"<<std::endl<<JP1DotLeft<<std::endl;
  // std::cout<<"JP2DotLeft:"<<std::endl<<JP2DotLeft<<std::endl;
  JDotRight << (C1P1Right.transpose() *
                    (JP1DotRight + B1C1Right * JRight.block(0, 0, 1, 2) * omegaRightEst * JRight.block(0, 0, 1, 2)) +
                omegaRightEst.transpose() * (JP1Right - JC1Right).transpose() * (JP1Right - JC1Right)) /
                   (B1C1Right(2) * C1P1Right(0) - B1C1Right(0) * C1P1Right(2)),
      (C2P2Right.transpose() *
           (JP2DotRight + B2C2Right * JRight.block(1, 0, 1, 2) * omegaRightEst * JRight.block(1, 0, 1, 2)) +
       omegaRightEst.transpose() * (JP2Right - JC2Right).transpose() * (JP2Right - JC2Right)) /
          (B2C2Right(2) * C2P2Right(0) - B2C2Right(0) * C2P2Right(2));
  // std::cout<<"JDotLeft:"<<std::endl<<JDotLeft<<std::endl;

  Eigen::MatrixXd JC1Dot_ = Eigen::MatrixXd::Zero(3, 2);
  Eigen::MatrixXd JC2Dot_ = Eigen::MatrixXd::Zero(3, 2);
  JC1Dot_ << B1C1Right(2) * JDotRight.block(0, 0, 1, 2), 0.0, 0.0, -B1C1Right(0) * JDotRight.block(0, 0, 1, 2);
  JC2Dot_ << B2C2Right(2) * JDotRight.block(1, 0, 1, 2), 0.0, 0.0, -B2C2Right(0) * JDotRight.block(1, 0, 1, 2);

  JC1DotRight = -qDotRightEst(0) * B1C1Right * JRight.block(0, 0, 1, 2) + JC1Dot_;
  JC2DotRight = -qDotRightEst(1) * B2C2Right * JRight.block(1, 0, 1, 2) + JC2Dot_;

  ROmegaDotRight << 0.0, -sin(pitchRightEst) * omegaRightEst(0), 0.0, 0.0, 0.0, -cos(pitchRightEst) * omegaRightEst(0);
  return 1;
}
bool functionIKID_S2P::getAnkleState(Eigen::VectorXd& ankleOrienEst, Eigen::VectorXd& ankleOmegaEst,
                                     Eigen::VectorXd& ankleTorEst) {
  ankleOrienEst << pitchLeftEst, rollLeftEst, pitchRightEst, rollRightEst;
  ankleOmegaEst << omegaLeftEst, omegaRightEst;
  ankleTorEst << torLeftEst, torRightEst;
  return 1;
}

bool functionIKID_S2P::calcJointTorDes() {
  tauLeftDes = JLeft.completeOrthogonalDecomposition().pseudoInverse().transpose() * (ankleTauSLeftDes);
  tauRightDes = JRight.completeOrthogonalDecomposition().pseudoInverse().transpose() * (ankleTauSRightDes);
  tauDes << tauLeftDes, tauRightDes;
  // Eigen::Vector4d Kp;
  // Kp << 30.0,25.0, 25.0, 30.0;
  // //    Kp << 10.0,10.0;
  // Eigen::Vector4d Kd;
  // Kd<<0.7,0.7,0.7,0.7;
  // //    Kd << 0.3,0.3;
  qRef << qLeftRef, qRightRef;
  qEst << qLeftEst, qRightEst;
  qDotLeftRef = JLeft * omegaLeftRef;
  qDotRightRef = JRight * omegaRightRef;
  qDotRef << qDotLeftRef, qDotRightRef;
  qDotEst << qDotLeftEst, qDotRightEst;
  tauDesjointFB = tauDes;  // + Kp.asDiagonal()*(qRef-qEst) +
                           // Kd.asDiagonal()*(qDotRef-qDotEst);

  double torLimit = 55.0;
  if (tauDesjointFB(0) >= torLimit) {
    tauDesjointFB(0) = torLimit;
    std::cout << "Torque1 beyond upper limit!!!" << std::endl;
  }
  if (tauDesjointFB(0) <= -torLimit) {
    tauDesjointFB(0) = -torLimit;
    std::cout << "Torque1 beyond lower limit!!!" << std::endl;
  }
  if (tauDesjointFB(1) >= torLimit) {
    tauDesjointFB(1) = torLimit;
    std::cout << "Torque2 beyond upper limit!!!" << std::endl;
  }
  if (tauDesjointFB(1) <= -torLimit) {
    tauDesjointFB(1) = -torLimit;
    std::cout << "Torque2 beyond lower limit!!!" << std::endl;
  }
  if (tauDesjointFB(2) >= torLimit) {
    tauDesjointFB(2) = torLimit;
    std::cout << "Torque3 beyond upper limit!!!" << std::endl;
  }
  if (tauDesjointFB(2) <= -torLimit) {
    tauDesjointFB(2) = -torLimit;
    std::cout << "Torque3 beyond lower limit!!!" << std::endl;
  }
  if (tauDesjointFB(3) >= torLimit) {
    tauDesjointFB(3) = torLimit;
    std::cout << "Torque4 beyond upper limit!!!" << std::endl;
  }
  if (tauDesjointFB(3) <= -torLimit) {
    tauDesjointFB(3) = -torLimit;
    std::cout << "Torque4 beyond lower limit!!!" << std::endl;
  }
  return 1;
}
// bool functionIKID_S2P::calcJointTorDesPKMDynLeft(){

//    Eigen::MatrixXd Mq = Eigen::MatrixXd::Zero(2,2);
//    Eigen::MatrixXd Cq = Eigen::MatrixXd::Zero(2,2);
//    Eigen::VectorXd Gq = Eigen::VectorXd::Zero(2);
//    Eigen::Vector3d Gravity;
//    Gravity << 0.0, 0.0, 9800.0;
//    Mq = JLeft.transpose()*ILeftAnkleALimb*JLeft +
//    (mLeftAnklePLimb1/3.0*(JC1Left.transpose()*JC1Left+JP1Left.transpose()*JP1Left)+mLeftAnklePLimb2/3.0*(JC2Left.transpose()*JC2Left+JP2Left.transpose()*JP2Left)+mLeftAnklePLimb1/6.0*(JC1Left.transpose()*JP1Left+JP1Left.transpose()*JC1Left)+mLeftAnklePLimb2/6.0*(JC2Left.transpose()*JP2Left+JP2Left.transpose()*JC2Left));
//    // std::cout<<"Mq:"<<std::endl<<Mq<<std::endl;
//    Cq = JLeft.transpose()*ILeftAnkleALimb*JDotLeft +
//    (mLeftAnklePLimb1/3.0*(JC1Left.transpose()*JC1DotLeft+JP1Left.transpose()*JP1DotLeft)+mLeftAnklePLimb2/3.0*(JC2Left.transpose()*JC2DotLeft+JP2Left.transpose()*JP2DotLeft)+mLeftAnklePLimb1/6.0*(JP1Left.transpose()*JC1DotLeft+JC1Left.transpose()*JP1DotLeft)+mLeftAnklePLimb2/6.0*(JP2Left.transpose()*JC2DotLeft+JC2Left.transpose()*JP2DotLeft));
//    // std::cout<<"Cq:"<<std::endl<<Cq<<std::endl;
//    Gq =
//    (mLeftAnklePLimb1/2.0*JP1Left.transpose()+mLeftAnklePLimb2/2.0*JP2Left.transpose())*RBaseLeftEst.transpose()*(Gravity+1000.0*baseAccLeftDes.segment(3,3))+(mLeftAnklePLimb1/2.0*JC1Left.transpose()+mLeftAnklePLimb2/2.0*JC2Left.transpose()+mLeftAnkleALimb*leftAnkleALimbCom/leftAnkleALimb*(JC1Left.transpose()+JC2Left.transpose()))*RBaseLeftEst.transpose()*(Gravity+1000.0*baseAccLeftDes.segment(3,3));
//    // std::cout<<"Gq:"<<std::endl<<Gq<<std::endl;

//    Eigen::MatrixXd IarmcomLegeftAnkleALimb = Eigen::MatrixXd::Zero(3,3);
//    IarmcomLegeftAnkleALimb << 8.0,2.0,-9.0,
//                2.0,43.0,0.0,
//                -9.0,0.0,36.0;
//    Eigen::MatrixXd R_yq1 = Eigen::MatrixXd::Zero(3,3);
//    R_yq1 << cos(alpha1LeftEst-0.29378),0.0,sin(alpha1LeftEst-0.29378),
//             0.0,1.0,0.0,
//             -sin(alpha1LeftEst-0.29378),0.0,cos(alpha1LeftEst-0.29378);
//    Eigen::MatrixXd Iarmact1 = Eigen::MatrixXd::Zero(3,3);
//    Iarmact1 = R_yq1 * IarmcomLegeftAnkleALimb * R_yq1.transpose();
//    Eigen::Vector3d BcmLegeftAnkleALimb;
//    BcmLegeftAnkleALimb << 21.239,-10.5070,-6.4;
//    Eigen::Vector3d OB1;
//    OB1 << 0.0,leftAnkleBaseDis,leftAnklePLimb1;
//    Eigen::Vector3d OBcmLegeftAnkleALimb_foot;
//    OBcmLegeftAnkleALimb_foot = R_yq1 * BcmLegeftAnkleALimb + OB1;
//    Eigen::MatrixXd Iarmact1_foot_ = Eigen::MatrixXd::Zero(3,3);
//    Iarmact1_foot_ <<
//    mLeftAnkleALimb*(OBcmLegeftAnkleALimb_foot(1)*OBcmLegeftAnkleALimb_foot(1)
//    +
//    OBcmLegeftAnkleALimb_foot(2)*OBcmLegeftAnkleALimb_foot(2)),mLeftAnkleALimb*OBcmLegeftAnkleALimb_foot(0)*OBcmLegeftAnkleALimb_foot(1),mLeftAnkleALimb*OBcmLegeftAnkleALimb_foot(0)*OBcmLegeftAnkleALimb_foot(2),
//                      mLeftAnkleALimb*OBcmLegeftAnkleALimb_foot(0)*OBcmLegeftAnkleALimb_foot(1),mLeftAnkleALimb*(OBcmLegeftAnkleALimb_foot(0)*OBcmLegeftAnkleALimb_foot(0)+OBcmLegeftAnkleALimb_foot(2)*OBcmLegeftAnkleALimb_foot(2)),mLeftAnkleALimb*OBcmLegeftAnkleALimb_foot(1)*OBcmLegeftAnkleALimb_foot(2),
//                      mLeftAnkleALimb*OBcmLegeftAnkleALimb_foot(0)*OBcmLegeftAnkleALimb_foot(2),mLeftAnkleALimb*OBcmLegeftAnkleALimb_foot(1)*OBcmLegeftAnkleALimb_foot(2),mLeftAnkleALimb*(OBcmLegeftAnkleALimb_foot(0)*OBcmLegeftAnkleALimb_foot(0)+OBcmLegeftAnkleALimb_foot(1)*OBcmLegeftAnkleALimb_foot(1));
//    Eigen::MatrixXd Iarmact1_foot = Eigen::MatrixXd::Zero(3,3);
//    Iarmact1_foot = Iarmact1 + Iarmact1_foot_;
//    // std::cout<<"Iarmact1_foot:"<<std::endl<<Iarmact1_foot<<std::endl;

//    Eigen::MatrixXd Iarmcom2 = Eigen::MatrixXd::Zero(3,3);
//    Iarmcom2 << 8.0,-2.0,-9.0,
//                -2.0,43.0,0.0,
//                -9.0,0.0,36.0;
//    Eigen::MatrixXd R_yq2 = Eigen::MatrixXd::Zero(3,3);
//    R_yq2 << cos(alpha2LeftEst-0.29378),0.0,sin(alpha2LeftEst-0.29378),
//             0.0,1.0,0.0,
//             -sin(alpha2LeftEst-0.29378),0.0,cos(alpha2LeftEst-0.29378);
//    Eigen::MatrixXd Iarmact2 = Eigen::MatrixXd::Zero(3,3);
//    Iarmact2=R_yq2*Iarmcom2*R_yq2.transpose();
//    Eigen::Vector3d Bcm2;
//    Bcm2 << 21.25,10.5070,-6.4;
//    Eigen::Vector3d OB2;
//    OB2 << 0.0,-leftAnkleBaseDis,leftAnklePLimb2;
//    Eigen::Vector3d OBcm2_foot;
//    OBcm2_foot = R_yq2 * Bcm2 + OB2;
//    Eigen::MatrixXd Iarmact2_foot_ = Eigen::MatrixXd::Zero(3,3);
//    Iarmact2_foot_ <<
//    mLeftAnkleALimb*(OBcm2_foot(1)*OBcm2_foot(1)+OBcm2_foot(2)*OBcm2_foot(2)),mLeftAnkleALimb*OBcm2_foot(0)*OBcm2_foot(1),mLeftAnkleALimb*OBcm2_foot(0)*OBcm2_foot(2),
//                      mLeftAnkleALimb*OBcm2_foot(0)*OBcm2_foot(1),mLeftAnkleALimb*(OBcm2_foot(0)*OBcm2_foot(0)+OBcm2_foot(2)*OBcm2_foot(2)),mLeftAnkleALimb*OBcm2_foot(1)*OBcm2_foot(2),
//                      mLeftAnkleALimb*OBcm2_foot(0)*OBcm2_foot(2),mLeftAnkleALimb*OBcm2_foot(1)*OBcm2_foot(2),mLeftAnkleALimb*(OBcm2_foot(0)*OBcm2_foot(0)+OBcm2_foot(1)*OBcm2_foot(1));
//    Eigen::MatrixXd Iarmact2_foot = Eigen::MatrixXd::Zero(3,3);
//    Iarmact2_foot = Iarmact2 + Iarmact2_foot_;
//    // std::cout<<"Iarmact2_foot:"<<std::endl<<Iarmact2_foot<<std::endl;

//    Eigen::MatrixXd IlimbcomLegeftAnkleALimb = Eigen::MatrixXd::Zero(3,3);
//    IlimbcomLegeftAnkleALimb << 1138.0,0.0,-2.0,
//                 0.0,1138.0,-1.0,
//                 -2.0,-1.0,2.0;
//    Eigen::MatrixXd Ilimbact1 = Eigen::MatrixXd::Zero(3,3);
//    Ilimbact1 =
//    Eigen::Matrix3d::Identity()*IlimbcomLegeftAnkleALimb*Eigen::Matrix3d::Identity();
//    Eigen::Vector3d OC1;
//    OC1 <<
//    B1C1Left(0)+0.0,B1C1Left(1)+leftAnkleBaseDis,B1C1Left(2)+leftAnklePLimb1;
//    Eigen::Vector3d CPcmLegeftAnkleALimb;
//    CPcmLegeftAnkleALimb = 0.5*OC1+0.5*oP1BodyLeft;
//    Eigen::MatrixXd Ilimbact1_foot_ = Eigen::MatrixXd::Zero(3,3);
//    Ilimbact1_foot_ <<
//    mLeftAnklePLimb1*(CPcmLegeftAnkleALimb(1)*CPcmLegeftAnkleALimb(1)+CPcmLegeftAnkleALimb(2)*CPcmLegeftAnkleALimb(2)),mLeftAnklePLimb1*CPcmLegeftAnkleALimb(0)*CPcmLegeftAnkleALimb(1),mLeftAnklePLimb1*CPcmLegeftAnkleALimb(0)*CPcmLegeftAnkleALimb(2),
//                       mLeftAnklePLimb1*CPcmLegeftAnkleALimb(0)*CPcmLegeftAnkleALimb(1),mLeftAnklePLimb1*(CPcmLegeftAnkleALimb(0)*CPcmLegeftAnkleALimb(0)+CPcmLegeftAnkleALimb(2)*CPcmLegeftAnkleALimb(2)),mLeftAnklePLimb1*CPcmLegeftAnkleALimb(1)*CPcmLegeftAnkleALimb(2),
//                       mLeftAnklePLimb1*CPcmLegeftAnkleALimb(0)*CPcmLegeftAnkleALimb(2),mLeftAnklePLimb1*CPcmLegeftAnkleALimb(1)*CPcmLegeftAnkleALimb(2),mLeftAnklePLimb1*(CPcmLegeftAnkleALimb(0)*CPcmLegeftAnkleALimb(0)+CPcmLegeftAnkleALimb(1)*CPcmLegeftAnkleALimb(1));
//    Eigen::MatrixXd Ilimbact1_foot = Eigen::MatrixXd::Zero(3,3);
//    Ilimbact1_foot = Ilimbact1 + Ilimbact1_foot_;
//    // std::cout<<"Ilimbact1_foot:"<<std::endl<<Ilimbact1_foot<<std::endl;

//    Eigen::MatrixXd Ilimbcom2 = Eigen::MatrixXd::Zero(3,3);
//    Ilimbcom2 << 372.0,0.0,-1.0,
//                 0.0,372.0,-1.0,
//                 -1.0,-1.0,2.0;
//    Eigen::MatrixXd Ilimbact2 = Eigen::MatrixXd::Zero(3,3);
//    Ilimbact2 =
//    Eigen::Matrix3d::Identity()*Ilimbcom2*Eigen::Matrix3d::Identity();
//    Eigen::Vector3d OC2;
//    OC2 <<
//    B2C2Left(0)+0.0,B2C2Left(1)-leftAnkleBaseDis,B2C2Left(2)+leftAnklePLimb2;
//    Eigen::Vector3d CPcm2;
//    CPcm2 = 0.5*OC2+0.5*oP2BodyLeft;
//    Eigen::MatrixXd Ilimbact2_foot_ = Eigen::MatrixXd::Zero(3,3);
//    Ilimbact2_foot_<<
//    mLeftAnklePLimb2*(CPcm2(1)*CPcm2(1)+CPcm2(2)*CPcm2(2)),mLeftAnklePLimb2*CPcm2(0)*CPcm2(1),mLeftAnklePLimb2*CPcm2(0)*CPcm2(2),
//                      mLeftAnklePLimb2*CPcm2(0)*CPcm2(1),mLeftAnklePLimb2*(CPcm2(0)*CPcm2(0)+CPcm2(2)*CPcm2(2)),mLeftAnklePLimb2*CPcm2(1)*CPcm2(2),
//                      mLeftAnklePLimb2*CPcm2(0)*CPcm2(2),mLeftAnklePLimb2*CPcm2(1)*CPcm2(2),mLeftAnklePLimb2*(CPcm2(0)*CPcm2(0)+CPcm2(1)*CPcm2(1));
//    Eigen::MatrixXd Ilimbact2_foot = Eigen::MatrixXd::Zero(3,3);
//    Ilimbact2_foot = Ilimbact2 + Ilimbact2_foot_;
//    // std::cout<<"Ilimbact2_foot:"<<std::endl<<Ilimbact2_foot<<std::endl;

//    Eigen::MatrixXd IPM_foot = Eigen::MatrixXd::Zero(3,3);
//    IPM_foot=Ilimbact1_foot+Ilimbact2_foot+Iarmact1_foot+Iarmact2_foot;
//    // std::cout<<"IPM_foot:"<<std::endl<<IPM_foot<<std::endl;
//    Eigen::Vector3d w_foot;
//    w_foot = RBaseLeftEst.transpose()*baseOmegaLeftEst;
//    Eigen::Vector2d T_PM;
//    T_PM=ROmegaLeft.transpose()*IPM_foot*RBaseLeftEst.transpose()*baseAccLeftDes.segment(0,3)
//    +
//    ROmegaLeft.transpose()*Skew(w_foot)*IPM_foot*RBaseLeftEst.transpose()*baseOmegaLeftEst;
//    tauLeftDes =
//    JLeft.completeOrthogonalDecomposition().pseudoInverse().transpose()*(Mq*epsilonLeftDes+Cq*omegaLeftEst+Gq+ankleTauSLeftDes*1000000.0+1.0*T_PM);
// // tauLeftDes =
// JLeft.completeOrthogonalDecomposition().pseudoInverse().transpose()*(ankleTauSLeftDes*1000000.0);

//     Eigen::Vector2d Kp;
//     Kp<<30.0,25.0;
// //    Kp << 10.0,10.0;
//     Eigen::Vector2d Kd;
//     Kd<<0.7,0.7;
// //    Kd << 0.3,0.3;
//     qDotLeftRef = JLeft * omegaLeftRef;

//     tauLeftDesjointFB = tauLeftDes/1000000.0 +
//     Kp.asDiagonal()*(qLeftRef-qLeftEst) +
//     Kd.asDiagonal()*(qDotLeftRef-qDotLeftEst);
// //    tauLeftDesjointFB =  Kp.asDiagonal()*(qLeftRef-qLeftEst) +
// Kd.asDiagonal()*(qDotLeftRef-qDotLeftEst);

//     double torLimit = 18.0;
//     if (tauLeftDesjointFB(0)>=torLimit)
//     {
//         tauLeftDesjointFB(0)=torLimit;
//         std::cout<<"Torque1 beyond upper limit!!!"<<std::endl;
//     }
//     if (tauLeftDesjointFB(0)<=-torLimit)
//     {
//         tauLeftDesjointFB(0)=-torLimit;
//         std::cout<<"Torque1 beyond lower limit!!!"<<std::endl;
//     }
//     if (tauLeftDesjointFB(1)>=torLimit)
//     {
//         tauLeftDesjointFB(1)=torLimit;
//         std::cout<<"Torque2 beyond upper limit!!!"<<std::endl;
//     }
//     if (tauLeftDesjointFB(1)<=-torLimit)
//     {
//         tauLeftDesjointFB(1)=-torLimit;
//         std::cout<<"Torque2 beyond lower limit!!!"<<std::endl;
//     }
// }

Eigen::Matrix3d functionIKID_S2P::Skew(const Eigen::Vector3d& omg) {
  Eigen::Matrix3d m_ret;
  m_ret << 0.0, -omg(2), omg(1), omg(2), 0.0, -omg(0), -omg(1), omg(0), 0.0;
  return m_ret;
}
