#include "abs_encoder.h"
#include <cmath>
// #include <iostream>
#include <limits>
#include <vector>

PosSensor::PosSensor(double max_range) : max_range_(max_range) {}

double PosSensor::read(double value) { return fmod(value, max_range_); }

DoubleEncoderSystem::DoubleEncoderSystem(double encoder1_gear_ratio,
                                         double encoder2_gear_ratio,
                                         double min_end_detect_pos,
                                         double detect_accuracy,
                                         double encoder1_noise,
                                         double encoder2_noise)
    : encoder1_gear_ratio_(encoder1_gear_ratio),
      encoder2_gear_ratio_(encoder2_gear_ratio),
      min_end_detect_pos_(min_end_detect_pos),
      detect_accuracy_(detect_accuracy), encoder1_noise_(encoder1_noise),
      encoder2_noise_(encoder2_noise) {
  enc1_ptr_ = std::make_unique<PosSensor>(enc1_max_range_);
  enc2_ptr_ = std::make_unique<PosSensor>(enc2_max_range_);

  // get max detect round
  max_dectect_round_ = detect_accuracy_;

  while (fabs(fmod(encoder1_gear_ratio_ * max_dectect_round_, 1)) >
             detect_accuracy_ * encoder1_gear_ratio_ / 360.0 ||
         fabs(fmod(encoder2_gear_ratio_ * max_dectect_round_, 1)) >
             detect_accuracy_ * encoder2_gear_ratio_ / 360.0)
    max_dectect_round_ += detect_accuracy_ / 360.0;
  max_end_detect_pos_ = min_end_detect_pos_ + max_dectect_round_ * 360.0;

  min_end_detect_pos_from_encoder1_ =
      min_end_detect_pos_ + encoder1_noise_ / encoder1_gear_ratio_;
  max_end_detect_pos_from_encoder1_ =
      max_end_detect_pos_ - encoder1_noise_ / encoder1_gear_ratio_;
  min_end_detect_pos_from_encoder2_ =
      min_end_detect_pos_ + encoder2_noise_ / encoder2_gear_ratio_;
  max_end_detect_pos_from_encoder2_ =
      max_end_detect_pos_ - encoder2_noise_ / encoder2_gear_ratio_;

  min_end_detect_pos_ = std::max(min_end_detect_pos_from_encoder1_,
                                 min_end_detect_pos_from_encoder2_);
  max_end_detect_pos_ = std::min(max_end_detect_pos_from_encoder1_,
                                 max_end_detect_pos_from_encoder2_);
}

void DoubleEncoderSystem::SetEncoder1Offset(double offset) {
  encoder1_offset_ = offset;
}

void DoubleEncoderSystem::SetEncoder2Offset(double offset) {
  encoder2_offset_ = offset;
}

void DoubleEncoderSystem::SetEncoder1Dir(int dir) { encoder1_dir_ = dir; }
void DoubleEncoderSystem::SetEncoder2Dir(int dir) { encoder2_dir_ = dir; }

double DoubleEncoderSystem::GetMaxDetectRound() { return max_dectect_round_; }
double DoubleEncoderSystem::GetMinEndDetectPos() { return min_end_detect_pos_; }
double DoubleEncoderSystem::GetMaxEndDetectPos() { return max_end_detect_pos_; }

std::vector<double> DoubleEncoderSystem::GetAllowedEncoderNoise() {
  std::vector<double> allowed_encoder_noise(2);
  double encoder1_read_per_encoder2_cycle =
      enc1_ptr_->read(1 / encoder2_gear_ratio_ * encoder1_gear_ratio_ * 360.0);
  double encoder2_read_per_encoder1_cycle =
      enc2_ptr_->read(1 / encoder1_gear_ratio_ * encoder2_gear_ratio_ * 360.0);

  double encoder1_read_per_end_pos_accuracy =
      enc1_ptr_->read(detect_accuracy_ * encoder1_gear_ratio_);

  double encoder2_read_per_end_pos_accuracy =
      enc2_ptr_->read(detect_accuracy_ * encoder2_gear_ratio_);

  allowed_encoder_noise[0] = std::min(encoder1_read_per_encoder2_cycle,
                                      encoder1_read_per_end_pos_accuracy);
  allowed_encoder_noise[1] = std::min(encoder2_read_per_encoder1_cycle,
                                      encoder2_read_per_end_pos_accuracy);
  return allowed_encoder_noise;
}

double DoubleEncoderSystem::GetEndPos(double encoder1_value,
                                      double encoder2_value) {
  // check validation
  if (encoder1_gear_ratio_ <= 0 || encoder2_gear_ratio_ <= 0)
    return std::numeric_limits<double>::quiet_NaN();

  vec_possible_end_pos_from_encoder1_.clear();
  vec_possible_end_pos_from_encoder2_.clear();

  double end_pos = std::numeric_limits<double>::quiet_NaN();

  double end_pos_per_encoder1_read = encoder1_dir_ / encoder1_gear_ratio_;
  double end_pos_per_encoder2_read = encoder2_dir_ / encoder2_gear_ratio_;
  double possible_end_pos_from_encoder1 =
      (encoder1_value - encoder1_offset_) * end_pos_per_encoder1_read;
  double possible_end_pos_from_encoder2 =
      (encoder2_value - encoder2_offset_) * end_pos_per_encoder2_read;

  // std::cout << "possible_end_pos_from_encoder1: "
  //           << possible_end_pos_from_encoder1 << std::endl;
  // std::cout << "possible_end_pos_from_encoder2: "
  //           << possible_end_pos_from_encoder2 << std::endl;

  // get possible end positions from encoder1
  for (double end_pos_from_encoder1 =
           possible_end_pos_from_encoder1 - 360.0 / encoder1_gear_ratio_;
       end_pos_from_encoder1 >= min_end_detect_pos_;
       end_pos_from_encoder1 -= 360.0 / encoder1_gear_ratio_) {
    vec_possible_end_pos_from_encoder1_.insert(
        vec_possible_end_pos_from_encoder1_.begin(), end_pos_from_encoder1);
  }
  for (double end_pos_from_encoder1 = possible_end_pos_from_encoder1;
       end_pos_from_encoder1 <= max_end_detect_pos_;
       end_pos_from_encoder1 += 360.0 / encoder1_gear_ratio_) {
    vec_possible_end_pos_from_encoder1_.push_back(end_pos_from_encoder1);
  }
  // std::cout << "possible_end_pos_from_encoder1:" << std::endl;
  // for (auto possible_end_pos_from_encoder1 :
  //      vec_possible_end_pos_from_encoder1_) {
  //   std::cout << possible_end_pos_from_encoder1 << "\t";
  // }
  // std::cout << std::endl;
  // get possible end positions from encoder2
  for (double end_pos_from_encoder2 =
           possible_end_pos_from_encoder2 - 360.0 / encoder2_gear_ratio_;
       end_pos_from_encoder2 >= min_end_detect_pos_;
       end_pos_from_encoder2 -= 360.0 / encoder2_gear_ratio_) {
    vec_possible_end_pos_from_encoder2_.insert(
        vec_possible_end_pos_from_encoder2_.begin(), end_pos_from_encoder2);
  }
  for (double end_pos_from_encoder2 = possible_end_pos_from_encoder2;
       end_pos_from_encoder2 <= max_end_detect_pos_;
       end_pos_from_encoder2 += 360.0 / encoder2_gear_ratio_) {
    vec_possible_end_pos_from_encoder2_.push_back(end_pos_from_encoder2);
  }
  // std::cout << "possible_end_pos_from_encoder2:" << std::endl;
  // for (auto possible_end_pos_from_encoder2 :
  //      vec_possible_end_pos_from_encoder2_) {
  //   std::cout << possible_end_pos_from_encoder2 << "\t";
  // }
  // std::cout << std::endl;

  for (auto i = 0; i < vec_possible_end_pos_from_encoder1_.size(); i++) {
    for (auto j = 0; j < vec_possible_end_pos_from_encoder2_.size(); j++) {
      if (fabs(vec_possible_end_pos_from_encoder1_[i] -
               vec_possible_end_pos_from_encoder2_[j]) < detect_accuracy_) {
        end_pos = vec_possible_end_pos_from_encoder1_[i];
        return end_pos;
      }
    }
  }

  return end_pos;
}
