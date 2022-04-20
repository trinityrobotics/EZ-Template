#include "main.h"
#include "trc/util.hpp"
#include "trc/mgps.hpp"
#include "pros/llemu.hpp"
#include "pros/screen.hpp"

using namespace trc;

// Constructor for integrated encoders

// Constructor for integrated encoders and GPS augmented
Mgps::Mgps(std::vector<int> gps_ports, std::vector<double> gps_x_offsets,
        std::vector<double> gps_y_offsets, std::vector<double> gps_yaw_offsets) {
  int i = 0;
  for (auto port : gps_ports) {
    pros::Gps gps(gps_ports[i], gps_x_offsets[i], gps_y_offsets[i]);
    mgps.push_back(gps);
    gps_yaw_offset_.push_back(gps_yaw_offsets[i]);
    i++;
  }
}

void Mgps::wait_gps() {
  while(get_mgps_error() > .02) {
    pros::delay(ez::util::DELAY_TIME);
  }
}

double Mgps::get_mgps_error() {
  double return_error;
  double gps_error = std::numeric_limits<double>::max();
  for (auto gps : mgps) {
    if (gps.get_error() < gps_error) {
      gps_error = gps.get_error();
    }
  }
  return gps_error;
}

double Mgps::get_mgps_heading() {
  double gps_heading;
  double gps_error = std::numeric_limits<double>::max();
  int i = 0;
  for (auto gps : mgps) {
    if (gps.get_error() < gps_error) {
      gps_heading = ez::util::corrected_heading(gps.get_heading(), gps_yaw_offset_[i]);
      if (current_heading_ == 0) {
        current_heading_ = gps_heading;
      } else {
        current_heading_ = (current_heading_ + gps_heading) / 2;
      }
      gps_error = gps.get_error();
    }
    i++;
  }
  return current_heading_;
}

pros::c::gps_status_s_t Mgps::get_mgps_status() {
  pros::c::gps_status_s_t gps_status;
  double gps_error = std::numeric_limits<double>::max();
  for (auto gps : mgps) {
    if (gps.get_error() < gps_error) {
      gps_status = gps.get_status();
      gps_error = gps.get_error();
    }
  }
  return gps_status;
}

double Mgps::get_heading() {
  return get_mgps_heading(); 
}

double Mgps::get_target_heading(bool use_cached) {
  if (!use_cached) {
    pros::c::gps_status_s_t gpsData = get_mgps_status();
    target_heading_ = ez::util::get_angle(gpsData.x, gpsData.y, target_x_, target_y_);  
  }
  return target_heading_;
}

double Mgps::get_target_heading(double target_x, double target_y) {
  pros::c::gps_status_s_t gpsData = get_mgps_status();
  return ez::util::get_angle(gpsData.x, gpsData.y, target_x, target_y);
}

double Mgps::get_target_bearing(bool use_cached) {
  double current_heading_ = Mgps::get_heading();
  double target_heading_ = Mgps::get_target_heading(use_cached);
  double bearing = target_heading_ - current_heading_ < 0 ? target_heading_ - current_heading_ + 360 : target_heading_ - current_heading_;
  // The target is to our left, so reverse the sign/direction.
  if (bearing > 180) {
    return bearing - 360;
  } else {
    return bearing;
  } 
}

double Mgps::get_target_distance() {
  pros::c::gps_status_s_t gpsData = get_mgps_status();
  return ez::util::get_distance(gpsData.x, gpsData.y, target_x_, target_y_, target_offset_);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           double target_heading_ = ez::util::get_angle(gpsData.x, gpsData.y, target_x_, target_y_);
}

double Mgps::get_target_distance(double target_x, double target_y, double target_offset) {
  pros::c::gps_status_s_t gpsData = get_mgps_status();
  return ez::util::get_distance(gpsData.x, gpsData.y, target_x, target_y, target_offset);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           double target_heading_ = ez::util::get_angle(gpsData.x, gpsData.y, target_x_, target_y_);
}

void Mgps::set_position(double x, double y, double heading) {
  double heading_offset;
  int i = 0;
  for (auto gps : mgps) {
    heading_offset =+ gps_yaw_offset_[i];
    gps.set_position(x, y, heading_offset);
    i++;
  }
}

void Mgps::set_target(double input_x, double input_y, double offset) {
  target_x_ = input_x;
  target_y_ = input_y;
  target_offset_ = offset;
}
