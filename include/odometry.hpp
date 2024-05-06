#include "main.h"
#include "pros/rotation.hpp"
#include "parametrics.hpp"

class Odometry{
 private:
 pros::Rotation* vertical;
 pros::Rotation* horizontal;
 pros::Imu* imu;
 Pose pose = Pose(0, 0, 0);
 pros::Task *OdomTask = nullptr;
 float lastAngle = 0;

 float calcDeltaTheta(std::vector<std::shared_ptr<pros::IMU>> &imu, bool update = true);
 float calcDeltaTheta(pros::Rotation &tracker1, pros::Rotation &tracker2);

 public:
 Odometry(pros::Rotation &vertical, pros::Rotation &horizontal, pros::Imu &imu){
  this->vertical = &vertical;
  this->horizontal = &vertical;
  this->imu        = &imu;
 }
 void init();
 void calibrate(bool calibrateIMUs = true);
 void update();

};