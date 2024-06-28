#include "main.h"
#include "pros/rotation.hpp"
#include "parametrics.hpp"
#include <memory>

class Odometry{
 private:
 Pose pose = Pose(0, 0, 0);

 double wheelDiameter, verticalOffset, horizontalOffset;
 
 std::unique_ptr<pros::Task> OdomTask = nullptr;

 /*Update vars*/
 float deltaX = 0;
 float deltaY = 0;

 /*Prev vars*/
 float prevVertical = 0;
 float prevHorizontal = 0;
 float prevImu = 0;

 float calcDeltaTheta(std::vector<std::shared_ptr<pros::IMU>> &imu, bool update = true);
 float calcDeltaTheta(pros::Rotation &tracker1, pros::Rotation &tracker2);

 public:
 Odometry(pros::Rotation &vertical, double verticalOffset, pros::Rotation &horizontal, 
        double horizontalOffset,double wheelDiameter, pros::Imu &imu){
        this->vertical   = &vertical;
        this->horizontal = &horizontal;
        this->imu        = &imu;
        this->verticalOffset = verticalOffset;
        this->horizontalOffset = horizontalOffset;
        this->wheelDiameter  = wheelDiameter;
    }
 
 pros::Rotation* vertical;
 pros::Rotation* horizontal;
 pros::Imu* imu;

 double degreeToInch(double deg);
 double inchToDegree(double inch);

 const int getVertPos();
 const int getHoriPos();
 const int getX();
 const int getY();

 const Pose getCurrentPose();

 void init();
 void calibrate(bool calibrateIMU = true);
 void update();
};