#include "include.hpp"

#include "include.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

//define controller
pros::Controller controller(CONTROLLER_MASTER);

pros::Motor frontleft (18);
pros::Motor midleft   (19);
pros::Motor backleft  (20);

pros::Motor frontright(-13);
pros::Motor midright  (-12);
pros::Motor backright (-11);

pros::MotorGroup leftMotors ({18,19,20});
pros::MotorGroup rightMotors ({-13,-12,-11});

//Define V5 sensorss
pros::Imu imu(17);

//Motors
pros::Motor intake(6);
pros::Motor hang(4);

//rotation sensor 
pros::Rotation verticalTracker(5);
pros::Rotation horizontalTracker(21);

pros::Rotation hangRot(9);

//ADI Digital Out Devices 
pros::adi::DigitalOut backWings('G');

//toggle vars
bool backWingTog = true;
bool frontWingTog = true;
bool hangRachet = true;
bool fn_Lock= true;
 
