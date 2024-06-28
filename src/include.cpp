#include "include.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

//define controller
pros::Controller controller(CONTROLLER_MASTER);

/*
pros::Motor frontleft (-2);
pros::Motor midleft   (-11);
pros::Motor backleft  (-20);

pros::Motor frontright(12);
pros::Motor midright  (3);
pros::Motor backright (14);
*/

pros::MotorGroup leftMotors ({-2,-11,-20});
pros::MotorGroup rightMotors ({12,3,14});

//Define V5 sensorss
pros::Imu imu(5);

//Motors
pros::Motor intake(-1);
pros::Motor lift(-13);

//rotation sensor 
pros::Rotation verticalTracker(4);
pros::Rotation horizontalTracker(6);

pros::Rotation liftRot(10);

//ADI Digital Out Devices 
pros::adi::DigitalOut backClamp('G');

//toggle vars
bool hangRachet = true;
bool fn_Lock= true;
