#include "include.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

//define controller
pros::Controller controller(CONTROLLER_MASTER);

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
pros::adi::DigitalOut mogoMechPisses('A');

//Tog vars
bool backClampTog = true;

