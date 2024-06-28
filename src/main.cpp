#include "auton.hpp"
#include "drive.hpp"
#include "include.hpp"
//#include "lvgl_funcs.hpp"`

#define AUTO_NUMBER 8
uint8_t auton = AUTO_NUMBER; 

#define AUTO_SWITCH(){ \
	switch(auton%AUTO_NUMBER){\
    case 0:  controller.print(2, 0, "Win Point   %.2f               ",imu.get_heading()); break;\
		case 1:  controller.print(2, 0, "Left Side   %.2f               ",imu.get_heading()); break;\
		case 2:  controller.print(2, 0, "Right Side  %.2f               ",imu.get_heading()); break;\
		case 3:  controller.print(2, 0, "Left Elims  %.2f               ",imu.get_heading()); break;\
		case 4:  controller.print(2, 0, "Right Elims %.2f               ",imu.get_heading()); break;\
    case 5:  controller.print(2, 0, "Skills      %.2f               ",imu.get_heading()); break;\
		case 6:  controller.print(2, 0, "Nothing     %.2f               ",imu.get_heading()); break;\
    case 7:  controller.print(2, 0, "Tune        %.2f               ",imu.get_heading()); break;\
	}\
}\

void initialize(){
	//initBarGraph();
	//pros::Task brainDisplayTask(updateBarGraph_fn);
	drive.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  
	imu.reset();
} 

void disabled(){
	while(true){
    AUTO_SWITCH()
		//Change auton value
		if(controller.get_digital_new_press(DIGITAL_LEFT)){auton--;}
		if(controller.get_digital_new_press(DIGITAL_RIGHT)){auton++;}
		pros::delay(20);
	}
}

void competition_initialize(){
	while(true){
		//Change auton value
		if(controller.get_digital_new_press(DIGITAL_LEFT)){auton--;}
		if(controller.get_digital_new_press(DIGITAL_RIGHT)){auton++;}
		pros::delay(20);
	}
}

void autonomous(){
  /* Run the auton currently selected and displayed */
	autos[auton%AUTO_COUNT].autonomous();
}

void set_tank(int l_stick, int r_stick) {
  leftMotors.move_voltage(l_stick * (12000.0 / 127.0));
  rightMotors.move_voltage(r_stick * (12000.0 / 127.0));
}

double left_curve_function(double x, double left_curve_scale) {
  if (left_curve_scale != 0) {
    return (powf(2.718, -(left_curve_scale / 10)) + powf(2.718, (fabs(x) - 127) / 10) * (1 - powf(2.718, -(left_curve_scale / 10)))) * x;
  }
  return x;
}

void arcade_standard(double curve) {

 int fwd_stick, turn_stick;

 // Put the joysticks through the curve function
 fwd_stick = left_curve_function(controller.get_analog(ANALOG_LEFT_Y), curve);
 turn_stick = left_curve_function(controller.get_analog(ANALOG_RIGHT_X), curve);

 // Set robot to l_stick and r_stick, check joystick threshold, set active brake
 set_tank(fwd_stick + turn_stick, fwd_stick - turn_stick);
}

void opcontrol() {
 drive.odom->init();
 while (true) {
   /*Display current autonomous on the controller*/
   AUTO_SWITCH()

   /*Change auton value*/
   if(controller.get_digital_new_press(DIGITAL_LEFT)){auton--;}
   if(controller.get_digital_new_press(DIGITAL_RIGHT)){auton++;}
     
   /*Run the currently selected autonomous when UP is pressed*/
   if(controller.get_digital_new_press(DIGITAL_DOWN)){autonomous();}
      
   /*Reset all sensors*/
   if(controller.get_digital_new_press(DIGITAL_UP)){
     drive.odom->calibrate();
     while(drive.imu->is_calibrating()){
       controller.print(2,0,"Calibrating");
       pros::delay(20);
     }
   } 
     
   /*Calibrate Odom*/
   else if (controller.get_digital_new_press(DIGITAL_B)){drive.odom->calibrate(false);}

   /*DRIVER CONTROL */
   arcade_standard(5);
   if (controller.get_digital(DIGITAL_L1)) 
   {
     intake.move_voltage(12000);
   }
   else if (controller.get_digital(DIGITAL_L2)) 
   {
     intake.move_voltage(-12000);
   }
   else {intake.move_voltage(0);}

   if (controller.get_digital(DIGITAL_R1)) 
   {
     lift.move_voltage(12000);
   }
   else if (controller.get_digital(DIGITAL_R2)) 
   {
     lift.move_voltage(-12000);
   }
   else {lift.move_voltage(0);}

   //Debugging 
   pros::screen::print(TEXT_LARGE,0, "Vert Val: %3d", drive.odom->getVertPos());
   pros::screen::print(TEXT_LARGE,2, "Hori Val: %3d", drive.odom->getHoriPos());
   pros::screen::print(TEXT_LARGE,4, "X Val: %3d", drive.odom->getX());
   pros::screen::print(TEXT_LARGE,6, "Y Val: %3d", drive.odom->getY());
     
   pros::delay(20);
  }
}