#include "drive.hpp"

/*Set PID constants changes the row of the array used by taking n and subtracting it by one so set 1 would be row 0 of the array
*Ex: setPID(2) would be set 2 which is located at array[1](the second row of the array)
*Array found in odom.hpp named "pidConstants"
*/
/*Set PID constants */
void Drive::setPID(uint8_t n)
{
  n--;
  kP   = PIDConstants[n].kP;
  kP_a = PIDConstants[n].kP_a;
  kI   = PIDConstants[n].kI;
  kI_a = PIDConstants[n].kI_a;
  kD   = PIDConstants[n].kD;
  kD_a = PIDConstants[n].kD_a;
  kP_d = PIDConstants[n].kP_d;
}

/* Use specified PID constants */
void Drive::setCustomPID(PIDprofile profile)
{
  kP   = profile.kP;
  kP_a = profile.kP_a;
  kI   = profile.kI; 
  kI_a = profile.kI_a;
  kD   = profile.kD;
  kD_a = profile.kD_a;
  kP_d = profile.kP_d;
}

void Drive::setScheduledConstants(PIDprofile constants)
{
  scheduledConstants = constants;
}

void Drive::setScheduledSwerveConstants(PIDprofile constants)
{
  scheduledSwerveConstants = constants;
}

void Drive::setScheduleThreshold_l(double error)
{
  scheduleThreshold_l = error;
}

void Drive::setScheduleThreshold_a(double error)
{
  scheduleThreshold_a = error;
}

void Drive::setScheduleThresholds_s(double error, double error_a)
{
  swerveThresholds.first  = error;
  swerveThresholds.second = error_a;
}

void Drive::setMaxVelocity(float velocity) {
 this->maxVolt = percentToVoltage(velocity);
}

void Drive::setMaxTurnVelocity(float velocity) {
 this->maxVolt_a = percentToVoltage(velocity);
}

/* Update slewProf given profile */
void Drive::setSlew(slewProfile profile)
{
  slewProf = profile;
}

/* Update slewProf given profile */
void Drive::setSlew_a(slewProfile profile)
{
  slewProf_a = profile;
}


void Drive::setStandStill(movement_Type type, uint8_t maxCycles, float maxStep) {
 if (type == lateral) {
     // Deactivate standstill if maxcycles is 0
     SSActive = !(maxCycles == 0);
     SSMaxCount = maxCycles;
     maxStepDistance = maxStep;
    } 

 else if (type == turn_type) {
     // Deactivate standstill if maxcycles is 0
     SSActive_t = !(maxCycles == 0);
     SSMaxCount_t = maxCycles;
     maxStepTurn = maxStep;
 }

}