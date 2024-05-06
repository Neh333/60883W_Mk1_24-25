#include "drive.hpp"

/* Updates voltage given the slewProfile */
void Drive::calculateSlew(double *voltage, double actualVelocity, slewProfile *profile){
{
  if(profile->slew && (fabs(actualVelocity) > profile->slew_lower_thresh) && (fabs(actualVelocity) < profile->slew_upper_thresh))
  {
    double deltaVelocity = voltageToVelocity(*voltage) - actualVelocity;
    if(fabs(deltaVelocity) > profile->slew)
    {
      *voltage = velocityToVoltage(actualVelocity + (profile->slew * sgn(deltaVelocity)));
    }
  }
}
}

void Drive::updateIntegral(double error, double lastError, double activeDistance, double &integral) {
  // Reset integral when crossing the zero point (going from error being positive to negative of vice versa)
  if ((error < 0) != (lastError < 0)) {
    integral = 0;
  }
  // Otherwise only continue to accumulate if within the active range
  else if (fabs(error) <= activeDistance) {
     integral += error;
     integral = std::clamp(integral,-INTEGRAL_MAX,INTEGRAL_MAX);
   }  else {
     integral = 0;
   }
}

 void Drive::updateStandstill(movement_Type type, bool &standStill, double error, double lastError, uint8_t &standStillCount) {
   if (type == lateral) {
     if (SSActive && fabs(lastError - error) <= maxStepDistance) {
       standStillCount++;
       if (standStillCount > SSMaxCount) {
         standStill = true;
       }
     } else {
       standStillCount = 0;
     }
   } 
  
   else if (type == turn_type) {
     if (SSActive_t && fabs(lastError - error) <= maxStepTurn) {
       standStillCount++;
       if (standStillCount > SSMaxCount_t) {
         standStill = true;
       }
     } else {
       standStillCount = 0;
      }
    }
}

 
/* Returns the result of the PID calculation, updates integral */
double Drive::updatePID(double KP, double KI, double KD, double error, double lastError, 
                      double &integral, double integralActive)
{
  updateIntegral(error, lastError, integralActive, integral);
  
  const double derivative = error - lastError;

  return KP*error + KI*integral + KD*derivative;
}