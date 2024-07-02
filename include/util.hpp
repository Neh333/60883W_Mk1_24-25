#pragma once
#define LAMBDA(func) [](){func;}

int sgn(double x);

double inchToTick(double inch);
double tickToInch(double tick);

double percentToVoltage(double percent);

// Max voltage is 12000, max velocity of standard motor is 200 (all that matters is the percent of max speed): 12000/60 = 200
double voltageToVelocity(double voltage);

// 200*60 = 12000
double velocityToVoltage(double velocity);

double imuTarget(double target);

double radToDeg(double rad);

double degToRad(double deg);

float wrapAngle(float rad);