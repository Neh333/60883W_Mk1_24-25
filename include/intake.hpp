#include "include.hpp"
extern bool JAM_PROTECTION_ACTIVE;
extern bool JAM_PROTECTION_INACTIVE;


extern pros::Mutex ringControlMutex;

class IntakeControl{
private:
    const uint16_t jamCycleThreshold = 10;
    const uint16_t dejamThreshold = 15;

    int16_t intakeVel;
    int16_t jamThresh;
    int32_t jamSpeed;
   
    uint8_t  intakeFlag = 0;
    uint16_t jamCycles;
    uint16_t reverseCycles;
    uint16_t deadCycles;
    
    bool jamProtection;
    bool isRunning = false;
    bool notifyOff = false;
    bool lastJamDead = true;
    

public:
    IntakeControl(){
        setJamThresh(360);
        setJamSpeed(-400);
        setJamProtection(JAM_PROTECTION_ACTIVE);
    }

    ~IntakeControl(){
        intake.move_voltage(0);
    }

    void startIntake();
    void stopIntake();
    void setIntake(int16_t velocity);
    void setJamThresh(int16_t velocity);
    void setJamSpeed(int16_t velocity);
    void setJamProtection(bool state);

    void run();

    int32_t intakeSpeed;

}; 
extern IntakeControl conveyor;

//Set the Intakes speed to run at when operating
void setIntake(int16_t velocity);
//Begin running the Intake at the previously specified speed
void startIntake();
//Stop running the Intake (Does not continously run at zero, rather set speed to zero then quits)
void stopIntake();

//float AIOtimeOut;
void IntakeControlSystem_fn(void *param);