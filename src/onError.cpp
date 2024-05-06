#include "drive.hpp"

/* onError function to be executed as a task */
void onError_fn(void* param)
{
  pros::Mutex onErrorMutex;
  std::uint32_t startTime = pros::millis();
  while (true){
    onErrorMutex.take();
    if(drive.getPIDStatus())
    {
      auto iter = drive.onErrorVector.begin();
      while(iter != drive.onErrorVector.end())
      {
        if (!iter->called && (iter->onError >= drive.getError()))
        {
          (iter->func)();
          iter->called = true;
        }
        else iter++;
      }
    }
    onErrorMutex.give();
    pros::Task::delay_until(&startTime, 10);  
  }
}


/* Add an error-function tuple to the onErrorVector */
void Drive::addErrorFunc(double onError, void input())
{
  onErrorVector.emplace_back(errorFuncTuple(input, inchToTick(onError), false));
}

const double Drive::getError(){
   return this->error;
}

const bool Drive::getPIDStatus(){
   return this->isNewPID;
}

