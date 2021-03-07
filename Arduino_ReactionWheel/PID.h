/**
 * This file is an ultra light/simple PID controller
 * library for Arduino.
 * 
 * Charles GRASSIN, 2019
 * MIT License
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
/* A standard PID controller class */
class PIDController {
private:
  double kp, ki, kd, lastErr, cumulErr;

protected:
  virtual double calcError(double setPoint, double current){
    return setPoint - current;
  }
  
public:
  PIDController(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd), lastErr(0), cumulErr(0){}
  
  double compute(double setPoint,double current, long detaT){
     /*Compute all the working error variables*/
     double error = calcError(setPoint, current);
     double dErr = (detaT==0)?0:calcError(error, lastErr) / detaT;
     
     /*Remember some variables for next time*/
     lastErr = error;
     cumulErr += error * detaT;

     /*Compute PID Output*/
     return kp * error + ki * cumulErr +  kd * dErr;
  }
  
  void reset(){
    lastErr=0;cumulErr=0;
  }
};

/* A PID controller to control angles*/
class PIDAngleController: public PIDController{
  
public:
  PIDAngleController(double kp, double ki, double kd) : PIDController(kp, ki, kd) {}

protected:
  virtual double calcError(double setPoint, double current){
    double distance = fmod(setPoint - current , 360.0);
    if (distance < -180)
        return distance + 360;
    else if (distance > 179)
        return distance - 360;
    return distance;
  }
};
#endif
