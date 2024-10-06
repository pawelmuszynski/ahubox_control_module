/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "PID_v1.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(uint16_t* Input, uint16_t* Output, uint16_t* Setpoint,
        uint16_t Kp, uint16_t Ki, uint16_t Kd, int POn, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    PID::SetOutputLimits(0, 255);				//default output limit corresponds to
												//the arduino pwm limits

    SampleTime = 1;							//default Controller Sample Time is 1 second

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd, POn);

    lastTime = millis()/1000-SampleTime;
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PID::PID(uint16_t* Input, uint16_t* Output, uint16_t* Setpoint,
        uint16_t Kp, uint16_t Ki, uint16_t Kd, int ControllerDirection)
    :PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection)
{

}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
   if(!inAuto) return false;
   uint16_t now = millis()/1000;
   uint16_t timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      uint16_t input = *myInput;
      int32_t error = (int32_t)*mySetpoint - input;
      int16_t dInput = input - lastInput;
      int32_t dError = error - lastError;
      outputSum+= (int32_t)(ki * error)/100;
      Serial.println("input: " + String(input));
      Serial.println("setpoint: " + String(*mySetpoint));
      Serial.println("Error: " + String(error));
      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(!pOnE) outputSum-= kp * dInput;

      if(outputSum > outMax) outputSum= outMax;
      else if(outputSum < outMin) outputSum= outMin;

      /*Add Proportional on Error, if P_ON_E is specified*/
	    int16_t output;
      if(pOnE) output = (kp * error)/100;
      else output = 0;

      /*Compute Rest of PID Output*/
      //output += outputSum - (kd * dInput)/1000;
      output += outputSum + (kd * dError)/1000;

	    if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	    *myOutput = output/100;

Serial.println("kp: " + String(kp));
Serial.println("ki: " + String(ki));
Serial.println("kd: " + String(kd/1000));
Serial.println("OutputSum: " + String(outputSum));
Serial.println("dError: " + String(dError));
Serial.println("Output: " + String(output));
Serial.println("----------");

      /*Remember some variables for next time*/
      lastInput = input;
      lastError = error;
      lastTime = now;
	    return true;
   }
   else return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(uint16_t Kp, uint16_t Ki, uint16_t Kd, int POn)
{
  if (Kp<0 || Ki<0 || Kd<0) return;

  pOn = POn;
  pOnE = POn == P_ON_E;

  kp = Kp;
  ki = (uint32_t)Ki * SampleTime;
  kd = (uint32_t)Kd * 1000 / SampleTime;

  if(controllerDirection ==REVERSE) {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID::SetTunings(uint16_t Kp, uint16_t Ki, uint16_t Kd)
{
    SetTunings(Kp, Ki, Kd, pOn); 
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Seconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(uint16_t NewSampleTime)
{
   if (NewSampleTime > 0) {
      float ratio  = (float)NewSampleTime
                      / (float)SampleTime;
/*
                      Serial.print("Ratio: ");
                      Serial.println(ratio);
                      Serial.print("ki: ");
                      Serial.println(ki);
                      Serial.print("kd: ");
                      Serial.println(kd);
*/
      ki *= ratio;
      kd /= ratio;
      SampleTime = NewSampleTime;
/*
                      Serial.print("new ki: ");
                      Serial.println(ki);
                      Serial.print("new kd: ");
                      Serial.println(kd);
*/
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(uint8_t Min, uint8_t Max)
{
   if(Min >= Max) return;
   outMin = (uint16_t)Min * 100;
   outMax = (uint16_t)Max * 100;

   if(inAuto)
   {
	   if(*myOutput > Max) *myOutput = Max;
	   else if(*myOutput < Min) *myOutput = Min;

	   if(outputSum > outMax) outputSum= outMax;
	   else if(outputSum < outMin) outputSum= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
   outputSum = *myOutput*100;
   lastInput = *myInput;
   lastError = 0;
   if(outputSum > outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	    kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
uint16_t PID::GetKp(){ return  kp;}
uint16_t PID::GetKi(){ return  ki / SampleTime;}
uint16_t PID::GetKd(){ return  kd * SampleTime;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}

