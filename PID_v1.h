#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.1.1

class PID
{


  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1
  #define P_ON_M 0
  #define P_ON_E 1

  //commonly used functions **************************************************************************
    PID(uint16_t*, uint16_t*, uint16_t*,        // * constructor.  links the PID to the Input, Output, and 
        uint16_t, uint16_t, uint16_t, int, int);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    PID(uint16_t*, uint16_t*, uint16_t*,        // * constructor.  links the PID to the Input, Output, and 
        uint16_t, uint16_t, uint16_t, int);     //   Setpoint.  Initial tuning parameters are also set here
	
    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(uint8_t, uint8_t); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
	


  //available but not commonly used functions ********************************************************
    void SetTunings(uint16_t, uint16_t,       // * While most users will set the tunings once in the 
                    uint16_t);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(uint16_t, uint16_t,       // * overload for specifying proportional mode
                    uint16_t, int);         	  

	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(uint16_t);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
										  
										  
  //Display functions ****************************************************************
	uint16_t GetKp();						  // These functions query the pid for interal values.
	uint16_t GetKi();						  //  they were created mainly for the pid front-end,
	uint16_t GetKd();						  // where it's important to know what is actually 
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //

  private:
	void Initialize();
	
	//float dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	//float dispKi;				//   format for display purposes
	//float dispKd;				//
    
	uint16_t kp;                  // * (P)roportional Tuning Parameter
  uint32_t ki;                  // * (I)ntegral Tuning Parameter
  uint32_t kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

  uint16_t *myInput;              // * Pointers to the Input, Output, and Setpoint variables
  uint16_t *myOutput;             //   This creates a hard link between the variables and the 
  uint16_t *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
			  
	uint16_t lastTime;
  int32_t outputSum;
	uint16_t lastInput;
  int32_t lastError;

	uint16_t SampleTime;
	uint16_t outMin, outMax;
	bool inAuto, pOnE;
};
#endif

