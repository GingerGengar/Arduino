/////////////////////////////////////////////////////////////////////////////////////////////////
//Library Dependencies
/////////////////////////////////////////////////////////////////////////////////////////////////
//This is for access to the IMU of the arduino
#include <Servo.h> //To control our servos
#include <ServoInput.h>
#include <MPU6050_tockn.h>
#include <Wire.h> //This is to begin i2c communication

/////////////////////////////////////////////////////////////////////////////////////////////////
//IS THIS VERSION PRODUCTION READY? No means define debug, yes means comment it out
/////////////////////////////////////////////////////////////////////////////////////////////////
//Comment next line if we dont want to debug 
#define DEBUG 1

/////////////////////////////////////////////////////////////////////////////////////////////////
//Specific DEBUG options
/////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef DEBUG //This means that these things wont even be defined if DEBUG is turned off

    /////////////////////////////////////////////////////////////////////////////////////////////
    //These are Input-Output Testing Modes!!! Only ENABLE ONE OR THE OTHER!!!
    /////////////////////////////////////////////////////////////////////////////////////////////
    //Define this if we want to test input to the controller
    //#define INPUT_TEST 1
    //Define this if we want to test output of the controller
    //#define OUTPUT_TEST 1

    /////////////////////////////////////////////////////////////////////////////////////////////
    //These are General Printing Options
    /////////////////////////////////////////////////////////////////////////////////////////////
    //Comment next line if we dont want to print angular velocity
    #define PRINT_ANGULAR_VEL 1
    //Comment next line if we dont want to see the status of the different sensors
    //#define PRINT_SENSOR_STATUS 1
    //Comment next line if we dont want to see the angular callibration values
    //#define PRINT_ANGULAR_CAL 1
    //Comment next line if we dont want to see the angular velocity integrations
    //#define PRINT_INTEGRATION_OMEGA 1
    //Comment next line if we dont want to see the controller outputs
    //#define PRINT_CONTROLLER_OUTPUT 1
    //Comment next line if we dont want to see the controller reference values
    //#define PRINT_CONTROLLER_REFERENCE 1
    ////Comment next line if we dont want to see raw control inputs
    //#define PRINT_RAW_INPUT 1
    //Comment next line if we dont want to see mapped control inputs
    //#define PRINT_MAPPED_INPUT 1

#endif

/////////////////////////////////////////////////////////////////////////////////////////////////
//Program Specific Pre-processor Directives
/////////////////////////////////////////////////////////////////////////////////////////////////
//How many cycles are we going to take before we begin callibration process
#define CAL_BEGIN 100
//How many cycles are we going to average over in the callibration process
#define CAL_DURATION 100
//Beta paramtere used in low pass filtering 0 < beta < 1
#define beta 0.50
//Stability factor to "bleed" out the integration
#define STABILITY_F 0.005
//Conversion from radians to degree
#define RAD2DEG 57.29577951308232

/////////////////////////////////////////////////////////////////////////////////////////////////
//IMPORTANT Pin Input Output Definitions
/////////////////////////////////////////////////////////////////////////////////////////////////
//Here we define the Input Pins given from the RC module
#define PinInOmegaX 3 //Digital Pin 2 for desired omega x
#define PinInOmegaY 4 //Digital Pin 3 for desired omega y
#define PinInOmegaZ 5 //Digital Pin 4 for desired omega z
//Here we define the Output Pins for the various Servos
#define PinOutOmegaX 11 //Digital pin 10 for servo output roll
#define PinOutOmegaY 10  //Digital pin  9 for servo output pitch
#define PinOutOmegaZ 9  //Digital pin  8 for servo output yaw

/////////////////////////////////////////////////////////////////////////////////////////////////
//IMPORTANT TWEAKABLE Controller Values
/////////////////////////////////////////////////////////////////////////////////////////////////
//Important Proportional Integral Derivative Gains that is tweaked from experimentations
//Proportional and Integral Gains for the angular velocity in x
float P_OmegaX = 0.3;
float I_OmegaX = 0.0;
//Proportional and Integral Gains for the angular velocity in y
float P_OmegaY = -0.2;
float I_OmegaY = -0.2;
//Proportional and Integral Gains for the angular velocity in z
float P_OmegaZ = 0.3;
float I_OmegaZ = 0.0;

//What is the "zero-input" from the remote control device, this is duty cycle PWM
//Zero-control position for omega-x
float TrimInputX = 0.42;
//Zero-control position for omega-y
float TrimInputY = 0.50;
//Zero-control position for omega-z
float TrimInputZ = 0.57;

//What is the max range of input pwm duty cycle values so TrimInputX + MaxRangeInpX = maximum val
//Maximum deviation from neutral control for omega-x channel
float MaxRangeInpX = 0.50;
//Maximum deviation from neutral control for omega-y channel
float MaxRangeInpY = 0.50;
//Maximum deviation from neutral control for omega-z channel
float MaxRangeInpZ = 0.50;

//What is the maximum desired angular rates (deg/s)
//Maximum roll rate we want to achieve
float MaxOmegaXDes = 360.0;
//Maximum pitch rate we want to achieve
float MaxOmegaYDes = 360.0;
//Maximum yaw rate we want to achieve
float MaxOmegaZDes = 150.0;

//Trim PWM Output Values 0 means always off, 255 means 100% Duty Cycle
//Default PWM output at trim control surface for omega-x
float TrimOutX = 90; 
//Default PWM output at trim control surface for omega-y
float TrimOutY = 90; 
//Default PWM output at trim control surface for omega-z
float TrimOutZ = 90;

/////////////////////////////////////////////////////////////////////////////////////////////////
//Deifnition of Global Variables
/////////////////////////////////////////////////////////////////////////////////////////////////
//At what frequency do we want to run our main control loop (this is slower than our CPU freq)
//333 Hz max for MARG, 500 Hz max for IMU only
//How many microseconds do we have to wait in between reading of the IMU and other sensors
//There is 1 million microsec in a second, dividing that by freq gives microsec per reading
unsigned long microsPerReading = 1000000/400;
//At what time did we last perform the readings of the IMU and other sensors
unsigned long microsPrevious = 0.0;
//What is the time right now
unsigned long microsNow = 0.0;
//Used to selectivey print DEBUG messages (every few main control loop cycles)
unsigned long i = 0;
//Used to trigger the callibration phase of the sensors and whatnot
unsigned long j = 0;
//Used to keep track of the callibration duration
unsigned long k = 0;
//This is angular velocity in body fixed coordinates
float wx, wy, wz = 0.0; 
//This is the raw values for angular velocity staright from IMU
float wxRaw, wyRaw, wzRaw = 0.0;
//This is the for the calibration of angular velocity
float wxCal, wyCal, wzCal = 0.0;
//float wyroCal = 1.0;
//This is a boolean to represent whether callibration process has been completed or not
bool Cal_Complete = false;
//Integration for angular velocity in x, y, and z
float IntOmegaX, IntOmegaY, IntOmegaZ = 0.0;
//This is a buffer to hold omega-x for integration
float IntWx[] = {0.0, 0.0};
//This is a buffer to hold omega-y for integration
float IntWy[] = {0.0, 0.0};
//This is a buffer to hold omega-z for integration
float IntWz[] = {0.0, 0.0};
//These are the reference angular rates
float RefOmegaX, RefOmegaY, RefOmegaZ = 0.0;
//These are the error between current readings and the desired control
float ErrOmegaX, ErrOmegaY, ErrOmegaZ = 0.0;
//These are the control outputs 
float OutputOmegaX, OutputOmegaY, OutputOmegaZ = 0.0;
//These are the variables that are supposed to read the PWM input
float InputOmegaX = TrimInputX; //If not modified later, assume the input is at trim
float InputOmegaY = TrimInputY; //If not modified later, assume the input is at trim
float InputOmegaZ = TrimInputZ; //If not modified later, assume the input is at trim

//Definitions of the servo
Servo servoOmegaX;
Servo servoOmegaY;
Servo servoOmegaZ;

//This is the mpu 6050 object
MPU6050 mpu6050(Wire);

//This is the rc receiver object
ServoInputPin<PinInOmegaX> RcInputX;
ServoInputPin<PinInOmegaY> RcInputY;
ServoInputPin<PinInOmegaZ> RcInputZ;



/////////////////////////////////////////////////////////////////////////////////////////////////
//Collection of User Defined Functions
/////////////////////////////////////////////////////////////////////////////////////////////////

/*Fourth order Integration scheme
IntResult is a float pointer that holds the main omega results
Placeholders it an array of floats that hold time history of omega values
CurrOmega is the readings that we get for the current time from gyroscope*/
void Ord4Int(float* IntResult, float* Placeholders, float CurrOmega) {
    //Forming how much the integration needs to be appended
    float PartialRes = 0.025*((5.0/12.0)*CurrOmega 
        + (4.0/6.0)*Placeholders[0] 
        - (1.0/12.0)*Placeholders[1]);
    //Performing the integration
    *IntResult = *IntResult + PartialRes - STABILITY_F*(*IntResult);
    //Rolling over the values to the placeholders
    Placeholders[1] = Placeholders[0];
    Placeholders[0] = CurrOmega;}

/*Input to Reference Angular Rates*/
float Inp2RefRate(float InputDutyCycle, float TrimInput, float MaxRange, float MaxOmega) {
    return float((float(InputDutyCycle) - float(TrimInput))/float(MaxRange))*MaxOmega;}

/////////////////////////////////////////////////////////////////////////////////////////////////
//Initial Setup Function, every GLOBAL variable must have proper Initializations!!!
/////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  
  //Start the serial commnunication and enter infinite loop if serial communication can't start
  Serial.begin(9600); while (!Serial); Serial.println("Started Communications...");

  Wire.begin();
  mpu6050.begin();
  //mpu6050.calcGyroOffsets(true);

  //This is an initialization of "time" used to make microcontroller run at fixed intervals
  microsPrevious = micros();

  ///////////////////////////////////////////////////////////////////////////////////////////////
  //Configure the pin outputs here for the pwm outputs
  ///////////////////////////////////////////////////////////////////////////////////////////////
  servoOmegaX.attach(PinOutOmegaX);
  servoOmegaY.attach(PinOutOmegaY);
  servoOmegaZ.attach(PinOutOmegaZ);

  //This is the minimum and maximum of the radio control receiver
  RcInputX.setRangeMin(1012); RcInputX.setRangeMax(2068);
  RcInputY.setRangeMin(1076); RcInputY.setRangeMax(2072);
  RcInputZ.setRangeMin(1004); RcInputZ.setRangeMax(1996);

  }

/////////////////////////////////////////////////////////////////////////////////////////////////
//Main control Loop
/////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  mpu6050.update(); 
  //Get the Current Time 
  microsNow = micros();
  
  //We only want to run our control loop at fixed intervals
  if (microsNow - microsPrevious >= microsPerReading) {
    
    /////////////////////////////////////////////////////////////////////////////////////////////
    //Read IMU, Apply Coordinate Transform, Apply Low_Pass_Filters
    /////////////////////////////////////////////////////////////////////////////////////////////
    //Reading the angular velocity
    wxRaw = mpu6050.getGyroX();
    wyRaw = mpu6050.getGyroY();
    wzRaw = mpu6050.getGyroZ();
    //Applying possible Coordinate Transformation(Ex: wzRaw = wzRaw*-1.0;) 
    wyRaw = wyRaw*-1.0;
    wzRaw = wzRaw*-1.0;
    //Applying Callibration only if Callibration Process is complete
    if (Cal_Complete) {//Callibration is applied to the Raw measurement Data
      wxRaw = wxRaw - wxCal; 
      wyRaw = wyRaw - wyCal; 
      wzRaw = wzRaw - wzCal;}
    //Applying Low-Pass Filters
    wx = (1.0-beta)*wx + beta*wxRaw;
    wy = (1.0-beta)*wy + beta*wyRaw;
    wz = (1.0-beta)*wz + beta*wzRaw;
       
    /////////////////////////////////////////////////////////////////////////////////////////////
    //Start of our callibration section, will only trigger once, tho might loop back after 70 mins
    /////////////////////////////////////////////////////////////////////////////////////////////
    if (j< CAL_BEGIN+2) {j++;} if(j >= CAL_BEGIN){
      //Only increment the calibration duration counter once calibration has already begun
      if (k<CAL_DURATION+2) {k++;}
      //Only print these messages if DEBUG mode is turned on
      #ifdef DEBUG
      if (j==CAL_BEGIN){Serial.println("Callibration Process Started!!!");}
      #endif
      //Summing over the duration to find the average of the measurement readings
      if (k<CAL_DURATION) {
        wxCal = wxCal + wxRaw; wyCal = wyCal + wyRaw; wzCal = wzCal + wzRaw;}
      else if (!Cal_Complete) { //Only run this when calibration hasn't been completed
          #ifdef DEBUG
          Serial.println("Callibration Process Completed!!!");
          #endif
          Cal_Complete = true; //Turn on to signify callibration process has been completed
          //Dividing the summation over how many samples were carried out for average
          wxCal = wxCal/float(CAL_DURATION); 
          wyCal = wyCal/float(CAL_DURATION); 
          wzCal = wzCal/float(CAL_DURATION);}}
    
    /////////////////////////////////////////////////////////////////////////////////////////////
    //Controller Section
    /////////////////////////////////////////////////////////////////////////////////////////////
    if (Cal_Complete) {//Only perform the Controller Section if callibrated
        /////////////////////////////////////////////////////////////////////////////////////////
        //Reading in the Raw PWM values
        /////////////////////////////////////////////////////////////////////////////////////////
        //If we are testing for the output, disable reading inputs
        #ifndef OUTPUT_TEST 
            //Read in the pwm signal over here to InputOmegaX
            if(RcInputX.available()) {InputOmegaX = RcInputX.getPercent();}
            //Read in the pwm signal over here to InputOmegaY
            if (RcInputY.available()) {InputOmegaY = RcInputY.getPercent();}
            //Read in the pwm signal over here to InputOmegaZ
            if (RcInputZ.available()) {InputOmegaZ = RcInputZ.getPercent();}
        #endif
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //Main Processing Body of the PI controller
        /////////////////////////////////////////////////////////////////////////////////////////
        //Map the input PWM signal into the reference desired omega-x
        RefOmegaX = Inp2RefRate(InputOmegaX, TrimInputX, MaxRangeInpX, MaxOmegaXDes);
        //Map the input PWM signal into the reference desired omega-y
        RefOmegaY = Inp2RefRate(InputOmegaY, TrimInputY, MaxRangeInpY, MaxOmegaYDes);
        //Map the input PWM signal into the reference desired omega-z
        RefOmegaZ = Inp2RefRate(InputOmegaZ, TrimInputZ, MaxRangeInpZ, MaxOmegaZDes);
        //Get the error for the omega x
        ErrOmegaX = wx - RefOmegaX;
        //Get the error for the omega y
        ErrOmegaY = wy - RefOmegaY;
        //Get the error for the omegaz
        ErrOmegaZ = wz - RefOmegaZ;
        //Perform integration on omega x
        Ord4Int(&IntOmegaX, IntWx, ErrOmegaX);
        //Perform integration on omega y
        Ord4Int(&IntOmegaY, IntWy, ErrOmegaY);
        //Perform integration on omega z
        Ord4Int(&IntOmegaZ, IntWz, ErrOmegaZ);
        //Produce PI output based on omega-x
        OutputOmegaX = P_OmegaX*ErrOmegaX + I_OmegaX*IntOmegaX + TrimOutX;
        //Produce PI output based on omega-y
        OutputOmegaY = P_OmegaY*ErrOmegaY + I_OmegaY*IntOmegaY + TrimOutY;
        //Produce PI output based on omega-z
        OutputOmegaZ = P_OmegaZ*ErrOmegaZ + I_OmegaZ*IntOmegaZ + TrimOutZ;
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //Output Servo values from 0(0% Duty Cycle) to 255(100% Duty Cycle) over here
        /////////////////////////////////////////////////////////////////////////////////////////
        //If we are testing for the inputs, disable producing outputs
        #ifndef INPUT_TEST
            //Write the PWM signal for omegaX over to its corresponding output pin
            servoOmegaX.write(int(OutputOmegaX));
            //Write the PWM signal for omegaY over to its corresponding output pin
            servoOmegaY.write(int(OutputOmegaY));
            //Write the PWM signal for omegaZ over to its corresponding output pin
            servoOmegaZ.write(int(OutputOmegaZ));
        #endif
    }

    /////////////////////////////////////////////////////////////////////////////////////////////
    //Debugging Section of our program
    /////////////////////////////////////////////////////////////////////////////////////////////
    #ifdef DEBUG
    i++; //Increment the integer counter used for printing debug messages
    if (i == 1){//'\t' means prints a single tab
      
      #ifdef PRINT_SENSOR_STATUS
      Serial.print("AccelStatus: ");  
      Serial.print(IMU.accelerationAvailable());    Serial.print('\t');
      Serial.print("GyrosStatus: ");  
      Serial.print(IMU.gyroscopeAvailable());       Serial.print('\t');
      Serial.print("MagneStatus: ");  
      Serial.print(IMU.magneticFieldAvailable());   Serial.print('\t');
      #endif
      
      #ifdef PRINT_ANGULAR_VEL
      //Print the angular velocities in x, y, and z
      Serial.print(wx);     Serial.print('\t');
      Serial.print(wy);     Serial.print('\t');
      Serial.print(wz);     Serial.print('\t');
      #endif
     
      #ifdef PRINT_ANGULAR_CAL
      //Print the values of the angular callibration variables
      Serial.print(wxCal);  Serial.print('\t');
      Serial.print(wyCal);  Serial.print('\t');
      Serial.print(wzCal);  Serial.print('\t');
      #endif

      #ifdef PRINT_INTEGRATION_OMEGA
      //Print the values of the error of angular velocity integrated
      Serial.print(IntOmegaX);  Serial.print('\t');
      Serial.print(IntOmegaY);  Serial.print('\t');
      Serial.print(IntOmegaZ);  Serial.print('\t');
      #endif

      #ifdef PRINT_CONTROLLER_OUTPUT
      //Print the values of what the controller output ought to be
      Serial.print(OutputOmegaX);   Serial.print('\t');
      Serial.print(OutputOmegaY);   Serial.print('\t');
      Serial.print(OutputOmegaZ);   Serial.print('\t');
      #endif

      #ifdef PRINT_CONTROLLER_REFERENCE
      Serial.print(RefOmegaX);  Serial.print('\t');
      Serial.print(RefOmegaY);  Serial.print('\t');
      Serial.print(RefOmegaZ);  Serial.print('\t');
      #endif
     
      #ifdef PRINT_RAW_INPUT
      Serial.print(RcInputX.getPulseRaw()); Serial.print('\t');
      Serial.print(RcInputY.getPulseRaw()); Serial.print('\t');
      Serial.print(RcInputZ.getPulseRaw()); Serial.print('\t');
      #endif

      #ifdef PRINT_MAPPED_INPUT
      Serial.print(InputOmegaX); Serial.print('\t');
      Serial.print(InputOmegaY); Serial.print('\t');
      Serial.print(InputOmegaZ); Serial.print('\t');
      #endif
      
      Serial.println(' ');
      i = 0;
    }
    #endif
    
    //After each time our control lop is run, reset the time counter
    microsPrevious = microsNow;
  }
  
  

}
