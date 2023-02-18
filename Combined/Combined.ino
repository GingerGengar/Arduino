/////////////////////////////////////////////////////////////////////////////////////////////////
//Library Dependencies
/////////////////////////////////////////////////////////////////////////////////////////////////
//This is for access to the IMU of the arduino
#include "Arduino_LSM9DS1.h"

/////////////////////////////////////////////////////////////////////////////////////////////////
//Various Compilation Options
/////////////////////////////////////////////////////////////////////////////////////////////////
//Comment next line if we dont want to debug 
#define DEBUG 1
//Comment next line if we dont want to print angular velocity
//#define PRINT_ANGULAR_VEL 1
//Comment next line if we dont want to see the status of the different sensors
//#define PRINT_SENSOR_STATUS 1
//Comment next line if we dont want to see the angular callibration values
//#define PRINT_ANGULAR_CAL 1
//Comment next line if we dont want to see the angular velocity integrations
//#define PRINT_INTEGRATION_OMEGA 1
//Comment next line if we dont want to see the controller outputs
//#define PRINT_CONTROLLER_OUTPUT 1
//Comment next line if we dont want to see the controller reference values
#define PRINT_CONTROLLER_REFERENCE 1

/////////////////////////////////////////////////////////////////////////////////////////////////
//Program Specific Pre-processor Directives
/////////////////////////////////////////////////////////////////////////////////////////////////
//How many cycles are we going to take before we begin callibration process
#define CAL_BEGIN 1000
//How many cycles are we going to average over in the callibration process
#define CAL_DURATION 1000
//Beta paramtere used in low pass filtering 0 < beta < 1
#define beta 0.50
//Stability factor to "bleed" out the integration
#define STABILITY_F 0.0002

/////////////////////////////////////////////////////////////////////////////////////////////////
//IMPORTANT TWEAKABLE Controller Values
/////////////////////////////////////////////////////////////////////////////////////////////////
//Important Proportional Integral Derivative Gains that is tweaked from experimentations
//Proportional and Integral Gains for the angular velocity in x
float P_OmegaX = 0.0;
float I_OmegaX = 1.0;
//Proportional and Integral Gains for the angular velocity in y
float P_OmegaY = 0.0;
float I_OmegaY = 1.0;
//Proportional and Integral Gains for the angular velocity in z
float P_OmegaZ = 0.0;
float I_OmegaZ = 1.0;

//What is the "zero-input" from the remote control device, this is duty cycle PWM
//Zero-control position for omega-x
unsigned long TrimInputX = 50;
//Zero-control position for omega-y
unsigned long TrimInputY = 50;
//Zero-control position for omega-z
unsigned long TrimInputZ = 50;

//What is the max range of input pwm duty cycle values so TrimInputX + MaxRangeInpX = maximum val
//Maximum deviation from neutral control for omega-x channel
unsigned long MaxRangeInpX = 20;
//Maximum deviation from neutral control for omega-y channel
unsigned long MaxRangeInpY = 20;
//Maximum deviation from neutral control for omega-z channel
unsigned long MaxRangeInpZ = 20;

//What is the maximum desired angular rates (deg/s)
//Maximum roll rate we want to achieve
float MaxOmegaXDes = 15.0;
//Maximum pitch rate we want to achieve
float MaxOmegaYDes = 15.0;
//Maximum yaw rate we want to achieve
float MaxOmegaZDes = 15.0;

//Trim PWM Output Values 0 means always off, 255 means 100% Duty Cycle
//Default PWM output at trim control surface for omega-x
unsigned long TrimOutX = 127; 
//Default PWM output at trim control surface for omega-y
unsigned long TrimOutY = 127; 
//Default PWM output at trim control surface for omega-z
unsigned long TrimOutZ = 127;

/////////////////////////////////////////////////////////////////////////////////////////////////
//Deifnition of Global Variables
/////////////////////////////////////////////////////////////////////////////////////////////////
//At what frequency do we want to run our main control loop (this is slower than our CPU freq)
unsigned long refreshRate = 400; //333 Hz max for MARG, 500 Hz max for IMU only
//How many microseconds do we have to wait in between reading of the IMU and other sensors
unsigned long microsPerReading = 0.0;
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
float InputOmegaX, InputOmegaY, InputOmegaZ = 0.0;

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
float Inp2RefRate(unsigned long InputDutyCycle, unsigned long TrimInput, 
unsigned long MaxRange, float MaxOmega) {
    return float((float(InputDutyCycle) - float(TrimInput))/float(MaxRange))*MaxOmega;}

/////////////////////////////////////////////////////////////////////////////////////////////////
//Initial Setup Function, every GLOBAL variable must have proper Initializations!!!
/////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  //Setup for the various LEDS just to make sure that the setup process was completed succesfully
  pinMode(LED_BUILTIN, OUTPUT);pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
  //This is the initial state of the LED's this is for debugging purposes
  digitalWrite(LED_BUILTIN, LOW); digitalWrite(LEDR, HIGH); 
  digitalWrite(LEDG, HIGH); digitalWrite(LEDB, HIGH);
  //Start the serial commnunication and enter infinite loop if serial communication can't start
  Serial.begin(9600); while (!Serial); Serial.println("Started");
  
  //If the IMU is unresponsive then say so using the serial print
  if (!IMU.begin()) {Serial.println("Failed to initialize IMU!");
    //If the IMU is unresponsive just keep blinking on and off the red LEDS
    while (1) {digitalWrite(LEDR, LOW); delay(100); //Blink the red LED on
      digitalWrite(LEDR, HIGH); delay(100);}} //Blink the red LED off
  
  //There is 1 million microsec in a second, dividing that by freq gives microsec per reading
  microsPerReading = 1000000/refreshRate;
  //This is an initialization of "time" used to make microcontroller run at fixed intervals
  microsPrevious = micros();

  //TODO: Configure the pin input here for the pwm's
  pinMode(2, OUTPUT);  // sets the pin as output
  //TODO: COnfigure the pin outputs here for the pwm outputs

  //This LED command is to tell us that we have succesfully completed the setup phase 
  digitalWrite(LED_BUILTIN, HIGH);}

/////////////////////////////////////////////////////////////////////////////////////////////////
//Main control Loop
/////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  analogWrite(2, 20); // analogRead values go from 0 to 1023, analogWrite values from 0 to 255

  //Get the Current Time 
  microsNow = micros();
  
  //We only want to run our control loop at fixed intervals
  if (microsNow - microsPrevious >= microsPerReading) {
    
    /////////////////////////////////////////////////////////////////////////////////////////////
    //Read IMU, Apply Coordinate Transform, Apply Low_Pass_Filters
    /////////////////////////////////////////////////////////////////////////////////////////////
    //Read the IMU for angular velocity only if available
    if (IMU.gyroscopeAvailable()) {//Reading the angular velocity
        IMU.readGyroscope(wxRaw, wyRaw, wzRaw);
        //Angular Velocity
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
       
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////////
    //Start of our callibration section, will only trigger once, tho might loop back after 70 mins
    /////////////////////////////////////////////////////////////////////////////////////////////
    j++; if(j >= CAL_BEGIN){
      k++; //Only increment the calibration duration counter once calibration has already begun
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
        //TODO: read in the pwm signal over here to InputOmegaX

        //Map the input PWM signal into the reference desired omega-x
        RefOmegaX = Inp2RefRate(70, TrimInputX, MaxRangeInpX, MaxOmegaXDes);
        //Map the input PWM signal into the reference desired omega-y
        RefOmegaY = Inp2RefRate(70, TrimInputY, MaxRangeInpY, MaxOmegaYDes);
        //Map the input PWM signal into the reference desired omega-z
        RefOmegaZ = Inp2RefRate(70, TrimInputZ, MaxRangeInpZ, MaxOmegaZDes);
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
        
        //TODO: Write the controller output pwm over here, write analog signal to OutputOmegaX
    }

    /////////////////////////////////////////////////////////////////////////////////////////////
    //Debugging Section of our program
    /////////////////////////////////////////////////////////////////////////////////////////////
    #ifdef DEBUG
    i++; //Increment the integer counter used for printing debug messages
    if (i == 10){//'\t' means prints a single tab
      
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
      Serial.print(k);      Serial.print('\t');
      #endif

      #ifdef PRINT_INTEGRATION_OMEGA
      Serial.print(IntOmegaX);  Serial.print('\t');
      Serial.print(IntOmegaY);  Serial.print('\t');
      Serial.print(IntOmegaZ);  Serial.print('\t');
      #endif

      #ifdef PRINT_CONTROLLER_OUTPUT
      Serial.print(OutputOmegaX);   Serial.print('\t');
      Serial.print(OutputOmegaY);   Serial.print('\t');
      Serial.print(OutputOmegaZ);   Serial.print('\t');
      #endif

      #ifdef PRINT_CONTROLLER_REFERENCE
      Serial.print(Inp2RefRate(70, TrimInputX, MaxRangeInpX, MaxOmegaXDes));  Serial.print('\t');
      Serial.print(Inp2RefRate(70, TrimInputY, MaxRangeInpY, MaxOmegaYDes));  Serial.print('\t');
      Serial.print(Inp2RefRate(70, TrimInputZ, MaxRangeInpZ, MaxOmegaZDes));  Serial.print('\t');
      #endif
      
      Serial.println(' ');
      i = 0;
    }
    #endif
    
    //After each time our control lop is run, reset the time counter
    microsPrevious = microsNow;
  }
  
  

}
