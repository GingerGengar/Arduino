/////////////////////////////////////////////////////////////////////////////////////////////////
//Library Dependencies
/////////////////////////////////////////////////////////////////////////////////////////////////
#include "Arduino_LSM9DS1.h"
#include <MadgwickAHRS.h>

//Comment next line if we dont want to debug 
#define DEBUG 1

//Comment next line if we dont want to print euler angles
#define PRINT_EULER_ANGLE 1

//Comment next line if we dont want to print angular velocity
//#define PRINT_ANGULAR_VEL 1

//Comment next line if we dont want linear acceleration
//#define PRINT_LINEAR_ACCEL 1

//Comment next line if we dont want to print magnetic field readings
//#define PRINT_MAGNETIC 1

//Comment next line if we dont want to see the status of the different sensors
//#define PRINT_SENSOR_STATUS 1

//How many cycles are we going to take before we begin callibration process
#define CAL_BEGIN 10000

//How many cycles are we going to average over in the callibration process
#define CAL_DURATION 500

/////////////////////////////////////////////////////////////////////////////////////////////////
//Madgwick Filter Object
/////////////////////////////////////////////////////////////////////////////////////////////////
Madgwick filter;

unsigned long refreshRate = 400; //333 Hz max for MARG, 500 Hz max for IMU only
unsigned long microsPerReading, microsPrevious, microsNow;
unsigned long i, j = 0;
float gyroCal = 1.0;

float rollCal, pitchCal, yawCal;

//This is angular velocity in body fixed coordinates
float gx, gy, gz; 

//This is linear acceleration in body fixed coordinates
float ax, ay, az; 

//This is the magnetic field lines in body fixed coordinates
float mx, my, mz; 

//These are the euler angles
float roll, pitch, yaw;

/////////////////////////////////////////////////////////////////////////////////////////////////
//Initial Setup Function, every GLOBAL variable must have proper Initializations!!!
/////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  //Setup for the various LEDS just to make sure that the setup process was completed succesfully
  pinMode(LED_BUILTIN, OUTPUT); pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
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
  
  filter.begin(refreshRate); //Start the madgwick filter with the coresponding refresh rate
  //There is 1 million microseconds in a second, dividing that by freq gives microseconds per reading
  microsPerReading = 1000000/refreshRate;
  //This is an initialization of "time" used to make microcontroller run at fixed intervals
  microsPrevious = micros();
   
  //This LED command is to tell us that we have succesfully completed the setup phase 
  digitalWrite(LED_BUILTIN, HIGH);}

/////////////////////////////////////////////////////////////////////////////////////////////////
//Main control Loop
/////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  //Get the Current Time 
  microsNow = micros();
  
  //We only want to run our control loop at fixed intervals
  if (microsNow - microsPrevious >= microsPerReading) {
    
    /////////////////////////////////////////////////////////////////////////////////////////////
    //Read IMU+Magnetometer, Apply Coordinate Transform, Apply Low_Pass_Filters
    /////////////////////////////////////////////////////////////////////////////////////////////
    //Read the IMU for angular velocity only if available
    if (IMU.gyroscopeAvailable()) {//Reading the angular velocity
        IMU.readGyroscope(gx, gy, gz);
        //Angular Velocity
        gz = gz*-1.0;
    }
    
    //Read the IMU for linear acceleration only if available
    if (IMU.accelerationAvailable()) {//Reading the linear acceleration
        IMU.readAcceleration(ax,ay,az);
        //Linear Acceleration Coordinate Transformations
        az = az*-1.0;
    }
    
    //Read the magnetic field lines in x, y, and z only if available
    if (IMU.magneticFieldAvailable()) {//Reading the magnetic field
        IMU.readMagneticField(mx, my, mz);
        //Magnetic Reading Coordinate Transformations
        mx = mx*-1.0; mz = -1.0*mz;
    } else { //This is just in we dont get readings for the Madgwick, to tell we dont have data
      mx = 0.0f; my = 0.0f; mz = 0.0f;}

    /////////////////////////////////////////////////////////////////////////////////////////////
    //MADGWICk Filter Section
    /////////////////////////////////////////////////////////////////////////////////////////////
    //Give the IMU readings over to the madgwick filter
    filter.update(gx,gy,gz,-ax,-ay,az,mx,my,mz); //filter.update(-gx,-gy,gz,ax,ay,az,mx,my,mz);
    //Get the results of the madgwick filter, in roll pitch and yaw
    roll = filter.getRoll(); pitch = filter.getPitch(); yaw = filter.getYaw();
    //Apply a coordinate transformation 
    yaw = yaw*-1.0; //pitch = pitch*-1.0;
    //Apply a callibration to these orientation angles, all these are initiailly zero
    roll = roll - rollCal, pitch = pitch - pitchCal; yaw = yaw - yawCal;

    //This will trigger ONLY ONCE, and it is used for callibration
    if(j == CAL_BEGIN){
      Serial.println("Callibration Triggered!!!");
      rollCal = roll + rollCal; pitchCal = pitch + pitchCal; yawCal = yaw + yawCal;
    }

    microsPrevious = microsPrevious + microsPerReading;
    i++;j++;
    
    #ifdef DEBUG
    if (i == 10){//'\t' means prints a single tab
     
      #ifdef PRINT_SENSOR_STATUS
      Serial.print("\tAccelStatus: ");  Serial.print(IMU.accelerationAvailable());
      Serial.print("\tGyrosStatus: ");  Serial.print(IMU.gyroscopeAvailable());
      Serial.print("\tMagneStatus: ");  Serial.print(IMU.magneticFieldAvailable());
      #endif

      #ifdef PRINT_EULER_ANGLE
      //Print the Euler angle rates roll pitch yaw
      Serial.print(roll);   Serial.print('\t');  
      Serial.print(pitch);  Serial.print('\t');
      Serial.print(yaw);    Serial.print('\t');
      #endif
      
      #ifdef PRINT_ANGULAR_VEL
      //Print the angular velocities in x, y, and z
      Serial.print(gx);     Serial.print('\t');
      Serial.print(gy);     Serial.print('\t');
      Serial.print(gz);     Serial.print('\t');
      #endif

      #ifdef PRINT_LINEAR_ACCEL
      //Print the linear acceleration
      Serial.print(ax);    Serial.print('\t');
      Serial.print(ay);    Serial.print('\t');
      Serial.print(az);   Serial.print('\t');
      #endif
      
      #ifdef PRINT_MAGNETIC
      Serial.print(mx);    Serial.print('\t');
      Serial.print(my);    Serial.print('\t');
      Serial.print(mz);   Serial.print('\t');
      #endif
      
      Serial.println(' ');
      i = 0;
    }
    #endif
  }

  

}
