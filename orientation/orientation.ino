
#include "Arduino_LSM9DS1.h"
#include <MadgwickAHRS.h>

Madgwick filter;
unsigned long refreshRate = 400; //333 Hz max for MARG, 500 Hz max for IMU only
unsigned long microsPerReading, microsPrevious, microsNow;
unsigned long i = 0;
float gyroCal = 1.125;
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);

  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {
      digitalWrite(LEDR, LOW);
      delay(100);
      digitalWrite(LEDR, HIGH);
      delay(100);
    }
  }
  filter.begin(refreshRate);
  microsPerReading = 1000000/refreshRate;
  microsPrevious = micros();
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  float gx, gy, gz, ax, ay, az, mx, my, mz, roll, pitch, yaw;


  microsNow = micros();

  if (microsNow - microsPrevious >= microsPerReading) {
    IMU.readIMU(ax,ay,az,gx,gy,gz);

    // filter.update(-gx,-gy,gz,ax,ay,az,mx,my,mz);
    filter.updateIMU(gyroCal*gx,gyroCal*gy,gyroCal*gz,-ax,-ay,az);
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();

    analogWrite(LEDR,255*(1-abs(roll)/180));
    analogWrite(LEDG,255*(1-abs(pitch)/180));
    analogWrite(LEDB,255*(1-(yaw-180)/360));
    microsPrevious = microsPrevious + microsPerReading;
    i++;
    if (i == 10){
      Serial.print(roll);
      Serial.print('\t');
      Serial.print(pitch);
      Serial.print('\t');
      Serial.println(yaw-180);
      // Serial.print('\t');
      // Serial.print(gx);
      // Serial.print('\t');
      // Serial.print(gy);
      // Serial.print('\t');
      // Serial.print(gz);
      // Serial.print('\t');
      // Serial.print(-ax);
      // Serial.print('\t');
      // Serial.print(-ay);
      // Serial.print('\t');
      // Serial.println(az);
      i = 0;
    }
  }

  

}
