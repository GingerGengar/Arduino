#include <Arduino_LSM9DS1.h>
#include <MadgwickAHRS.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;

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
  digitalWrite(LED_BUILTIN, HIGH);
  IMU.setOneShotMode();
  // IMU.setContinuousMode();
}

void loop() {
  float gx, gy, gz, ax, ay, az;
  unsigned long starttime = micros();
  for (int i=0; i<100; i++){

    //IMU.readIMU(ax,ay,az,gx,gy,gz);
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx,gy,gz);
    // IMU.readGyroscope(gx,gy,gz);
    // i++;
    // vx = vx + (x-ix)*9.8;
    // vy = vy + (y-iy)*9.8;
    // vz = vz + (z-iz)*9.8;
    analogWrite(LEDR,255*(1-abs(gx)/2000));
    analogWrite(LEDG,255*(1-abs(gy)/2000));
    analogWrite(LEDB,255*(1-abs(gz)/2000));
    // Serial.print(-1);
    // Serial.print('\t');
    // Serial.print(1);
    // Serial.print('\t');
    // Serial.print(p);
    // Serial.print('\t');
    // Serial.print(q);
    // Serial.print('\t');
    // Serial.println(r);
    // Serial.print('\t');
    // Serial.print(x/16);
    // Serial.print('\t');
    // Serial.print(y/16);
    // Serial.print('\t');
    // Serial.println(z/16);
    
  }
  unsigned long endtime = micros(); // take note of end time
  unsigned long duration = endtime - starttime; // duration of the event in microseconds
  Serial.print(100000000/float(duration));
  Serial.print('\t');
  Serial.print(gx);
  Serial.print('\t');
  Serial.print(gy);
  Serial.print('\t');
  Serial.print(gz);
  Serial.print('\t');
  Serial.print(ax);
  Serial.print('\t');
  Serial.print(ay);
  Serial.print('\t');
  Serial.println(az);

}
