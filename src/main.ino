
#include "I2Cdev.h"
#include "Servo.h"
#include "PWM.hpp"
#include "Wire.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#include "quaternionFilters.h"
#include "MPU9250.h"
#define I2Cclock 400000                                 // I2C clock is 400 kilobits/s
#define I2Cport Wire                                    // I2C using Wire library
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0             // MPU9250 address when ADO = 0 (0x68)  
MPU9250 FCIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);      // Create FCIMU instance using I2C at 400 kilobits/s


float
Mag_x_offset = 395.815,
Mag_y_offset = 256.995,
Mag_z_offset = -195.48499,
Mag_x_scale = 1.1486549,
Mag_y_scale = 1.0514367,
Mag_z_scale = 0.8486538;

#define True_North false                                // change this to "true" for True North                
float Declination = +0.12254;                           // substitute your magnetic declination 

// ----- Processing variables
char InputChar;                                         // incoming characters stored here
bool LinkEstablished = false;                           // receive flag
String OutputString = "";                               // outgoing data string to Processing

// ----- software timer
unsigned long Timer1 = 500000L;                         // 500mS loop ... used when sending data to to Processing
unsigned long Stop1;                                    // Timer1 stops when micros() exceeds this value


int rPitch, rRoll, rHeading;


unsigned long time;
unsigned long start_time;
unsigned long gps_old_time;
unsigned long gps_new_time;
unsigned long navLights_old_time;
unsigned long navLights_new_time;


Servo elevator; //2
Servo rudder; //3
Servo throttle; //4
Servo rightAileron; //5
Servo leftAileron; //6
Servo payloadDrop; //7
Servo aux1; //8
Servo aux2; //9

PWM CH1(10); //rudder
PWM CH2(11); //elevator
PWM CH3(12); //throttle
PWM CH4(13); //aileron



#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 23

#define LED_PIN    22
#define LED_COUNT 15
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
  //boot tone
  tone(23, 262, 500);
  delay(100);
  tone(23, 330, 500);
  delay(100);
  tone(23, 392, 750);
  colorWipe(strip.Color(255,   0,   0), 1);

  elevator.attach(2);
  rudder.attach(3);
  elevator.write(90);
  rudder.write(90);
  delay(1000);

  elevator.write(50);
  rudder.write(50);
  Serial.print("1");
  delay(1000);
  elevator.write(130);
  rudder.write(130);
  delay(1000);
  Serial.print("1");
  elevator.write(90);
  rudder.write(90);
  delay(1000);
  Serial.print("1");

  pinMode(24, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(28, INPUT_PULLUP);




  Serial.begin(115200);   //Debug

  Serial1.begin(57600);   //telemetry unit

  Serial2.begin(9600);    //GPS Unit
  
  //MPU 
  Wire.begin();                                         // Start I2C as master
  Wire.setClock(400000);                                // Set I2C clock speed to 400kbs
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);
  
  // ----- Look for MPU9250|MPU9255
  byte gyroID = FCIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (!((gyroID == 0x71) || (gyroID == 0x73))){
    // ------ Failed to connect
    Serial.print(F("WHO_AM_I = "));
    Serial.println(gyroID, HEX);
    Serial.println(F("Could not connect to the MPU9250/MPU9255"));
    Serial.println(F("Communication failed ... program aborted !!"));
    Serial.flush();
    abort();
  }
  else
  {
    // ----- MPU9250|MPU9255 found
    if (gyroID == 0x71) Serial.println(F("WHO_AM_I = 0x71\nMPU9250 is online ..."));
    if (gyroID == 0x73) Serial.println(F("WHO_AM_I = 0x73\nMPU9255 is online ..."));
    Serial.println("");

    // ----- Start by performing self test and reporting values
    FCIMU.MPU9250SelfTest(FCIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(FCIMU.selfTest[0], 1); Serial.println("% of factory value");

    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(FCIMU.selfTest[1], 1); Serial.println("% of factory value");

    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(FCIMU.selfTest[2], 1); Serial.println("% of factory value");

    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(FCIMU.selfTest[3], 1); Serial.println("% of factory value");

    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(FCIMU.selfTest[4], 1); Serial.println("% of factory value");

    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(FCIMU.selfTest[5], 1); Serial.println("% of factory value");
    Serial.println("");

    Serial.println(F("Place the compass on a level surface"));
    Serial.println(F(""));


    delay(4000);                        // allow time to place the compass in position

    // ----- Calibrate gyro and accelerometers, load biases in bias registers
    //FCIMU.calibrateMPU9250(FCIMU.gyroBias, FCIMU.accelBias);

    // ----- Initialize device for active mode read of accelerometer, gyroscope, and
    //       temperature
    FCIMU.initMPU9250();
    Serial.println(F("MPU9250 initialized for active data mode...."));
    Serial.println("");
  }
  
  // ----- Look for the AK8963 magnetometer
  byte MagID = FCIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  if (!(MagID == 0x48))
  {
    // ----- Communication failed, stop here
    Serial.print(F("WHO_AM_I = "));
    Serial.println(MagID, HEX);
    Serial.println(F("Could not connect to the AK8963"));
    Serial.println(F("Communication failed ... program aborted !!"));
    Serial.flush();
    abort();
  }
  else
  {
    // ----- AK8963 found
    Serial.println(F("WHO_AM_I = 0x48\nAK8963 is online ..."));
    Serial.println("");

    // ----- Get factory ASA calibration values
    FCIMU.initAK8963(FCIMU.factoryMagCalibration);

    // ----- Initialize device for active mode read of magnetometer
    Serial.println(F("AK8963 initialized for active data mode...."));
    Serial.println("");

    // ----- Display AK8963 fuse rom values
    Serial.println(F("AK8963 Fuse ROM values: "));
    Serial.print(F("ASAX: "));
    Serial.println(FCIMU.factoryMagCalibration[0], 2);
    Serial.print(F("ASAY: "));
    Serial.println(FCIMU.factoryMagCalibration[1], 2);
    Serial.print(F("ASAZ: "));
    Serial.println(FCIMU.factoryMagCalibration[2], 2);
    Serial.println("");

    // ----- Get correct sensor resolutions (You only need to do this once)
    FCIMU.getAres();      // milli-gravity
    FCIMU.getGres();      // dps
    FCIMU.getMres();      // milli-Gauss 14-bit|16-bit

    // ---- display sensor scale multipliers
    Serial.println(F("Sensor-scale multipliers"));
    Serial.print(F("accel: "));
    Serial.println(FCIMU.aRes, 6);
    Serial.print(F(" gyro: "));
    Serial.println(FCIMU.gRes, 6);
    Serial.print(F("  mag: "));
    Serial.println(FCIMU.mRes, 6);
    Serial.println("");
  }

  FCIMU.magBias[0] = Mag_x_offset;
  FCIMU.magBias[1] = Mag_y_offset;
  FCIMU.magBias[2] = Mag_z_offset;

  // ----- Copy the soft-iron scalefactors
  FCIMU.magScale[0] = Mag_x_scale;
  FCIMU.magScale[1] = Mag_y_scale;
  FCIMU.magScale[2] = Mag_z_scale;

  // ----- Display offsets & scale-factors
  Serial.println("");
  Serial.print("Mag_x_offset = ");
  Serial.println(FCIMU.magBias[0]);
  Serial.print("Mag_y_offset = ");
  Serial.println(FCIMU.magBias[1]);
  Serial.print("Mag_z_offset = ");
  Serial.println(FCIMU.magBias[2]);
  Serial.print("Mag_x_scale = ");
  Serial.println(FCIMU.magScale[0]);
  Serial.print("Mag_y_scale = ");
  Serial.println(FCIMU.magScale[1]);
  Serial.print("Mag_z_scale = ");
  Serial.println(FCIMU.magScale[2]);
  Serial.println("");


  //end mpu  

  CH1.begin(true);
  CH2.begin(true);
  CH3.begin(true);
  CH4.begin(true);



  tone(23, 430, 100);
  delay(400);
  tone(23, 430, 100);

  start_time = millis();

  colorWipe(strip.Color(0,   255,   0), 50);
  strip.setPixelColor(1, strip.Color(255,   0,   0));
  strip.show();


}



void loop() {

  refresh_data();  

  if (micros() > Stop1)
  {
    Stop1 += Timer1;                                    // Reset timer

    display_compass_heading_on_serial_monitor();    // Display compass pitch roll & heading on Serial Monitor (115200 bauds)
    Serial.println();
    
  } 



  int roll = 1 * 180/M_PI;
  int pitch = 1 * 180/M_PI;


  roll = map(roll, -90, 90, 0, 180);
  pitch = map(pitch, -90, 90, 0, 180);


  int read1 = CH1.getValue();
  int read2 = CH2.getValue();
  int read3 = CH3.getValue();
  int read4 = CH4.getValue();


  read1 = map(read1, 1000, 2000, -90, 90);
  read2 = map(read2, 1000, 2000, -90, 90);
  read3 = map(read3, 1000, 2000, 0, 180);
  read4 = map(read4, 1000, 2000, -90, 90);


  // Serial.print("\t");
  // Serial.print(read1);
  
  // Serial.print("\t");
  // Serial.print(read2);

  // Serial.print("\t");
  // Serial.print(read3);

  // Serial.print("\t");
  // Serial.println(read4);


  roll = roll + read1;
  pitch = pitch + read2;



  leftAileron.write(roll);
  leftAileron.write(roll);

  elevator.write(pitch);

  throttle.write(read3);

  rudder.write(read2);

  //recieveGS();
  UAVLights();

  
}

void recieveGS(){
  while(Serial1.available() > 0){

  }
}

void transmitGS(){

}



void altitudeHold(int setpoint){


  //get current
  
  
  //Calculate incline

  //
}

//GPS Shit
char gps(50);


void getGPS(char gpsCurrent()){
  //Serial read

  //Parse

  //Store data

}


void APRSTransmit(){
  //for later development
}


void autoWaypointNav(){

}

//void logString(char Rdata(50)){
  //write rdata to new line on sd card
//}

void autoTakeoff(){
  //read accel _ for pulse larger than _, if detected, throw motor 75% and gain altitude to 100 at 20degree pitch
}

void locatorBeacon(bool active){
  //needed? buzzer fluctuate tone from 50 hz to 1000 hz
  if (active){
    
    for (int i = 1000; i <= 2000; i++) {
      tone(23, i, 10);
      delay(1);
    }
    tone(12, 1000, 100);

  }
  else{
    noTone(12);
  }

}

void UAVLights(){
  navLights_new_time = millis();
  if (navLights_new_time - navLights_old_time >= 1000){
    strip.setPixelColor(1, strip.Color(0,0,255));
    strip.setPixelColor(4, strip.Color(0,0,255));
    strip.setPixelColor(7, strip.Color(0,0,255));
    strip.setPixelColor(10, strip.Color(0,0,255));


    strip.show();
    navLights_old_time = navLights_new_time;
  }
  else if (navLights_new_time - navLights_old_time <= 100){
    strip.setPixelColor(1, strip.Color(0,0,255));
    strip.setPixelColor(4, strip.Color(0,0,255));
    strip.setPixelColor(7, strip.Color(0,0,255));
    strip.setPixelColor(10, strip.Color(0,0,255));

    strip.show();
  }
  
  else{
    strip.setPixelColor(1, strip.Color(0,0,0));
    strip.setPixelColor(4, strip.Color(0,0,0));
    strip.setPixelColor(7, strip.Color(0,0,0));
    strip.setPixelColor(10, strip.Color(0,0,0));


    strip.show();
  }
}

void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}




void refresh_data()
{
  // ----- Poll the MPU9250 interrupt status in I2C mode
  if (FCIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    // ----- Read the accelerometer x|y|z register values
    FCIMU.readAccelData(FCIMU.accelCount);                                  // Read accelerometer register values

    // ----- Now we'll calculate the acceleration value into actual g's
    //       This depends on scale being set
    FCIMU.ax = (float)FCIMU.accelCount[0] * FCIMU.aRes;                     // Convert raw register value to milli-Gauss
    FCIMU.ay = (float)FCIMU.accelCount[1] * FCIMU.aRes;                     // Convert raw register value to milli-Gauss
    FCIMU.az = (float)FCIMU.accelCount[2] * FCIMU.aRes;                     // Convert raw register value to milli-Gauss

    // ----- Read the gyro x|y|z register values
    FCIMU.readGyroData(FCIMU.gyroCount);                                    // Read gyro register values

    // ----- Calculate the gyro value into actual degrees per second
    //       This depends on scale being set
    FCIMU.gx = (float)FCIMU.gyroCount[0] * FCIMU.gRes;                      // Convert raw register value to dps  <-+   plus -ve sign for positive pitch
    FCIMU.gy = (float)FCIMU.gyroCount[1] * FCIMU.gRes;                      // Convert raw register value to dps  <-+--- gx & gy interchanged
    FCIMU.gz = (float)FCIMU.gyroCount[2] * FCIMU.gRes;                      // Convert raw register value to dps <----- applied -ve sign for CW rotation

    // Read the magnetometer x|y|z register values
    FCIMU.readMagData(FCIMU.magCount);                                      // Read magnetometer register values

    // ----- Calculate the magnetometer values in milliGauss and  apply
    //       the ASA fuse ROM values and milli-Gauss scale corrections
    //       The MPU92590 magnetometer uses the 14-bit scale-correction of 0.6
    FCIMU.mx = (float)FCIMU.magCount[0] * FCIMU.mRes * FCIMU.factoryMagCalibration[0] - FCIMU.magBias[0];   // Convert/correct raw register value to milli-Gauss
    FCIMU.my = (float)FCIMU.magCount[1] * FCIMU.mRes * FCIMU.factoryMagCalibration[1] - FCIMU.magBias[1];   // Convert/correct raw register value to milli-Gauss
    FCIMU.mz = (float)FCIMU.magCount[2] * FCIMU.mRes * FCIMU.factoryMagCalibration[2] - FCIMU.magBias[2];   // Convert/correct raw register value to milli-Gauss
  }

  // ----- This library function MUST be called before updating the Mahoney quaternions!
  FCIMU.updateTime();

  /*
     The following quaternion values assume that the MPU-9250 gyro X-axis
     is pointing North and that the gyro Z-axis is pointing upwards.

     These values produce:
      - a clockwise heading of 0..360 degrees if we use the formula "Heading = atan2(FCIMU.mx, FCIMU.my;"
      - q0,q1,q2,q3 values of 1,0,0,0 when the compass is pointing north
  */

  MahonyQuaternionUpdate(  FCIMU.ax,              -FCIMU.ay,              FCIMU.az,
                           FCIMU.gx * DEG_TO_RAD, -FCIMU.gy * DEG_TO_RAD, FCIMU.gz * DEG_TO_RAD,
                           FCIMU.my,              -FCIMU.mx,              -FCIMU.mz,
                           FCIMU.deltat);

  //  MadgwickQuaternionUpdate( FCIMU.ax,              -FCIMU.ay,              FCIMU.az,
  //                            FCIMU.gx * DEG_TO_RAD, -FCIMU.gy * DEG_TO_RAD, FCIMU.gz * DEG_TO_RAD,
  //                            FCIMU.my,              -FCIMU.mx,              -FCIMU.mz,
  //                            FCIMU.deltat);

  // ----- calculate the pitch in degrees using Magwick quaternions
  FCIMU.pitch = asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() * *(getQ() + 2)));

  // ----- calculate the roll in degrees using Magwick quaternions
  FCIMU.roll = -atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2) * *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1)
                      * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3) * *(getQ() + 3));

  // ----- calculate the yaw in degrees using Magwick quaternions
  FCIMU.yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() * *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1)
                    * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3) * *(getQ() + 3));
}


void display_compass_heading_on_serial_monitor()
{
  //  // ----- display sample rate (Hz)
  //  Serial.print("rate ");
  //  printNumber((float)FCIMU.sumCount / FCIMU.sum);

  // ----- display quaternions
  Serial.print(" |   q0|qx|qy|qz ");
  Serial.print(*getQ());
  Serial.print(" ");
  Serial.print(*(getQ() + 1));
  Serial.print(" ");
  Serial.print(*(getQ() + 2));
  Serial.print(" ");
  Serial.print(*(getQ() + 3));
  Serial.print(" ");
  

  //  // ----- display accelerometer xyz in milli-gravities (mg)
  //  Serial.print(" |   accel ");
  //  printNumber((short)(FCIMU.ax * 1000));
  //  printNumber((short)(FCIMU.ay * 1000));
  //  printNumber((short)(FCIMU.az * 1000));

  //  // ----- display gyro xyz in degrees/sec (dps)
  //  Serial.print(" |   gyro ");
  //  printNumber((short)(FCIMU.gx));
  //  printNumber((short)(FCIMU.gy));
  //  printNumber((short)(FCIMU.gz));

  //  // ----- display magnetometer xyz in milliGausss (mG)
  //  Serial.print(" |   mag ");
  //  printNumber((short)FCIMU.mx);
  //  printNumber((short)FCIMU.my);
  //  printNumber((short)FCIMU.mz);

  //  // ----- display pitch/roll/yaw in degrees (deg)
  //  Serial.print(" |   p/r/yaw ");
  //  printNumber((short)FCIMU.pitch);
  //  printNumber((short)FCIMU.roll);
  //  float yaw = FCIMU.yaw;
  //  if (yaw > 360.0) yaw -= 360.0;
  //  if (yaw < 0.0) yaw += 360.0;
  //  printNumber((short)yaw);

  //  // ----- display the compass heading in degrees (float)
  //  Serial.print(" |   Heading: ");
  //  printNumber(Heading);

  // ----- display the pitch
  Serial.print(" |   Pitch: ");
  rPitch = (round(FCIMU.pitch * RAD_TO_DEG));

  Serial.print(rPitch);

  // ----- display the roll
  Serial.print(" |   Roll: ");
  rRoll = (round(FCIMU.roll * RAD_TO_DEG));
  Serial.print(rRoll);


  Serial.print(" |   Heading: ");
  float heading = FCIMU.yaw * RAD_TO_DEG;
  if (heading < 0) heading += 360.0;              // Yaw goes negative between 180 amd 360 degrees
  if (True_North == true) heading += Declination; // calculate True North
  if (heading < 0) heading += 360.0; 
  if (heading > 360) heading -= 360.0;
  rHeading = (round(heading));
  Serial.print(rHeading);

}



long printNumber(short number) {
  String myString = String(number);
  short numberChars = myString.length();
  for (short i = 0; i < 6 - numberChars; i++) {
    Serial.print(" ");
  }
  Serial.print(myString);
}

// ----- Routine to stop floats jumping around
float printNumber(float number) {
  String myString = String(number);
  short numberChars = myString.length();
  for (short i = 0; i < 6 - numberChars; i++) {
    Serial.print(" ");
  }
  Serial.print(myString);
}


