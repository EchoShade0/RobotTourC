#include <Arduino.h>
#include <Wire.h>
#include "src/MeSingleLineFollower.h"
#include "src/MeCollisionSensor.h"
#include "src/MeBarrierSensor.h"
#include "src/MeNewRGBLed.h"
#include <MeMegaPi.h>


/* MOTOR DIAGRAM
10 __^__ 1
  |     |
  |     |
2 ------- 9
*/

MeMegaPiDCMotor motor_1(1);
MeMegaPiDCMotor motor_9(9);
MeMegaPiDCMotor motor_2(2);
MeMegaPiDCMotor motor_10(10);
double angle_rad = PI/180.0;
double angle_deg = 180.0/PI;
MeNewRGBLed rgbled_67(67,4);
MeNewRGBLed rgbled_68(68,4);
MeCollisionSensor collision_65(65);
MeCollisionSensor collision_66(66);
double currentTime = 0;
double lastTime = 0;

float Idx = 0;
String Status = "INIT";
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float elapsedTime, current_Time, previousTime;
int c = 0;
int idx = 0;
int t = 0;
int ci = 1;
bool toggle = false;
#define LOG_CAPACITY      128        // How many entries the buffer holds
#define LOG_ENTRY_MAXLEN  16        // Max characters per entry (incl. '\0')


float AccErrorX = -0.99;
float AccErrorY = 0.66;
float GyroErrorX = -0.68;
float GyroErrorY = 1.71;
float GyroErrorZ = -0.98;
float interval = 20/10; //Interval between moves, Target Time divide by moves

/*Copy and Paste functions from here if needed
Enter_Arena();
Move_Forward();
Move_Backward();
Turn_Right();
Turn_Left();
End_Recenter();
*/
void Put_Plan___Here(){ //Make sure to put ; down after moves
  Plan__2();
}
void Plan__1(){
  while(!((collision_65.isCollision())  ||  (collision_66.isCollision()))) {
    Turn_Right();
    Move_Forward();
  }
}
void Plan__2(){
  Move_Forward();
  Move_Backward();
}
void Plan__3(){
  Enter_Arena();
}
void Plan__4(){
  Enter_Arena();
}

void setup() {
  Serial.begin(9600);
  log("Hello World");
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  rgbled_67.fillPixelsBak(0, 2, 1);
  rgbled_68.fillPixelsBak(0, 2, 1);
  for(int count=0;count<10;count++){
    rgbled_67.setColor(0, 1,153,207);
    rgbled_67.show();
    rgbled_68.setColor(0, 1,153,207);
    rgbled_68.show();
    initmpu();
    idx = 0;
    log("INIT Comp.");
    while(!((collision_65.isCollision())  ||  (collision_66.isCollision())))
    {
      if (t > 50) { //Rotate LEDS while waiting for input on collision pins
        t = 0;
        rotateLight();
      }
      t = t + 1;
      delay(5);
    }

    Status = "Run";
    log("Begin Run");
    rgbled_67.setColor(0, 0,255,0);
    rgbled_67.show();
    rgbled_68.setColor(0, 0,255,0);
    rgbled_68.show();
    lastTime = millis()/1000.0;

    _delay(1);
    
    Put_Plan___Here();

    Status = "Done";
    log("Run Finished"); //255,39,39
    rgbled_67.setColor(0, 255,39,39);
    rgbled_67.show();
    rgbled_68.setColor(0, 255,39,39);
    rgbled_68.show();
    _delay(1);
  }
log("Runs Completed");
while (!collision_65.isCollision()) { //If needed to send log then set to false instead
  if (t > 100) { //blink both LEDs red as all runs are finished
    t = 0;
    toggle = !toggle;
    if (toggle) {
      rgbled_67.setColor(0, 255,39,39);
      rgbled_67.show();
      rgbled_68.setColor(0, 255,39,39);
      rgbled_68.show();
    } else {
      rgbled_67.setColor(0, 0,0,0);
      rgbled_67.show();
      rgbled_68.setColor(0, 0,0,0);
      rgbled_68.show();
    }
  }
  t = t + 1;
  delay(5);
}
log("Flushing Log..."); //Send log
_delay(1);
flushLog(); //just a reminder that the log is now sent to 9600 baud (used to use a different one)
}

void Turn_Right(){

  motor_1.run(-30 / 100.0 * 255);
  motor_9.run(-30 / 100.0 * 255);
  motor_2.run(-30 / 100.0 * 255);
  motor_10.run(-30 / 100.0 * 255);
  
  zero_mpu();
  while (yaw < 90) {
    mpu_update();
    if (false){
      motor_1.run(-20 / 100.0 * 255);
      motor_9.run(-20 / 100.0 * 255);
      motor_2.run(-20 / 100.0 * 255);
      motor_10.run(-20 / 100.0 * 255);
    }
    if (yaw < 70 && yaw > 15){
      motor_1.run(-30 / 100.0 * 255);
      motor_9.run(-30 / 100.0 * 255);
      motor_2.run(-30 / 100.0 * 255);
      motor_10.run(-30 / 100.0 * 255);
    }
    if (yaw > 70){
      motor_1.run(-15 / 100.0 * 255);
      motor_9.run(-15 / 100.0 * 255);
      motor_2.run(-15 / 100.0 * 255);
      motor_10.run(-15 / 100.0 * 255);
    }
  }

  motor_1.run(0 / 100.0 * 255);
  motor_9.run(0 / 100.0 * 255);
  motor_2.run(0 / 100.0 * 255);
  motor_10.run(0 / 100.0 * 255);
  log("TR");

  _delay(interval);
}
void Turn_Left(){ //delay of 1??

  motor_1.run(29.5 / 100.0 * 255);
  motor_9.run(29.5 / 100.0 * 255);
  motor_2.run(29.5 / 100.0 * 255);
  motor_10.run(29.5 / 100.0 * 255);

  zero_mpu();
  while (yaw > -86) {
    mpu_update();
    if (yaw > -70 && yaw < -15){
      motor_1.run(30 / 100.0 * 255);
      motor_9.run(30 / 100.0 * 255);
      motor_2.run(30 / 100.0 * 255);
      motor_10.run(30 / 100.0 * 255);
    }
    if (yaw < -70){
      motor_1.run(15 / 100.0 * 255);
      motor_9.run(15 / 100.0 * 255);
      motor_2.run(15 / 100.0 * 255);
      motor_10.run(15 / 100.0 * 255);
    }
  }

  motor_1.run(0 / 100.0 * 255);
  motor_9.run(0 / 100.0 * 255);
  motor_2.run(0 / 100.0 * 255);
  motor_10.run(0 / 100.0 * 255);
  log("TL");

  _delay(interval);
}
void Move_Forward(){ // original delay 4.6

  motor_1.run(25 / 100.0 * 255);
  motor_9.run(25 / 100.0 * 255);
  motor_2.run(-25 / 100.0 * 255);
  motor_10.run(-25 / 100.0 * 255);

  zero_mpu();
  t = 0;
  while (t < 1450) {
    mpu_update();
    if (yaw > 0.25) {
      motor_1.run(32 / 100.0 * 255);
      motor_9.run(32 / 100.0 * 255);
      motor_2.run(-25 / 100.0 * 255);
      motor_10.run(-25 / 100.0 * 255);
    }
    if (yaw < -0.25) {
      motor_1.run(25 / 100.0 * 255);
      motor_9.run(25 / 100.0 * 255);
      motor_2.run(-32 / 100.0 * 255);
      motor_10.run(-32 / 100.0 * 255);
    }
    if (yaw < 0.1 && yaw > -0.1) {
      motor_1.run(25 / 100.0 * 255);
      motor_9.run(25 / 100.0 * 255);
      motor_2.run(-25 / 100.0 * 255);
      motor_10.run(-25 / 100.0 * 255); 
    }
    t++;

  }
  log("F");
  motor_1.run(0 / 100.0 * 255);
  motor_9.run(0 / 100.0 * 255);
  motor_2.run(0 / 100.0 * 255);
  motor_10.run(0 / 100.0 * 255);

  _delay(interval);
}
void Move_Backward(){

  motor_1.run(-25 / 100.0 * 255);
  motor_9.run(-25 / 100.0 * 255);
  motor_2.run(25 / 100.0 * 255);
  motor_10.run(25 / 100.0 * 255);

  zero_mpu();
  t = 0;
  while (t < 1500) {
    mpu_update();
    if (yaw < -0.25) {
      motor_1.run(-32 / 100.0 * 255);
      motor_9.run(-32 / 100.0 * 255);
      motor_2.run(25 / 100.0 * 255);
      motor_10.run(25 / 100.0 * 255);
    }
    if (yaw > 0.25) {
      motor_1.run(-25 / 100.0 * 255);
      motor_9.run(-25 / 100.0 * 255);
      motor_2.run(32 / 100.0 * 255);
      motor_10.run(32 / 100.0 * 255);
    }
    if (yaw < 0.1 && yaw > -0.1) {
      motor_1.run(-25 / 100.0 * 255);
      motor_9.run(-25 / 100.0 * 255);
      motor_2.run(25 / 100.0 * 255);
      motor_10.run(25 / 100.0 * 255); 
    }
    t++;

  }
  log("B");
  motor_1.run(0 / 100.0 * 255);
  motor_9.run(0 / 100.0 * 255);
  motor_2.run(0 / 100.0 * 255);
  motor_10.run(0 / 100.0 * 255);

  _delay(interval);
}
void End_Recenter(){ //10 cm backwards to put dowel in center of square

  motor_1.run(-25 / 100.0 * 255);
  motor_9.run(-25 / 100.0 * 255);
  motor_2.run(25 / 100.0 * 255);
  motor_10.run(25 / 100.0 * 255);

  zero_mpu();
  t = 0;
  while (t < 250) {
    mpu_update();
    if (yaw < -0.25) {
      motor_1.run(-32 / 100.0 * 255);
      motor_9.run(-32 / 100.0 * 255);
      motor_2.run(25 / 100.0 * 255);
      motor_10.run(25 / 100.0 * 255);
    }
    if (yaw > 0.25) {
      motor_1.run(-25 / 100.0 * 255);
      motor_9.run(-25 / 100.0 * 255);
      motor_2.run(32 / 100.0 * 255);
      motor_10.run(32 / 100.0 * 255);
    }
    if (yaw < 0.1 && yaw > -0.1) {
      motor_1.run(-25 / 100.0 * 255);
      motor_9.run(-25 / 100.0 * 255);
      motor_2.run(25 / 100.0 * 255);
      motor_10.run(25 / 100.0 * 255); 
    }
    t++;

  }
  log("B");
  motor_1.run(0 / 100.0 * 255);
  motor_9.run(0 / 100.0 * 255);
  motor_2.run(0 / 100.0 * 255);
  motor_10.run(0 / 100.0 * 255);

  _delay(interval);
}
void Enter_Arena(){ //moves 35 cm - So center of robot is in center of square

  motor_1.run(25 / 100.0 * 255);
  motor_9.run(25 / 100.0 * 255);
  motor_2.run(-25 / 100.0 * 255);
  motor_10.run(-25 / 100.0 * 255);

  zero_mpu();
  t = 0;
  while (t < 1015) {
    mpu_update();
    if (yaw > 0.5) {
      motor_1.run(32 / 100.0 * 255);
      motor_9.run(32 / 100.0 * 255);
      motor_2.run(-25 / 100.0 * 255);
      motor_10.run(-25 / 100.0 * 255);
    }
    if (yaw < -0.5) {
      motor_1.run(25 / 100.0 * 255);
      motor_9.run(25 / 100.0 * 255);
      motor_2.run(-32 / 100.0 * 255);
      motor_10.run(-32 / 100.0 * 255);
    }
    if (yaw < 0.1 && yaw > -0.1) {
      motor_1.run(25 / 100.0 * 255);
      motor_9.run(25 / 100.0 * 255);
      motor_2.run(-25 / 100.0 * 255);
      motor_10.run(-25 / 100.0 * 255); 
    }
    t++;
  }
  log("Half");
  motor_1.run(0 / 100.0 * 255);
  motor_9.run(0 / 100.0 * 255);
  motor_2.run(0 / 100.0 * 255);
  motor_10.run(0 / 100.0 * 255);

  _delay(interval);
}
void Move_Left(){

  motor_1.run(25 / 100.0 * 255);
  motor_9.run(-25 / 100.0 * 255);
  motor_2.run(-25 / 100.0 * 255);
  motor_10.run(25 / 100.0 * 255);

  zero_mpu();
  t = 0;
  while (t < 2500) {
    mpu_update();
    if (yaw > 1.5) {
      motor_1.run(25 / 100.0 * 255);
      motor_9.run(-32 / 100.0 * 255);
      motor_2.run(-25 / 100.0 * 255);
      motor_10.run(32 / 100.0 * 255);
    }
    if (yaw < -1.5) {
      motor_1.run(25 / 100.0 * 255);
      motor_9.run(-32 / 100.0 * 255);
      motor_2.run(-32 / 100.0 * 255);
      motor_10.run(25 / 100.0 * 255);
    }
    if (yaw < 0.5 && yaw > -0.5) {
      motor_1.run(25 / 100.0 * 255);
      motor_9.run(-25 / 100.0 * 255);
      motor_2.run(-25 / 100.0 * 255);
      motor_10.run(25 / 100.0 * 255); 
    }
    t++;
  }

  motor_1.run(0 / 100.0 * 255);
  motor_9.run(0 / 100.0 * 255);
  motor_2.run(0 / 100.0 * 255);
  motor_10.run(0 / 100.0 * 255);

  _delay(interval);
}
void Move_Right(){

  motor_1.run(-25 / 100.0 * 255);
  motor_9.run(25 / 100.0 * 255);
  motor_2.run(25 / 100.0 * 255);
  motor_10.run(-25 / 100.0 * 255);

  zero_mpu();
  t = 0;
  while (t < 2500) {
    mpu_update();
    if (yaw < -1.5) {
      motor_1.run(-25 / 100.0 * 255);
      motor_9.run(32 / 100.0 * 255);
      motor_2.run(25 / 100.0 * 255);
      motor_10.run(-32 / 100.0 * 255);
    }
    if (yaw > 1.5) {
      motor_1.run(-25 / 100.0 * 255);
      motor_9.run(32 / 100.0 * 255);
      motor_2.run(32 / 100.0 * 255);
      motor_10.run(-25 / 100.0 * 255);
    }
    if (yaw < 0.5 && yaw > -0.5) {
      motor_1.run(-25 / 100.0 * 255);
      motor_9.run(25 / 100.0 * 255);
      motor_2.run(25 / 100.0 * 255);
      motor_10.run(-25 / 100.0 * 255); 
    }
    t++;
  }

  motor_1.run(0 / 100.0 * 255);
  motor_9.run(0 / 100.0 * 255);
  motor_2.run(0 / 100.0 * 255);
  motor_10.run(0 / 100.0 * 255);

  _delay(interval);
}

void rotateLight(){
  rgbled_67.setColor(ci, 1,153,207);
  rgbled_67.show();
  rgbled_68.setColor(ci, 1,153,207);
  rgbled_68.show();
  if (ci == 1){
    rgbled_67.setColor(4, 0,0,0);
    rgbled_67.show();
    rgbled_68.setColor(4, 0,0,0);
    rgbled_68.show();
  }
  else {
    rgbled_67.setColor(ci-1, 0,0,0);
    rgbled_67.show();
    rgbled_68.setColor(ci-1, 0,0,0);
    rgbled_68.show();
  }
  ci++;
  if (ci > 4){
    ci = 1;
  }
}
void flashright() {
  rgbled_67.setColor(0, 255,0,0);
  rgbled_67.show();
  rgbled_68.setColor(0, 0,0,0);
  rgbled_68.show();
}
void flashleft() {
  rgbled_67.setColor(0, 0,0,0);
  rgbled_67.show();
  rgbled_68.setColor(0, 255,0,0);
  rgbled_68.show();
}
void flashcenter() {
  rgbled_67.setColor(0, 0,0,0);
  rgbled_67.show();
  rgbled_68.setColor(0, 0,0,0);
  rgbled_68.show();
}



void _delay(float seconds) {
  if(seconds < 0.0){
    seconds = 0.0;
  }
  long endTime = millis() + seconds * 1000;
  while(millis() < endTime) _loop();
}

void _loop() {
//hello
}

void loop() {
  _loop(); 
}

void initmpu() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  /*
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */
  // Call this function if you need to get the IMU error values for your module
  //calculate_IMU_error();
  log("MPU Init");
  delay(20);
}
void zero_mpu() {
  yaw = 0;
  roll = 0;
  pitch = 0;
}
void mpu_update() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // AccErrorY ~(-1.58) remember operations are flipped
  // === Read gyroscope data === //
  previousTime = current_Time;        // Previous time is stored before the actual time read
  current_Time = millis();            // Current time actual time read
  elapsedTime = (current_Time - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values, operation flipped
  GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-0.66)
  GyroY = GyroY - GyroErrorY; // GyroErrorY ~(1.86)
  GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-1.23)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime * -1;
  // Complementary filter - combine acceleromter and gyro angle values
  //roll =  0.96 * gyroAngleX + 0.04 * accAngleX;
  //pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
  // Print the values on the serial monitor
  //Serial.print(roll);
  //Serial.print("/");
  //Serial.print(pitch);
  //Serial.print("/");
  //Serial.println(yaw);
}
void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 500 times
  AccErrorX = 0; //reset variables as these are used in other areas too
  AccErrorY = 0;
  GyroErrorX = 0;
  GyroErrorY = 0;
  GyroErrorZ = 0;

  while (c < 500) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 500 to get the error value
  AccErrorX = AccErrorX / 500;
  AccErrorY = AccErrorY / 500;
  c = 0;
  // Read gyro values 500 times
  while (c < 500) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 500 to get the error value
  GyroErrorX = GyroErrorX / 500;
  GyroErrorY = GyroErrorY / 500;
  GyroErrorZ = GyroErrorZ / 500;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}

/* ----------  Log storage (circular buffer) ---------- */
struct LogEntry {
  char txt[LOG_ENTRY_MAXLEN];
};

static LogEntry logBuffer[LOG_CAPACITY];
static uint8_t headIdx = 0;   // Index where the next entry will be written
static uint8_t count   = 0;   // Number of valid entries currently stored

/* ----------  Public logging API ---------- */

/**
 * Store a C‑string in the log.
 * If the string is longer than LOG_ENTRY_MAXLEN‑1 it will be truncated.
 */
void log(const char *msg) {
  // Copy (with truncation) into the next slot
  strncpy(logBuffer[headIdx].txt, msg, LOG_ENTRY_MAXLEN - 1);
  logBuffer[headIdx].txt[LOG_ENTRY_MAXLEN - 1] = '\0';   // Ensure NUL termination

  // Advance the circular pointer
  headIdx = (headIdx + 1) % LOG_CAPACITY;
  if (count < LOG_CAPACITY) ++count;   // Grow until full, then stay at max
}

/**
 * Store a flash‑resident string (PROGMEM) – useful for constant messages.
 */
void logf(const __FlashStringHelper *msg) {
  // Copy from flash to RAM, respecting the same size limit
  size_t i = 0;
  while (i < LOG_ENTRY_MAXLEN - 1) {
    char c = pgm_read_byte_near((PGM_P)msg + i);
    if (c == '\0') break;
    logBuffer[headIdx].txt[i] = c;
    ++i;
  }
  logBuffer[headIdx].txt[i] = '\0';

  headIdx = (headIdx + 1) % LOG_CAPACITY;
  if (count < LOG_CAPACITY) ++count;
}

/**
 * Send all stored log lines over Serial, oldest first.
 * After flushing the buffer is cleared.
 */
void flushLog() {
  Serial.println("NAPOLEON v3.0 LOG");
  if (count == 0) {
    Serial.println(F("[Log] empty"));
    return;
  }

  // Compute the index of the oldest entry
  uint8_t idx = (headIdx + LOG_CAPACITY - count) % LOG_CAPACITY;
  for (uint8_t i = 0; i < count; ++i) {
    Serial.println(logBuffer[idx].txt);
    idx = (idx + 1) % LOG_CAPACITY;
  }

  // Reset buffer state
  count   = 0;
  headIdx = 0;
}
