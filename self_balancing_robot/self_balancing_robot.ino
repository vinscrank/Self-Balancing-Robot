
#include <PID_v1.h>
#include <Wire.h>
#include <Math.h>
#include "Kalman.h" //


// FUNCTIONS DECLARATION

// EXTERNAL VARIABLES DECLARATION
extern float kp; extern float proportional;
extern float ki; extern float integral;
extern float kd; extern float derivative;
//extern float setpoint;

//PID
double error;
double originalSetpoint = 1.9;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output, input2;

//adjust these values to fit your own design
double Kp = 100;   
double Kd = 5.5;
double Ki = 900;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.7;
double motorSpeedFactorRight = 0.6;


Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
/* IMU Data */
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle; // Angle calculate using the accelerometer
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
double kalAngleX,kalAngleY; // Calculate the angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// Time of the last call to kalman function, using microseconds precision
unsigned long oldTime;



void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  
  // initialize serial communication
  Serial.begin(38400);

  #if PRINT_INFO 
  Serial.println(F("Initializing L298N motor driver..."));
  #endif
  initMotors();
  TWBR = ((F_CPU / 400000L) - 16) / 2;//set gyro rate to 400khz 
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x03; // Set Gyro Full Scale Range to Â±250deg/s
  i2cData[3] = 0x03; // Set Accelerometer Full Scale Range to Â±2g
  while(i2cWrite(0x19,i2cData,4,false)); // Write to all four registers at once
  while(i2cWrite(0x6B,0x01,true)); // PLL with X axis gyroscope reference and disable sleep mode 
  
  while(i2cRead(0x75,i2cData,1));
  if(i2cData[0] != 0x68) 
  {
    // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while(1);
  }
  
  
  delay(100); // Wait for sensor to stabilize
  
  /* Set kalman and gyro starting angle */
  while(i2cRead(0x3B,i2cData,6));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  // atan2 outputs the value of -Ï€ to Ï€ (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2Ï€ and then from radians to degrees
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;


  // initializing oldTime with the current time
  oldTime = micros();

 pid.SetMode(AUTOMATIC);
 pid.SetSampleTime(10);
 pid.SetOutputLimits(-255, 255); 
 
  

}

void loop() {

  // acquiring current time and computing delta time wrt the previous iteration
  // in seconds
  unsigned long time = micros();
  unsigned long dt_usec = time - oldTime;
  float dt_sec = float(dt_usec) / 1000000.0f;
  
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];
    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
   
   error=setpoint-input;
  timer = micros();

 
 

  // PID & Motor Control
  //float pidOutput = updatePID(kalAngleY, dt_sec);
   pid.Compute();
  // enabling motors only if the robot is near the vertical position
  if (abs(kalAngleY) < 30) {
    runMotors(output);
  } else {
    stopMotors();
  }

  // printing values for plotter

 
   //Serial.print(kalAngleY); Serial.print('\t');                  // Kalman filtered angle
   //Serial.print(output/10); Serial.print('\t');
   Serial.print(error);// PID output (normalized between -25 and +25)
  //Serial.print(dt_usec); Serial.print('\t');                      // duration of the cycle in milliseconds
  
  Serial.println();
  

  oldTime = time;

  //delay(15);
  input=kalAngleY;
}
