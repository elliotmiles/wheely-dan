#include <Encoder.h>
#include <cmath>

#include <Arduino.h>
#include <Adafruit_BNO08x.h>


#define R_DIR_pin 4
#define R_PWM_pin 5
#define L_DIR_pin 6
#define L_PWM_pin 7

#define L_hall_A 8
#define L_hall_B 9
#define R_hall_A 10
#define R_hall_B 11

#define radius 0.0325 // wheel radius in metres
#define separation 0.202

#define enc_cpr 2800.0 // encoder counts per revolution (700 on datasheet but 4x counting, so 2800)

// ----- IMU -----
#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

long reportIntervalUs = 5000;

float qw, qx, qy, qz;
float gx, gy, gz;
float ax, ay, az;

void setReports(long report_interval) {
  Serial.println("Setting desired reports");

  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, report_interval)) {
    Serial.println("Could not enable quaternion");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, report_interval)) {
    Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, report_interval)) {
    Serial.println("Could not enable accelerometer");
  }
}
// ------------------

Encoder encLeft(L_hall_B, L_hall_A);
Encoder encRight(R_hall_A, R_hall_B);

float linear_vel, angular_vel;
float targetOmegaL, targetOmegaR; // angular velocities around each wheel's respective axis

float omegaLeftFilt = 0;
float omegaLeftPrev = 0;
float omegaRightFilt = 0;
float omegaRightPrev = 0;

float eIntegralLeft = 0;
float eIntegralRight = 0;


// --- send speed to motor ---
void setMotor(int dir, int pwmVal, int pwm_pin, int dir_pin){

  if(dir == 1)
  { 
    digitalWrite(dir_pin,HIGH);
  }
  else
  {
    digitalWrite(dir_pin,LOW);
  }

  analogWrite(pwm_pin,pwmVal); // Motor speed
}

void setup() {

  pinMode(L_DIR_pin, OUTPUT);
  pinMode(L_PWM_pin, OUTPUT);
  pinMode(R_DIR_pin, OUTPUT);
  pinMode(R_PWM_pin, OUTPUT);

  Serial.begin(115200);

  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");

    while (1) {
      delay(10);
    }
  }

  Serial.println("BNO08x Found!");

  setReports(reportIntervalUs);
}


void loop() {

  if (bno08x.wasReset()) {
    Serial.println("Sensor reset");

    setReports(reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        qw = sensorValue.un.arvrStabilizedRV.real;
        qx = sensorValue.un.arvrStabilizedRV.i;
        qy = sensorValue.un.arvrStabilizedRV.j;
        qz = sensorValue.un.arvrStabilizedRV.k;

        break;

      case SH2_GYROSCOPE_CALIBRATED:

        gx = sensorValue.un.gyroscope.x;
        gy = sensorValue.un.gyroscope.y;
        gz = sensorValue.un.gyroscope.z;

        break;

      case SH2_LINEAR_ACCELERATION:

        ax = sensorValue.un.linearAcceleration.x;
        ay = sensorValue.un.linearAcceleration.y;
        az = sensorValue.un.linearAcceleration.z;

        break;
    }
  }

  static long last = 0;
  long now = micros();
  if ((now - last) >= 10000) {

    last = now;

    Serial.print("IMU,");

    Serial.print(qw);
    Serial.print(",");
    Serial.print(qx);
    Serial.print(",");
    Serial.print(qy);
    Serial.print(",");
    Serial.print(qz);
    Serial.print(",");

    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.print(gz);
    Serial.print(",");

    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.println(az);
  }


  static long posLeftPrev = 0;
  static long posRightPrev = 0;
  static unsigned long prevT = 0;

  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');

    int comma_idx = msg.indexOf(',');

    String field1 = msg.substring(0, comma_idx);
    String field2 = msg.substring(comma_idx + 1);

    linear_vel = field1.toFloat();
    angular_vel = field2.toFloat();

    targetOmegaL = (linear_vel - (angular_vel * (separation)/2))/radius;
    targetOmegaR = (linear_vel + (angular_vel * (separation)/2))/radius;
  }
  
  // --- read position ---
  long posLeft = encLeft.read(); // if turning forwards, encoder counts up
  long posRight = encRight.read();

  // --- calculate measured velocity ---
  unsigned long currentT = micros(); 
  if(prevT == 0) 
  {
    prevT = currentT;
    return;
  } 

  if (currentT - prevT < 10000) 
  {
    return;
  }
  float deltaT = ((float) (currentT-prevT))/1.0e6;
  float velLeft = (posLeft - posLeftPrev)/deltaT;
  float velRight = (posRight - posRightPrev)/deltaT;

  posLeftPrev = posLeft;
  posRightPrev = posRight;
  prevT = currentT;

  // --- convert counts/s to rad/s ---
  float omegaLeft = (velLeft * 2.0 * M_PI) / enc_cpr;
  float omegaRight = (velRight * 2.0 * M_PI) / enc_cpr;

  // --- low-pass filter (25Hz cutoff) ---
  omegaLeftFilt = 0.854*omegaLeftFilt + 0.0728*omegaLeft + 0.0728*omegaLeftPrev;
  omegaLeftPrev = omegaLeft;

  omegaRightFilt = 0.854*omegaRightFilt + 0.0728*omegaRight + 0.0728*omegaRightPrev;
  omegaRightPrev = omegaRight;

  Serial.print("Odom,");

  Serial.print(posLeft);
  Serial.print(",");

  Serial.print(posRight);
  Serial.print(",");

  Serial.print(omegaLeftFilt);
  Serial.print(",");

  Serial.println(omegaRightFilt);

  float kp = 60.0;
  float ki = 20.0;
  float eLeft = targetOmegaL - omegaLeftFilt;
  float eRight = targetOmegaR - omegaRightFilt;  
  
  eIntegralLeft += eLeft*deltaT;
  eIntegralRight += eRight*deltaT;  

  eIntegralLeft = constrain(eIntegralLeft, -20, 20);
  eIntegralRight = constrain(eIntegralRight, -20, 20);

  float uLeft = kp*eLeft + ki*eIntegralLeft;
  float uRight = kp*eRight + ki*eIntegralRight;

  // --- set the motor speed and direction ---

  int dirLeft = 1;
  if (uLeft<0)
  {
    dirLeft = -1;
  }
  int pwrLeft = (int) fabs(uLeft);
  if (pwrLeft > 255) 
  {
    pwrLeft = 255;
  }

    int dirRight = 1;
  if (uRight<0)
  {
    dirRight = -1;
  }
  int pwrRight = (int) fabs(uRight);
  if (pwrRight > 255) 
  {
    pwrRight = 255;
  }

  setMotor(-dirLeft, pwrLeft, L_PWM_pin, L_DIR_pin); // note the negative sign
  setMotor(dirRight, pwrRight, R_PWM_pin, R_DIR_pin);
}
