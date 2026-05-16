// ================================================
// ESP32 Self-Balancing Cube - Full Complete Code
// Latest upright reading: X ≈ -149, Y ≈ 91
// ================================================

#include <Wire.h>
#include <EEPROM.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ================== PIN DEFINITIONS ==================
#define BUZZER      27
#define VBAT        34
#define BRAKE       26

#define DIR1        4
#define PWM1        32
#define DIR2        15
#define PWM2        25
#define DIR3        5
#define PWM3        18

#define TIMER_BIT   8
#define BASE_FREQ   20000

#define MPU6050     0x68
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG  0x1B
#define PWR_MGMT_1  0x6B

#define accSens     0
#define gyroSens    1

#define EEPROM_SIZE 64

// ================== VARIABLES ==================
float K1 = 160.0f;
float K2 = 10.50f;
float K3 = 0.03f;

int   loop_time = 10;

float Gyro_amount = 0.1f;
float alpha = 0.74f;

bool vertical = false;
bool calibrating = false;
bool calibrated = false;
int  balancing_point = 0;

struct OffsetsObj {
  int   ID1, ID2, ID3, ID4;
  float X1, Y1, X2, Y2, X3, Y3, X4, Y4;
};

OffsetsObj offsets = {0};

int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
int16_t GyX_offset = 0, GyY_offset = 0, GyZ_offset = 0;

float robot_angleX = 0.0f, robot_angleY = 0.0f;
float angleX = 0.0f, angleY = 0.0f;

float gyroY = 0.0f, gyroZ = 0.0f;
float gyroYfilt = 0.0f, gyroZfilt = 0.0f;

int32_t motor_speed_X = 0;
int32_t motor_speed_Y = 0;

unsigned long currentT = 0, previousT_1 = 0, previousT_2 = 0;
unsigned long lastTipTime = 0;

bool showLiveAngle = false;
bool debugMode = false;

// === YOUR MEASURED UPRIGHT OFFSET (updated) ===
const float UPRIGHT_X_OFFSET = -149.0f;
const float UPRIGHT_Y_OFFSET = 91.4f;

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-AshtonCube");

  EEPROM.begin(EEPROM_SIZE);

  pinMode(BUZZER, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, LOW);

  pinMode(DIR1, OUTPUT); 
  pinMode(DIR2, OUTPUT); 
  pinMode(DIR3, OUTPUT);

  ledcAttach(PWM1, BASE_FREQ, TIMER_BIT);
  ledcAttach(PWM2, BASE_FREQ, TIMER_BIT);
  ledcAttach(PWM3, BASE_FREQ, TIMER_BIT);

  Motor1_control(0);
  Motor2_control(0);
  Motor3_control(0);

  EEPROM.get(0, offsets);
  calibrated = (offsets.ID1 == 99 && offsets.ID2 == 99 && offsets.ID3 == 99 && offsets.ID4 == 99);

  robot_angleX = robot_angleY = angleX = angleY = 0.0f;
  motor_speed_X = motor_speed_Y = 0;
  gyroYfilt = gyroZfilt = 0.0f;

  delay(2000);

  digitalWrite(BUZZER, HIGH); delay(70); digitalWrite(BUZZER, LOW);

  angle_setup();

  Serial.println("=== ESP32 Cube Ready ===");
  SerialBT.println("Cube Ready! Send 'h' for help");
}

// ================== LOOP ==================
void loop() {
  currentT = millis();

  if (currentT - previousT_1 >= loop_time) {
    Tuning();
    angle_calc();

    if (showLiveAngle) {
      SerialBT.printf("Live → X: %.2f   Y: %.2f\n", angleX, angleY);
    }

    if (debugMode) {
      Serial.printf("Debug | X:%.2f  Y:%.2f  | BalPt:%d  | Vert:%s\n",
                    angleX, angleY, balancing_point, vertical ? "YES" : "NO");
    }

    if (vertical && calibrated && !calibrating) {
      digitalWrite(BRAKE, HIGH);

      gyroZ = GyZ / 131.0f;
      gyroY = GyY / 131.0f;

      gyroYfilt = alpha * gyroY + (1.0f - alpha) * gyroYfilt;
      gyroZfilt = alpha * gyroZ + (1.0f - alpha) * gyroZfilt;

      float correctedX = angleX - UPRIGHT_X_OFFSET;
      float correctedY = angleY - UPRIGHT_Y_OFFSET;

      float pwm_X = K1 * correctedX + K2 * gyroZfilt + K3 * motor_speed_X;
      float pwm_Y = K1 * correctedY + K2 * gyroYfilt + K3 * motor_speed_Y;

      pwm_X = constrain(pwm_X, -255.0f, 255.0f);
      pwm_Y = constrain(pwm_Y, -255.0f, 255.0f);

      motor_speed_X += (int32_t)pwm_X;
      motor_speed_Y += (int32_t)pwm_Y;

      if (balancing_point == 1) XY_to_threeWay(-pwm_X, -pwm_Y);
      else if (balancing_point == 2) Motor1_control((int)pwm_Y);
      else if (balancing_point == 3) Motor2_control(-(int)pwm_Y);
      else if (balancing_point == 4) Motor3_control((int)pwm_X);
    } else {
      XY_to_threeWay(0, 0);
      digitalWrite(BRAKE, LOW);
      motor_speed_X = motor_speed_Y = 0;
    }

    previousT_1 = currentT;
  }

  if (currentT - previousT_2 >= 2000) {
    battVoltage((double)analogRead(VBAT) / 207.0);

    if (!calibrated && !calibrating && (currentT - lastTipTime > 10000)) {
      SerialBT.println("Tip: Send 'h' for help");
      lastTipTime = currentT;
    }
    previousT_2 = currentT;
  }
}

// ================== TUNING ==================
int Tuning() {
  if (!SerialBT.available()) return 0;

  char param = SerialBT.read();
  char cmd = ' ';

  if (param != 'h' && param != '?' && param != 'a' && param != 'd') {
    if (!SerialBT.available()) return 0;
    cmd = SerialBT.read();
  }

  switch (param) {
    case 'h': 
    case '?':
      showHelp();
      break;

    case 'a':
      showLiveAngle = !showLiveAngle;
      SerialBT.print("Live angles: ");
      SerialBT.println(showLiveAngle ? "ON" : "OFF");
      break;

    case 'd':
      debugMode = !debugMode;
      SerialBT.print("Debug mode: ");
      SerialBT.println(debugMode ? "ON" : "OFF");
      Serial.print("Debug mode: ");
      Serial.println(debugMode ? "ON" : "OFF");
      break;

    case 'p':
      if (cmd == '+') K1 += 1.0f;
      if (cmd == '-') K1 -= 1.0f;
      printValues();
      break;

    case 'i':
      if (cmd == '+') K2 += 0.05f;
      if (cmd == '-') K2 -= 0.05f;
      printValues();
      break;

    case 's':
      if (cmd == '+') K3 += 0.005f;
      if (cmd == '-') K3 -= 0.005f;
      printValues();
      break;

    case 'c':
      if (cmd == '+' && !calibrating) {
        calibrating = true;
        SerialBT.println("\n=== CALIBRATION ===");
        SerialBT.println("Hold cube upright on vertex steadily");
        SerialBT.println("Send 'c-' when readings are stable");
      }
      if (cmd == '-' && calibrating) {
        float avgX = 0.0f, avgY = 0.0f;
        for (int i = 0; i < 25; i++) {
          angle_calc();
          avgX += angleX;
          avgY += angleY;
          delay(12);
        }
        avgX /= 25.0f;
        avgY /= 25.0f;

        SerialBT.printf("Measured Raw → X: %.2f   Y: %.2f\n", avgX, avgY);

        // Forgiving tolerance for your hardware
        if (abs(avgX + 145) < 30 && abs(avgY - 91) < 15) {
          offsets.ID1 = 99;
          offsets.X1 = avgX;
          offsets.Y1 = avgY;
          SerialBT.println("✅ Vertex (Upright) saved successfully!");
          save();
        }
        else if (avgX > -50 && avgX < -20 && avgY > -35 && avgY < -5) {
          offsets.ID2 = 99; offsets.X2 = avgX; offsets.Y2 = avgY;
          SerialBT.println("✅ Edge 1 saved!");
          save();
        }
        else if (avgX > 15 && avgX < 50 && avgY > -35 && avgY < -5) {
          offsets.ID3 = 99; offsets.X3 = avgX; offsets.Y3 = avgY;
          SerialBT.println("✅ Edge 2 saved!");
          save();
        }
        else if (abs(avgX) < 20 && avgY > 25 && avgY < 60) {
          offsets.ID4 = 99; offsets.X4 = avgX; offsets.Y4 = avgY;
          SerialBT.println("✅ Edge 3 saved!");
          save();
        }
        else {
          SerialBT.println("❌ Out of range. Hold steadier and try again.");
          beep(); beep();
        }
      }
      break;
  }
  return 1;
}

void showHelp() {
  SerialBT.println("\n=== Commands ===");
  SerialBT.println("h or ?   - Show help");
  SerialBT.println("a        - Toggle live angles");
  SerialBT.println("d        - Toggle debug (Serial Monitor only)");
  SerialBT.println("p+/p-    - Adjust K1");
  SerialBT.println("i+/i-    - Adjust K2");
  SerialBT.println("s+/s-    - Adjust K3");
  SerialBT.println("c+       - Start calibration");
  SerialBT.println("c-       - Save current position");
}

void printValues() {
  SerialBT.printf("K1=%.1f  K2=%.2f  K3=%.4f\n", K1, K2, K3);
}

// ================== HELPER FUNCTIONS ==================
void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void beep() {
  digitalWrite(BUZZER, HIGH); delay(70); digitalWrite(BUZZER, LOW); delay(80);
}

void save() {
  EEPROM.put(0, offsets);
  EEPROM.commit();
  EEPROM.get(0, offsets);
  calibrated = true;
  calibrating = false;
  SerialBT.println("Calibration saved.");
  beep();
}

void angle_setup() {
  Wire.begin();
  Wire.setClock(400000);
  delay(100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3);
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3);
  delay(100);

  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    sumX += GyX; sumY += GyY; sumZ += GyZ;
    delay(3);
  }
  GyX_offset = sumX >> 10;
  GyY_offset = sumY >> 10;
  GyZ_offset = sumZ >> 10;
  beep(); beep(); beep();
}

void angle_calc() {
  Wire.beginTransmission(MPU6050); Wire.write(0x43); Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);
  GyX = (Wire.read() << 8) | Wire.read();
  GyY = (Wire.read() << 8) | Wire.read();
  GyZ = (Wire.read() << 8) | Wire.read();

  Wire.beginTransmission(MPU6050); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);
  AcX = (Wire.read() << 8) | Wire.read();
  AcY = (Wire.read() << 8) | Wire.read();
  AcZ = (Wire.read() << 8) | Wire.read();

  GyX -= GyX_offset;
  GyY -= GyY_offset;
  GyZ -= GyZ_offset;

  robot_angleX += (float)GyZ * loop_time / 1000.0f / 65.536f;
  float Acc_angleX = atan2(AcY, -AcX) * 57.2958f;
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0f - Gyro_amount);

  robot_angleY += (float)GyY * loop_time / 1000.0f / 65.536f;
  float Acc_angleY = -atan2(AcZ, -AcX) * 57.2958f;
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0f - Gyro_amount);

  angleX = robot_angleX;
  angleY = robot_angleY;
}

void XY_to_threeWay(float pwm_X, float pwm_Y) {
  int16_t m1 = round(0.5f * pwm_X - 0.75f * pwm_Y);
  int16_t m2 = round(0.5f * pwm_X + 0.75f * pwm_Y);
  int16_t m3 = round(-pwm_X);
  Motor1_control(constrain(m1, -255, 255));
  Motor2_control(constrain(m2, -255, 255));
  Motor3_control(constrain(m3, -255, 255));
}

void battVoltage(double voltage) {
  digitalWrite(BUZZER, (voltage > 8.0 && voltage <= 9.5) ? HIGH : LOW);
}

void Motor1_control(int sp) {
  digitalWrite(DIR1, (sp >= 0) ? HIGH : LOW);
  ledcWrite(PWM1, constrain(abs(sp), 0, 255));
}

void Motor2_control(int sp) {
  digitalWrite(DIR2, (sp >= 0) ? HIGH : LOW);
  ledcWrite(PWM2, constrain(abs(sp), 0, 255));
}

void Motor3_control(int sp) {
  digitalWrite(DIR3, (sp >= 0) ? HIGH : LOW);
  ledcWrite(PWM3, constrain(abs(sp), 0, 255));
}