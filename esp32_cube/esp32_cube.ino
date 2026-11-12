// =====================================================================
// esp32_cube_enc.ino - optimized firmware + command console (single file)
// Based on github.com/remrc/Self-Balancing-Cube (esp32_cube_enc)
// This must be the ONLY file in the esp32_cube_enc sketch folder.
// Type "help" in the Serial Monitor (115200 baud, New Line) or over Bluetooth.
// =====================================================================
#include <Arduino.h>
#include <strings.h>
#include <Wire.h>
#include <EEPROM.h>
#include <FastLED.h>
#include "soc/gpio_reg.h"

// ============================== PINS ==============================
#define BUZZER      27
#define VBAT        34
#define BRAKE       26          // HIGH = wheels free, LOW = brake

#define DIR1        4
#define ENC1_1      35
#define ENC1_2      33
#define PWM1        32
#define PWM1_CH     1

#define DIR2        15
#define ENC2_1      13
#define ENC2_2      14
#define PWM2        25
#define PWM2_CH     0

#define DIR3        5
#define ENC3_1      16
#define ENC3_2      17
#define PWM3        18
#define PWM3_CH     2

#define TIMER_BIT   8
#define BASE_FREQ   20000

// ============================== LEDS ==============================
#define STATUS_LED  2           // blue LED on the ESP32 board (set to -1 if your board has none)
#define LED_PIN     19          // optional WS2812B RGB LEDs
#define NUM_PIXELS  3
#define LED_ORDER   RGB         // if the first power-up flash is GREEN instead of RED, change to GRB
#define LED_RED       0xC80000
#define LED_GREEN     0x00C800
#define LED_YELLOW    0xC89600
#define LED_CYAN      0x00C8C8
#define LED_PURPLE    0x8C00C8
#define LED_DIM_RED   0x300000
#define LED_DIM_GREEN 0x003000

// ============================== CONSOLE ==============================
#define SERIAL_BAUD    115200
#define USE_BLUETOOTH  1            // turned off automatically on chips without Classic BT
#define BT_NAME        "ESP32-Cube"
#if USE_BLUETOOTH && defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED) && defined(CONFIG_BT_SPP_ENABLED)
  #define HAVE_BT 1
  #include "BluetoothSerial.h"
  BluetoothSerial SerialBT;
#else
  #define HAVE_BT 0
#endif

// ============================== MPU6050 ==============================
#define MPU_ADDR        0x68
#define MPU_SMPLRT_DIV  0x19
#define MPU_CONFIG      0x1A
#define MPU_GYRO_CFG    0x1B
#define MPU_ACCEL_CFG   0x1C
#define MPU_ACCEL_XOUT  0x3B
#define MPU_PWR_MGMT_1  0x6B

#define ACC_1G          16384.0f    // LSB per g at +-2 g
#define GYRO_LSB_DPS    131.0f      // LSB per deg/s at +-250 deg/s
// The original integrates the gyro with 65.536 although the sensor is at +-250 deg/s
// (131 LSB per deg/s), so the fused angle reacts 2x to fast motion. The stock gains
// were tuned like that, so it is kept. If you set this to 131.0f, roughly double K1/eK1.
#define GYRO_INTEG_LSB  65.536f
#define RAD2DEG         57.2957795f

// ============================== CONTROL LOOP ==============================
#define LOOP_MS          15         // all gains are tuned for 15 ms
#define LOOP_S           (LOOP_MS / 1000.0f)
#define VERTEX_ACX_MAX   2000       // |AcX| below this = standing on the vertex
#define EDGE_ACX_MIN     7000       // |AcX| inside this window = standing on the edge
#define EDGE_ACX_MAX     10000
#define CAL_ACY_MAX      2000
#define I2C_FAIL_LIMIT   3          // bad IMU reads in a row before the motors stop

// ============================== CALIBRATION / TEST JOBS ==============================
#define CAL_SAMPLES       100       // 1.5 s average per balance point
#define CAL_MAX_DPS       15.0f     // reject if the cube rotated faster than this
#define GYRO_SAMPLES      134       // ~2 s
#define GYRO_STILL_RANGE  300       // max raw peak-to-peak while zeroing the gyro
#define AUTOTRIM_SAMPLES  600       // ~9 s of balancing
#define MTEST_PWM         70
#define MTEST_OK_COUNTS   150       // counts in ~0.3 s = wheel spinning the right way
#define MTEST_REV_COUNTS  100

// ============================== SAVED SETTINGS ==============================
#define EEPROM_SIZE   256
#define CFG_MAGIC     0xC0BE5A02UL

struct CubeConfig {
  uint32_t magic;
  float acXv, acYv, acZv;            // accel at the vertex balance point
  float acXe, acYe, acZe;            // accel at the edge balance point
  uint8_t vertexCal, edgeCal, rsv0, rsv1;
  float K1, K2, K3, K4, zK2, zK3;    // vertex controller
  float eK1, eK2, eK3, eK4;          // edge controller
  float tvx, tvy, te;                // balance-point trims, degrees
  float gyroAmt, alpha;              // complementary filter weight, rate low-pass
  float armV, armE, fall;            // start / stop angles, degrees
  float vbatDiv;                     // ADC counts per volt
  float dlpf;                        // MPU6050 low-pass setting 0..6
  uint32_t crc;
};
static_assert(sizeof(CubeConfig) <= EEPROM_SIZE, "CubeConfig does not fit");

struct LegacyOffsets { int ID; float acXv, acYv, acZv, acXe, acYe, acZe; };  // original layout

enum CubeMode : uint8_t { MODE_IDLE = 0, MODE_VERTEX, MODE_EDGE };
enum CubeJob  : uint8_t { JOB_NONE = 0, JOB_GYRO, JOB_CAL_VERTEX, JOB_CAL_EDGE, JOB_MTEST };
enum LedPat   : uint8_t { PAT_OFF = 0, PAT_ON, PAT_SLOW, PAT_MED, PAT_FAST, PAT_DOUBLE, PAT_BLIP };

struct Sampler {
  int32_t n, sAx, sAy, sAz, sGx, sGy, sGz;
  int16_t gMin[3], gMax[3];
  float maxDps;
};
struct TuneParam { const char *name; float *ptr; float minV, maxV, step; const char *help; };
struct LineBuf   { char buf[128]; uint8_t len; uint32_t lastMs; };

// ISR-safe GPIO read straight from the input registers
#define FAST_READ(pin) ((pin) < 32 ? (int)(((*(volatile uint32_t *)GPIO_IN_REG) >> ((pin) & 31)) & 1) \
                                   : (int)(((*(volatile uint32_t *)GPIO_IN1_REG) >> (((pin) - 32) & 31)) & 1))
// Quadrature table, index = prevA prevB A B (same transitions as the original code).
// Not const, so it lives in RAM and is safe to read from the encoder interrupts.
int8_t QDEC[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// ============================== SHARED STATE ==============================
CubeConfig cfg;
bool cfgDirty = false;

volatile CubeMode cubeMode = MODE_IDLE;
volatile CubeJob  cubeJob  = JOB_NONE;
volatile CubeJob  pendingJob = JOB_NONE;
volatile bool motorsEnabled = true, jobDone = false, pendingDlpf = false, mtAborted = false;
volatile bool evtArmed = false, evtFell = false;
volatile bool atActive = false, atDone = false, atAborted = false;
volatile bool imuNeedInit = false, evtImuInit = false;   // (re)wake the MPU6050 from the control task

Sampler smp;
int jobTick = 0;
int32_t mtStart = 0, mtResult[3] = {0, 0, 0};
int atN = 0;
float atSumX = 0, atSumY = 0;
CubeMode atMode = MODE_IDLE, armedMode = MODE_IDLE;
float fallX = 0, fallY = 0;

int16_t AcX, AcY, AcZ, GyXr, GyYr, GyZr;                  // raw IMU
float gyroOffX = 0, gyroOffY = 0, gyroOffZ = 0;           // raw LSB
float gyroX, gyroY, gyroZ, gyroXfilt, gyroYfilt;          // deg/s
float robot_angleX, robot_angleY, Acc_angleX, Acc_angleY; // deg

volatile int32_t encCount[3] = {0, 0, 0};                 // only written by the ISRs
int32_t encLast[3] = {0, 0, 0};
int16_t motor_speed[3] = {0, 0, 0};                       // counts per loop
float speed_X, speed_Y, motors_speed_X, motors_speed_Y;
int32_t motors_speed_Z;
float outX, outY, outZ;

uint32_t loopCount = 0, overruns = 0, execMaxUs = 0, periodMaxUs = 0, lastStepUs = 0;
uint32_t i2cErrors = 0;
uint8_t i2cFailRun = 0;
float vbat = 0;

LineBuf usbBuf = {}, btBuf = {};
uint32_t pendingAt = 0, nextChirp = 0;
Print *telOut = nullptr;
uint32_t telPeriod = 0, telNext = 0;
Print *watchOut = nullptr;             // "watch": human-readable position guide
uint32_t watchPeriod = 0, watchNext = 0;
uint32_t ledResultUntil = 0;           // LED result display (good = solid/green, bad = flicker/red)
bool ledResultOk = false;

CRGB leds[NUM_PIXELS];
TaskHandle_t ctrlTask = nullptr;

static_assert(configTICK_RATE_HZ == 1000, "Control loop expects a 1 kHz FreeRTOS tick");

// ============================== FUNCTION DECLARATIONS ==============================
void controlTask(void *arg);
void controlStep();
void updateAngles();
void enterMode(CubeMode m);
void updateMode();
void runVertex();
void runEdge();
void idleOutputs();
void runJob(bool imuOk);
void handleHousekeeping();
uint32_t cfgCrc(const CubeConfig &c);
void cfgDefaults(CubeConfig &c, bool keepCalibration);
int cfgRead(CubeConfig &c);
void cfgWrite();
void cfgReplace(const CubeConfig &n);
void imuWrite(uint8_t reg, uint8_t val);
bool imuPresent();
void imuConfigure();
void imuInitRegs();
bool imuRead();
bool gyroCalibrateBlocking();
void encodersInit();
void motorPwm(int i, uint32_t duty);
void motorsInit();
void motorRaw(int i, int sp);
void motorControl(int i, int sp);
void XYZ_to_threeWay(float pwm_X, float pwm_Y, float pwm_Z);
void threeWay_to_XY(int s1, int s2, int s3);
void beep();
void chirp();
void setLeds(uint8_t r, uint8_t g, uint8_t b);
void showLeds(int n, uint32_t c);
void statusLed(bool on);
bool patLevel(uint8_t p, uint32_t now);
bool poseOk(CubeJob j);
void flashResult(bool ok);
float readBattery();
void logAll(const char *fmt, ...);
bool cmdIs(const char *a, const char *b);
float toFloat(const char *s);
TuneParam *findParam(const char *name);
void printParam(const TuneParam *p, Print &out);
void applyParam(TuneParam *p, float v, Print &out);
bool paramShortcut(int argc, char **argv, Print &out);
void printHelp(Print &out, const char *topic);
void printStatus(Print &out);
void scheduleJob(CubeJob j, int secs, Print &out);
void startJob(CubeJob j);
void handleCountdown();
void finishGyroCal();
void finishBalanceCal(bool edge);
void finishMotorTest();
void handleJobResults();
void handleEvents();
void handleTelemetry();
void handleWatch();
void runLine(LineBuf &lb, Print &out);
void pollStream(Stream &in, Print &out, LineBuf &lb, bool idleFlush);
void handleCommand(char *line, Print &out);

// ============================== SETUP / LOOP ==============================
void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(BUZZER, OUTPUT); digitalWrite(BUZZER, LOW);
  pinMode(BRAKE, OUTPUT);  digitalWrite(BRAKE, LOW);      // wheels braked until balancing
#if STATUS_LED >= 0
  pinMode(STATUS_LED, OUTPUT);
#endif
  statusLed(false);
  motorsInit();
  encodersInit();

  FastLED.addLeds<WS2812B, LED_PIN, LED_ORDER>(leds, NUM_PIXELS);
  setLeds(120, 0, 0); delay(300);                        // RGB colour check: red, green, blue
  setLeds(0, 120, 0); delay(300);
  setLeds(0, 0, 120); delay(300);
  setLeds(0, 0, 0);

  EEPROM.begin(EEPROM_SIZE);
  int loaded = cfgRead(cfg);

#if HAVE_BT
  SerialBT.begin(BT_NAME);
#endif

  Wire.begin();
  Wire.setClock(400000);                                 // 4x the default I2C speed
  bool imuOk = imuPresent();
  bool gyroOk = false;
  if (imuOk) {
    imuConfigure();
    setLeds(0, 0, 150);                                  // zeroing the gyro - keep still
    statusLed(true);
    for (int i = 0; i < 5 && !gyroOk; i++) {            // ~1 s, repeated if the cube moved
      gyroOk = gyroCalibrateBlocking();
      if (!gyroOk) { beep(); delay(400); }
    }
    setLeds(0, 0, 0);
    statusLed(false);
    if (imuRead()) { updateAngles(); robot_angleX = Acc_angleX; robot_angleY = Acc_angleY; }
  } else {
    imuNeedInit = true;                                  // keep looking; wake it when it shows up
  }
  beep();
  vbat = readBattery();

  // Balancing runs in its own high-priority task with an exact 15 ms period, so
  // console, Bluetooth and LED work in loop() can never delay it.
  xTaskCreatePinnedToCore(controlTask, "ctrl", 4096, nullptr, 10, &ctrlTask, xPortGetCoreID());

  logAll(" ");
  logAll("Self-balancing cube ready - type help  (USB %d baud%s)", SERIAL_BAUD,
         HAVE_BT ? ", Bluetooth \"" BT_NAME "\"" : "");
  if (loaded == 1) logAll("Imported the calibration from the original firmware (gains at defaults).");
  if (loaded == 0) logAll("No saved settings - using defaults.");
  if (!imuOk)       logAll("ERROR: no MPU6050 at 0x68 - check SDA/SCL. Motors stay off (retrying).");
  else if (!gyroOk) logAll("WARNING: cube moved while zeroing the gyro. Lay it still and type gyrocal.");
  else              logAll("Gyro offsets %.1f %.1f %.1f", gyroOffX, gyroOffY, gyroOffZ);
  if (!cfg.vertexCal && !cfg.edgeCal) logAll("Not calibrated yet: run cal v and cal e, then save.");
  logAll("Battery %.2f V", vbat);
  printHelp(Serial, nullptr);
}

void loop() {
  pollStream(Serial, Serial, usbBuf, false);
#if HAVE_BT
  pollStream(SerialBT, SerialBT, btBuf, true);
#endif
  handleCountdown();
  handleJobResults();
  handleEvents();
  handleTelemetry();
  handleWatch();
  handleHousekeeping();
  delay(1);
}

// ============================== CONTROL TASK ==============================
void controlTask(void *arg) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(LOOP_MS));
    controlStep();
  }
}

void controlStep() {
  uint32_t t0 = micros();
  if (lastStepUs) {
    uint32_t p = t0 - lastStepUs;
    if (p > periodMaxUs) periodMaxUs = p;
    if (p > LOOP_MS * 1100UL) overruns++;
  }
  lastStepUs = t0;

  if (pendingDlpf) { pendingDlpf = false; imuWrite(MPU_CONFIG, (uint8_t)constrain((int)cfg.dlpf, 0, 6)); }

  if (imuNeedInit && imuPresent()) { imuInitRegs(); imuNeedInit = false; evtImuInit = true; }
  bool imuOk = !imuNeedInit && imuRead();
  if (imuOk && AcX == 0 && AcY == 0 && AcZ == 0) imuOk = false;   // asleep / just reset: gravity is never 0
  if (imuOk) i2cFailRun = 0;
  else {
    static uint8_t retry = 0;
    i2cErrors++;
    if (i2cFailRun < 255) i2cFailRun++;
    if (i2cFailRun >= I2C_FAIL_LIMIT && retry++ % 33 == 0) imuNeedInit = true;  // re-wake every ~0.5 s
  }

  // Wheel speeds: the ISRs only ever add to encCount, so differences can't lose counts.
  for (int i = 0; i < 3; i++) {
    int32_t c = encCount[i];
    motor_speed[i] = (int16_t)(c - encLast[i]);
    encLast[i] = c;
  }
  threeWay_to_XY(motor_speed[0], motor_speed[1], motor_speed[2]);
  motors_speed_Z = motor_speed[0] + motor_speed[1] + motor_speed[2];

  if (imuOk) updateAngles();
  if (cubeJob != JOB_NONE) runJob(imuOk);
  updateMode();

  if (cubeMode == MODE_VERTEX)    runVertex();
  else if (cubeMode == MODE_EDGE) runEdge();
  else if (cubeJob != JOB_MTEST)  idleOutputs();

  if (atActive && cubeMode != MODE_IDLE) {
    atSumX += robot_angleX;
    atSumY += robot_angleY;
    if (++atN >= AUTOTRIM_SAMPLES) { atActive = false; atDone = true; }
  }

  loopCount++;
  uint32_t ex = micros() - t0;
  if (ex > execMaxUs) execMaxUs = ex;
}

void updateAngles() {
  // Lock the offsets to the position being balanced; only choose by AcX when idle.
  bool edgeSet = (cubeMode == MODE_EDGE) || (cubeMode == MODE_IDLE && abs(AcX) >= VERTEX_ACX_MAX);
  float ax, ay, az;
  if (edgeSet) { ax = AcX - cfg.acXe; ay = AcY - cfg.acYe; az = AcZ - cfg.acZe; }
  else         { ax = AcX - cfg.acXv; ay = AcY - cfg.acYv; az = AcZ - cfg.acZv; }

  Acc_angleY =  atan2f(ax, -az) * RAD2DEG;
  Acc_angleX = -atan2f(ay, -az) * RAD2DEG;
  if (edgeSet) Acc_angleX -= cfg.te;
  else { Acc_angleX -= cfg.tvx; Acc_angleY -= cfg.tvy; }

  float gx = GyXr - gyroOffX, gy = GyYr - gyroOffY, gz = GyZr - gyroOffZ;

  const float k = LOOP_S / GYRO_INTEG_LSB, a = cfg.gyroAmt;   // float: no truncation
  robot_angleY = (robot_angleY + gy * k) * a + Acc_angleY * (1.0f - a);
  robot_angleX = (robot_angleX + gx * k) * a + Acc_angleX * (1.0f - a);

  gyroX = gx / GYRO_LSB_DPS;
  gyroY = gy / GYRO_LSB_DPS;
  gyroZ = gz / GYRO_LSB_DPS;
  gyroXfilt = cfg.alpha * gyroX + (1.0f - cfg.alpha) * gyroXfilt;
  gyroYfilt = cfg.alpha * gyroY + (1.0f - cfg.alpha) * gyroYfilt;
}

void enterMode(CubeMode m) {
  robot_angleX = Acc_angleX;
  robot_angleY = Acc_angleY;
  motors_speed_X = 0;
  motors_speed_Y = 0;
  gyroXfilt = gyroX;
  gyroYfilt = gyroY;
  armedMode = m;
  cubeMode = m;
  evtArmed = true;
}

void updateMode() {
  // No balancing while a calibration/test is counting down or running.
  bool canRun = motorsEnabled && cubeJob == JOB_NONE && pendingJob == JOB_NONE &&
                i2cFailRun < I2C_FAIL_LIMIT;
  if (cubeMode == MODE_IDLE) {
    if (!canRun) return;
    int ax = abs(AcX);
    if (cfg.vertexCal && ax < VERTEX_ACX_MAX &&
        fabsf(Acc_angleX) < cfg.armV && fabsf(Acc_angleY) < cfg.armV) {
      enterMode(MODE_VERTEX);
    } else if (cfg.edgeCal && ax > EDGE_ACX_MIN && ax < EDGE_ACX_MAX &&
               ((AcX > 0) == (cfg.acXe > 0)) && fabsf(Acc_angleX) < cfg.armE) {
      enterMode(MODE_EDGE);
    }
  } else if (!canRun || fabsf(robot_angleX) > cfg.fall || fabsf(robot_angleY) > cfg.fall) {
    fallX = robot_angleX;
    fallY = robot_angleY;
    cubeMode = MODE_IDLE;
    evtFell = true;
    if (atActive) { atActive = false; atAborted = true; }
  }
}

void runVertex() {
  digitalWrite(BRAKE, HIGH);
  float pX = constrain(cfg.K1 * robot_angleX + cfg.K2 * gyroXfilt + cfg.K3 * speed_X + cfg.K4 * motors_speed_X, -255.0f, 255.0f);
  float pY = constrain(cfg.K1 * robot_angleY + cfg.K2 * gyroYfilt + cfg.K3 * speed_Y + cfg.K4 * motors_speed_Y, -255.0f, 255.0f);
  float pZ = constrain(cfg.zK2 * gyroZ + cfg.zK3 * motors_speed_Z, -255.0f, 255.0f);
  motors_speed_X += speed_X / 5.0f;
  motors_speed_Y += speed_Y / 5.0f;
  outX = pX; outY = pY; outZ = pZ;
  XYZ_to_threeWay(-pX, pY, -pZ);
}

void runEdge() {
  digitalWrite(BRAKE, HIGH);
  float p = constrain(cfg.eK1 * robot_angleX + cfg.eK2 * gyroXfilt + cfg.eK3 * motor_speed[2] + cfg.eK4 * motors_speed_X, -255.0f, 255.0f);
  motors_speed_X += motor_speed[2] / 5.0f;
  outX = p; outY = 0; outZ = 0;
  motorRaw(0, 0);
  motorRaw(1, 0);
  motorControl(2, (int)lroundf(p));
}

void idleOutputs() {
  for (int i = 0; i < 3; i++) motorRaw(i, 0);
  digitalWrite(BRAKE, LOW);
  motors_speed_X = 0;
  motors_speed_Y = 0;
  outX = 0; outY = 0; outZ = 0;
}

void runJob(bool imuOk) {
  if (jobDone) return;
  if (cubeJob == JOB_MTEST) {                  // per motor: 0.6 s drive, 0.3 s brake
    if (!motorsEnabled) { idleOutputs(); mtAborted = true; jobDone = true; return; }   // "stop" pressed
    int m = jobTick / 60, t = jobTick % 60;
    jobTick++;
    if (m >= 3) { idleOutputs(); jobDone = true; return; }
    if (t < 40) {
      digitalWrite(BRAKE, HIGH);
      for (int i = 0; i < 3; i++) motorRaw(i, i == m ? MTEST_PWM : 0);
      if (t == 20) mtStart = encCount[m];
      if (t == 39) mtResult[m] = encCount[m] - mtStart;
    } else {
      idleOutputs();
    }
    return;
  }
  if (!imuOk) return;                          // averaging jobs (gyro zero / balance points)
  int16_t g[3] = {GyXr, GyYr, GyZr};
  smp.sAx += AcX; smp.sAy += AcY; smp.sAz += AcZ;
  smp.sGx += g[0]; smp.sGy += g[1]; smp.sGz += g[2];
  for (int k = 0; k < 3; k++) {
    if (g[k] < smp.gMin[k]) smp.gMin[k] = g[k];
    if (g[k] > smp.gMax[k]) smp.gMax[k] = g[k];
  }
  float d = fmaxf(fabsf(gyroX), fmaxf(fabsf(gyroY), fabsf(gyroZ)));
  if (d > smp.maxDps) smp.maxDps = d;
  if (++smp.n >= (cubeJob == JOB_GYRO ? GYRO_SAMPLES : CAL_SAMPLES)) jobDone = true;
}

// Battery alarm + LED status (blue board LED patterns and optional RGB LEDs)
void handleHousekeeping() {
  static uint32_t tBatt = 0, tLed = 0, lastKey = 0xFFFFFFFF;
  static uint8_t pat = PAT_OFF;
  static bool lastLevel = false;
  uint32_t now = millis();
  if (now - tBatt >= 2000) {
    tBatt = now;
    vbat = readBattery();
    digitalWrite(BUZZER, (vbat > 8.0f && vbat <= 9.5f) ? HIGH : LOW);  // 3S low-battery alarm
  }

  if (now - tLed >= 100) {
    tLed = now;
    bool blink = (now / 250) & 1;
    int n = 0;                 // how many RGB LEDs are lit (0..3)
    uint32_t c = 0;            // RGB colour 0xRRGGBB
    pat = PAT_OFF;             // blue LED pattern
    CubeJob pj = pendingJob, cj = cubeJob;

    if (ledResultUntil && (int32_t)(now - ledResultUntil) >= 0) ledResultUntil = 0;

    if (ledResultUntil) {                                              // job result
      n = (ledResultOk || blink) ? 3 : 0;
      c = ledResultOk ? LED_GREEN : LED_RED;
      pat = ledResultOk ? PAT_ON : PAT_FAST;
    } else if (pj != JOB_NONE) {                                       // countdown
      int32_t left = (int32_t)(pendingAt - now);
      n = constrain((int)((left + 999) / 1000), 1, 3);                 // RGB: lit LEDs = seconds left
      if (pj == JOB_CAL_VERTEX || pj == JOB_CAL_EDGE) {
        bool ok = poseOk(pj);
        c = ok ? LED_GREEN : LED_RED;
        pat = ok ? PAT_ON : PAT_FAST;
      } else {
        c = LED_YELLOW;
        pat = PAT_SLOW;
      }
    } else if (cj == JOB_MTEST) {                                      // motor test
      n = 3; c = LED_CYAN; pat = PAT_SLOW;
    } else if (cj != JOB_NONE) {                                       // sampling
      float d = fmaxf(fabsf(gyroX), fmaxf(fabsf(gyroY), fabsf(gyroZ)));
      bool bad = d > (cj == JOB_GYRO ? 5.0f : CAL_MAX_DPS) || !poseOk(cj);
      n = 3; c = bad ? LED_RED : LED_YELLOW;
      pat = bad ? PAT_FAST : PAT_MED;
    } else if (!motorsEnabled) {                                       // stopped
      n = 3; c = LED_DIM_RED; pat = PAT_BLIP;
    } else if (cubeMode != MODE_IDLE) {                                // balancing
      n = 3; c = LED_DIM_GREEN; pat = PAT_ON;
    } else if (!cfg.vertexCal && !cfg.edgeCal) {                       // not calibrated
      n = ((now / 1000) & 1) ? 3 : 0; c = LED_PURPLE; pat = PAT_DOUBLE;
    } else {                                                           // aiming guide
      int ax = abs(AcX);
      float err = -1.0f;
      if (cfg.vertexCal && ax < VERTEX_ACX_MAX)
        err = fmaxf(fabsf(Acc_angleX), fabsf(Acc_angleY));
      else if (cfg.edgeCal && ax > EDGE_ACX_MIN && ax < EDGE_ACX_MAX && ((AcX > 0) == (cfg.acXe > 0)))
        err = fabsf(Acc_angleX);
      if (err >= 0.0f) {
        if (err < 1.0f)      { n = 3; c = LED_GREEN;  pat = PAT_ON; }
        else if (err < 2.5f) { n = 2; c = LED_YELLOW; pat = PAT_MED; }
        else if (err < 5.0f) { n = 1; c = LED_RED;    pat = PAT_SLOW; }
      }
    }

    uint32_t key = ((uint32_t)n << 24) | (c & 0xFFFFFF);
    if (key != lastKey) { lastKey = key; showLeds(n, c); }
  }

  bool lvl = patLevel(pat, now);
  if (lvl != lastLevel) { lastLevel = lvl; statusLed(lvl); }
}

// ============================== SETTINGS (FLASH) ==============================
uint32_t cfgCrc(const CubeConfig &c) {
  const uint8_t *p = (const uint8_t *)&c;
  uint32_t h = 2166136261UL;                          // FNV-1a
  for (size_t i = 0; i < offsetof(CubeConfig, crc); i++) { h ^= p[i]; h *= 16777619UL; }
  return h;
}

void cfgDefaults(CubeConfig &c, bool keepCalibration) {
  CubeConfig old = c;
  memset(&c, 0, sizeof(c));
  c.magic = CFG_MAGIC;
  c.K1 = 180.0f;  c.K2 = 30.0f;  c.K3 = 1.6f;  c.K4 = 0.008f;    // original gains
  c.zK2 = 8.0f;   c.zK3 = 0.30f;
  c.eK1 = 190.0f; c.eK2 = 31.0f; c.eK3 = 2.5f; c.eK4 = 0.014f;
  c.gyroAmt = 0.996f; c.alpha = 0.7f;
  c.armV = 0.4f; c.armE = 0.3f; c.fall = 7.0f;
  c.vbatDiv = 204.0f; c.dlpf = 0;
  if (keepCalibration) {
    c.acXv = old.acXv; c.acYv = old.acYv; c.acZv = old.acZv;
    c.acXe = old.acXe; c.acYe = old.acYe; c.acZe = old.acZe;
    c.vertexCal = old.vertexCal; c.edgeCal = old.edgeCal;
    c.tvx = old.tvx; c.tvy = old.tvy; c.te = old.te;
    c.vbatDiv = old.vbatDiv;
  }
}

// 2 = loaded, 1 = imported the original firmware's offsets, 0 = defaults
int cfgRead(CubeConfig &c) {
  CubeConfig t;
  EEPROM.get(0, t);
  if (t.magic == CFG_MAGIC && t.crc == cfgCrc(t)) { c = t; return 2; }
  LegacyOffsets lg;
  EEPROM.get(0, lg);
  cfgDefaults(c, false);
  if (lg.ID == 96) {
    c.acXv = lg.acXv; c.acYv = lg.acYv; c.acZv = lg.acZv;
    c.acXe = lg.acXe; c.acYe = lg.acYe; c.acZe = lg.acZe;
    c.vertexCal = 1; c.edgeCal = 1;
    return 1;
  }
  return 0;
}

void cfgWrite() {
  cfg.magic = CFG_MAGIC;
  cfg.crc = cfgCrc(cfg);
  EEPROM.put(0, cfg);
  EEPROM.commit();
  cfgDirty = false;
}

void cfgReplace(const CubeConfig &n) {               // never let the control task see half a copy
  vTaskSuspendAll();
  cfg = n;
  xTaskResumeAll();
  pendingDlpf = true;
}

// ============================== IMU ==============================
void imuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

bool imuPresent() {
  Wire.beginTransmission(MPU_ADDR);
  return Wire.endTransmission(true) == 0;
}

// Register setup without waits, also used by the control task to re-wake the sensor
void imuInitRegs() {
  imuWrite(MPU_PWR_MGMT_1, 0x01);       // wake, clock from the gyro PLL (more stable)
  imuWrite(MPU_SMPLRT_DIV, 0x00);
  imuWrite(MPU_CONFIG, (uint8_t)constrain((int)cfg.dlpf, 0, 6));
  imuWrite(MPU_GYRO_CFG, 0x00);         // +-250 deg/s
  imuWrite(MPU_ACCEL_CFG, 0x00);        // +-2 g
}

void imuConfigure() {
  imuInitRegs();
  delay(10);
  imuInitRegs();                        // again once the clock has settled
  delay(50);
}

// One 14-byte burst: accel, temperature, gyro from the same instant, error-checked
bool imuRead() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 14) != 14) return false;
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();
  AcX  = (int16_t)((b[0] << 8) | b[1]);
  AcY  = (int16_t)((b[2] << 8) | b[3]);
  AcZ  = (int16_t)((b[4] << 8) | b[5]);
  GyXr = (int16_t)((b[8] << 8) | b[9]);
  GyYr = (int16_t)((b[10] << 8) | b[11]);
  GyZr = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

// All three axes at once (the original did them one after another, ~7.7 s)
bool gyroCalibrateBlocking() {
  const int N = 500;
  int32_t s[3] = {0, 0, 0};
  int16_t lo[3] = {32767, 32767, 32767};
  int16_t hi[3] = {-32768, -32768, -32768};
  for (int i = 0; i < N; i++) {
    if (!imuRead()) return false;
    int16_t g[3] = {GyXr, GyYr, GyZr};
    for (int k = 0; k < 3; k++) {
      s[k] += g[k];
      if (g[k] < lo[k]) lo[k] = g[k];
      if (g[k] > hi[k]) hi[k] = g[k];
    }
    delay(2);
  }
  for (int k = 0; k < 3; k++) if (hi[k] - lo[k] > GYRO_STILL_RANGE) return false;
  gyroOffX = s[0] / (float)N;
  gyroOffY = s[1] / (float)N;
  gyroOffZ = s[2] / (float)N;
  return true;
}

// ============================== ENCODERS ==============================
void IRAM_ATTR enc1ISR() {
  static uint8_t s = 0;
  s = (uint8_t)(((s << 2) | (FAST_READ(ENC1_1) << 1) | FAST_READ(ENC1_2)) & 0x0F);
  encCount[0] = encCount[0] + QDEC[s];
}
void IRAM_ATTR enc2ISR() {
  static uint8_t s = 0;
  s = (uint8_t)(((s << 2) | (FAST_READ(ENC2_1) << 1) | FAST_READ(ENC2_2)) & 0x0F);
  encCount[1] = encCount[1] + QDEC[s];
}
void IRAM_ATTR enc3ISR() {
  static uint8_t s = 0;
  s = (uint8_t)(((s << 2) | (FAST_READ(ENC3_1) << 1) | FAST_READ(ENC3_2)) & 0x0F);
  encCount[2] = encCount[2] + QDEC[s];
}

void encodersInit() {
  pinMode(ENC1_1, INPUT); pinMode(ENC1_2, INPUT);
  pinMode(ENC2_1, INPUT); pinMode(ENC2_2, INPUT);
  pinMode(ENC3_1, INPUT); pinMode(ENC3_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1_1), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_2), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_1), enc2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_2), enc2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_1), enc3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_2), enc3ISR, CHANGE);
}

// ============================== MOTORS ==============================
const uint8_t DIR_PIN[3] = {DIR1, DIR2, DIR3};
const uint8_t PWM_PIN[3] = {PWM1, PWM2, PWM3};
const uint8_t PWM_CHN[3] = {PWM1_CH, PWM2_CH, PWM3_CH};

void motorPwm(int i, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PWM_PIN[i], duty);            // core 3.x: by pin
#else
  ledcWrite(PWM_CHN[i], duty);            // core 2.x: by channel
#endif
}

void motorsInit() {
  for (int i = 0; i < 3; i++) {
    pinMode(DIR_PIN[i], OUTPUT);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PWM_PIN[i], BASE_FREQ, TIMER_BIT);
#else
    ledcSetup(PWM_CHN[i], BASE_FREQ, TIMER_BIT);
    ledcAttachPin(PWM_PIN[i], PWM_CHN[i]);
#endif
    motorRaw(i, 0);
  }
}

// sp -255..255, positive = DIR HIGH. The Nidec 24H PWM input is active-low.
void motorRaw(int i, int sp) {
  sp = constrain(sp, -255, 255);
  digitalWrite(DIR_PIN[i], sp < 0 ? LOW : HIGH);
  motorPwm(i, (uint32_t)(255 - abs(sp)));
}

// Original MotorX_control(): adds the measured wheel speed, now clamped (no duty wrap-around)
void motorControl(int i, int sp) {
  motorRaw(i, sp + motor_speed[i]);
}

void XYZ_to_threeWay(float pwm_X, float pwm_Y, float pwm_Z) {
  motorControl(0, (int)lroundf((0.5f * pwm_X - 0.866f * pwm_Y) / 1.37f + pwm_Z));
  motorControl(1, (int)lroundf((0.5f * pwm_X + 0.866f * pwm_Y) / 1.37f + pwm_Z));
  motorControl(2, (int)lroundf(-pwm_X / 1.37f + pwm_Z));
}

void threeWay_to_XY(int s1, int s2, int s3) {
  speed_X = ((s3 - (s2 + s1) * 0.5f) * 0.5f) * 1.81f;
  speed_Y = (0.866f * (s2 - s1)) / 1.1f;
}

// ============================== HELPERS ==============================
void beep()  { digitalWrite(BUZZER, HIGH); delay(70); digitalWrite(BUZZER, LOW); delay(80); }
void chirp() { digitalWrite(BUZZER, HIGH); delay(25); digitalWrite(BUZZER, LOW); }

void setLeds(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_PIXELS; i++) leds[i] = CRGB(r, g, b);
  FastLED.show();
}

// Light the first n RGB LEDs in colour c (0xRRGGBB), the rest off
void showLeds(int n, uint32_t c) {
  for (int i = 0; i < NUM_PIXELS; i++)
    leds[i] = (i < n) ? CRGB((c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF) : CRGB(0, 0, 0);
  FastLED.show();
}

void statusLed(bool on) {
#if STATUS_LED >= 0
  digitalWrite(STATUS_LED, on ? HIGH : LOW);
#endif
}

// Blue LED blink patterns
bool patLevel(uint8_t p, uint32_t now) {
  switch (p) {
    case PAT_ON:     return true;
    case PAT_SLOW:   return (now % 1000) < 500;                    // 1 blink per second
    case PAT_MED:    return (now % 400) < 200;                     // 2.5 blinks per second
    case PAT_FAST:   return (now % 160) < 80;                      // fast flicker = problem
    case PAT_DOUBLE: { uint32_t t = now % 1400; return t < 120 || (t >= 260 && t < 380); }
    case PAT_BLIP:   return (now % 2000) < 60;
    default:         return false;
  }
}

// Is the cube in the orientation this calibration job needs?
bool poseOk(CubeJob j) {
  int ax = abs(AcX), ay = abs(AcY);
  if (j == JOB_CAL_VERTEX) return ax < VERTEX_ACX_MAX && ay < CAL_ACY_MAX;
  if (j == JOB_CAL_EDGE)   return ax > EDGE_ACX_MIN && ax < EDGE_ACX_MAX && ay < CAL_ACY_MAX;
  return true;
}

// 2 s result display: good = solid / green, bad = fast flicker / blinking red
void flashResult(bool ok) {
  ledResultOk = ok;
  ledResultUntil = millis() + 2000;
  if (!ledResultUntil) ledResultUntil = 1;
}

float readBattery() {
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(VBAT);
  return (sum / 8.0f) / cfg.vbatDiv;
}

void logAll(const char *fmt, ...) {
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.println(buf);
#if HAVE_BT
  if (SerialBT.hasClient()) SerialBT.println(buf);
#endif
}

// ============================== CONSOLE ==============================
TuneParam params[] = {
  {"K1",      &cfg.K1,      -2000, 2000,    5.0f,   "vertex: angle gain"},
  {"K2",      &cfg.K2,      -500,  500,     1.0f,   "vertex: tilt-rate gain"},
  {"K3",      &cfg.K3,      -50,   50,      0.1f,   "vertex: wheel-speed gain"},
  {"K4",      &cfg.K4,      -1,    1,       0.001f, "vertex: wheel-position gain"},
  {"zK2",     &cfg.zK2,     -100,  100,     0.5f,   "vertex: yaw-rate gain"},
  {"zK3",     &cfg.zK3,     -10,   10,      0.05f,  "vertex: yaw wheel-speed gain"},
  {"eK1",     &cfg.eK1,     -2000, 2000,    5.0f,   "edge: angle gain"},
  {"eK2",     &cfg.eK2,     -500,  500,     1.0f,   "edge: tilt-rate gain"},
  {"eK3",     &cfg.eK3,     -50,   50,      0.1f,   "edge: wheel-speed gain"},
  {"eK4",     &cfg.eK4,     -1,    1,       0.001f, "edge: wheel-position gain"},
  {"tvx",     &cfg.tvx,     -5,    5,       0.05f,  "vertex trim X, deg"},
  {"tvy",     &cfg.tvy,     -5,    5,       0.05f,  "vertex trim Y, deg"},
  {"te",      &cfg.te,      -5,    5,       0.05f,  "edge trim, deg"},
  {"gyroamt", &cfg.gyroAmt, 0.9f,  0.9999f, 0.001f, "complementary filter gyro weight"},
  {"alpha",   &cfg.alpha,   0.05f, 1,       0.05f,  "rate low-pass (1 = off)"},
  {"armv",    &cfg.armV,    0.1f,  5,       0.1f,   "vertex: start within this angle, deg"},
  {"arme",    &cfg.armE,    0.1f,  5,       0.1f,   "edge: start within this angle, deg"},
  {"fall",    &cfg.fall,    2,     20,      0.5f,   "stop above this tilt, deg"},
  {"vdiv",    &cfg.vbatDiv, 50,    1000,    1.0f,   "battery ADC counts per volt (see vcal)"},
  {"dlpf",    &cfg.dlpf,    0,     6,       1.0f,   "MPU6050 filter 0=260Hz 1=184 2=94 3=44 4=21"},
};
const int NPARAMS = sizeof(params) / sizeof(params[0]);

const char *const MENU[] = {
  "",
  "================== CUBE CONSOLE ==================",
  "Type a command and press Enter. Chain with ;  e.g.  K1 185; K2 31; save",
  "",
  "INFO",
  "  help [topic]       this menu, or details: help cal, help tel, help K1",
  "  help leds          what the blue LED blink patterns mean",
  "  status  (s)        mode, position, battery, angles, loop health",
  "  params  (p)        every tunable value with a description",
  "  tel [hz] | tel off live data for the Arduino Serial Plotter",
  "  watch (w) | w off  live position guide: what is in / out of range",
  "",
  "SETUP & CALIBRATION",
  "  mtest              test motors + encoders (cube lying on a side)",
  "  cal v [sec]        record the VERTEX balance point",
  "  cal e [sec]        record the EDGE balance point",
  "  autotrim           while balancing: fine-tune the balance point",
  "  gyrocal [sec]      re-zero the gyro (cube still)",
  "  vcal <volts>       match the battery reading to a multimeter",
  "  buzz               test the buzzer (active vs passive)",
  "",
  "TUNING",
  "  K1 185             set a value (also: set K1 185, K1=185)",
  "  K1+  K2--  eK3+++  nudge a value by its step size",
  "  names: K1 K2 K3 K4 zK2 zK3 | eK1 eK2 eK3 eK4 | tvx tvy te",
  "         gyroamt alpha | armv arme fall | vdiv dlpf",
  "",
  "MOTORS & SETTINGS",
  "  stop | start       brake + disable the wheels / re-enable",
  "  save | load        write to flash / undo unsaved changes",
  "  defaults           original gains back (keeps calibration)",
  "  factory            erase everything incl. calibration",
  "  reboot",
  "",
  "Quick start: mtest, cal v, cal e, save, balance it, autotrim (twice), save",
  "==================================================",
};

// help <topic> details. Parameters (help K1 etc.) are handled automatically.
const char *const TOPICS[][2] = {
  {"leds",
   "Blue LED on the ESP32 board (GPIO 2):\n"
   "  ON at power-up          zeroing the gyro - keep the cube still\n"
   "  double blink            not calibrated yet\n"
   "  cal countdown           ON = right position, FAST flicker = wrong position\n"
   "  sampling (~1.5 s)       steady blinking = hold still, FAST flicker = you moved\n"
   "  result (2 s)            ON = accepted, FAST flicker = rejected (console says why)\n"
   "  ready to balance        slow blink within 5 deg, quicker within 2.5, ON within 1\n"
   "  ON                      balancing\n"
   "  short blip every 2 s    motors stopped (type start)\n"
   "  slow blink              motor test, or gyrocal / mtest countdown\n"
   "Optional RGB LEDs on GPIO 19 show the same states in colour: green = good,\n"
   "  red = problem, yellow = sampling / getting close, purple = not calibrated,\n"
   "  cyan = motor test, dim green = balancing, dim red = stopped."},
  {"status",
   "status (s): one-screen health check.\n"
   "  Mode IDLE / VERTEX / EDGE, motors enabled or stopped, which position the cube is in,\n"
   "  battery volts, fused and accelerometer angles (balancing starts when the accel angle\n"
   "  is inside the start window), raw sensor values, wheel speeds and loop timing.\n"
   "  Worst period should stay near 15000 us; late and I2C errors should stay at 0."},
  {"params",
   "params (p): lists every tunable value, its current setting and what it does.\n"
   "  K1-K4 zK2 zK3 = vertex controller, eK1-eK4 = edge controller, tvx tvy te = trims,\n"
   "  gyroamt alpha = filters, armv arme fall = start/stop angles, vdiv dlpf = hardware.\n"
   "  help <name> shows one parameter with its range and step."},
  {"tel",
   "tel [hz] | tel off: streams label:value lines (open Tools > Serial Plotter).\n"
   "  m        mode: 0 idle, 1 vertex, 2 edge\n"
   "  ax ay    fused tilt angle, deg\n"
   "  gx gy    filtered tilt rate, deg/s\n"
   "  w1-w3    wheel speeds, encoder counts per 15 ms\n"
   "  px py pz controller output, -255..255\n"
   "  ix iy    wheel-position term; a steady ramp means the balance point is off -> autotrim\n"
   "  Default 20 Hz, max 66. Over Bluetooth keep it at 20 Hz or less."},
  {"watch",
   "watch [hz] (w) | watch off: prints a readable line (default 4 Hz) showing what the cube checks.\n"
   "  raw X Y Z   accelerometer, 16384 = 1 g\n"
   "  During a cal countdown/sampling: the raw windows the blue LED uses\n"
   "    vertex: |X| < 2000 and |Y| < 2000     edge: |X| 7000..10000 and |Y| < 2000\n"
   "    and the rotation rate (must stay under 15 deg/s while sampling).\n"
   "  Otherwise: the tilt from the calibrated balance point. Balancing starts when every\n"
   "    angle is inside armv (vertex) / arme (edge); 'off' marks the axis to correct.\n"
   "    Tilt slowly and watch which way the number moves toward 0.\n"
   "  Blue LED aiming guide: slow blink < 5 deg, quicker < 2.5 deg, ON < 1 deg."},
  {"mtest",
   "mtest: lay the cube on a side first. Spins each wheel for 0.6 s and counts encoder pulses.\n"
   "  OK            encoder agrees with the drive direction\n"
   "  backwards     swap ENCx_1 and ENCx_2 for that motor in the PINS section\n"
   "  not spinning  check battery, BRAKE/DIR/PWM wiring and encoder wiring\n"
   "  Takes about 3 s. Blue LED blinks slowly while it runs, then ON (passed) or flickers."},
  {"cal",
   "cal v [sec] / cal e [sec]: records the balance point of the vertex or the edge.\n"
   "  Countdown (default 3 s): blue LED ON = right position, FAST flicker = wrong position.\n"
   "  Stand the cube on its corner (v) or edge (e), hold it lightly where it would not fall\n"
   "  either way. While sampling (~1.5 s) the LED blinks steadily - hold still.\n"
   "  Result: LED ON for 2 s = accepted, FAST flicker = rejected (console says why). Then: save"},
  {"autotrim",
   "autotrim: run while the cube is balancing on its own. Averages the lean for ~9 s and moves\n"
   "  the trim (tvx/tvy on the vertex, te on the edge) so the controller zero matches the real\n"
   "  balance point. Run it twice, then save. Aborts if the cube stops balancing."},
  {"gyrocal",
   "gyrocal [sec]: re-measures the gyro zero (also done automatically at power-up).\n"
   "  Leave the cube still on the table; rejected if it moves. Use it if the angle in status\n"
   "  drifts while the cube sits still, or after the electronics have warmed up."},
  {"vcal",
   "vcal <volts>: measure the battery with a multimeter and type the reading, e.g. vcal 11.8.\n"
   "  Sets vdiv so the battery value and low-battery alarm are correct. Then save.\n"
   "  The buzzer sounds continuously when the battery reads between 8 and 9.5 V."},
  {"buzz",
   "buzz: two-part buzzer test.\n"
   "  Test 1 turns the buzzer on steadily for 1 s - an ACTIVE buzzer beeps here.\n"
   "  Test 2 plays a 2 kHz tone for 1 s - a PASSIVE buzzer beeps here.\n"
   "  Beep in test 1: buzzer OK. Only test 2: passive buzzer (project needs an active 5 V one).\n"
   "  Silent in both: check buzzer polarity, 5 V supply, and the transistor on GPIO 27."},
  {"stop",
   "stop / start: stop brakes the wheels and keeps them off until start.\n"
   "  Handy while tuning on the bench. Settings and calibration are not affected."},
  {"save",
   "save / load: changes take effect immediately but are lost on reboot unless you save.\n"
   "  load re-reads flash, undoing anything not yet saved."},
  {"set",
   "set <param> <value>: same as <param> <value> or <param>=<value>. Values outside the\n"
   "  allowed range are limited. Nudge with <param>+ or <param>- (repeat signs for more steps)."},
  {"defaults",
   "defaults: original-firmware gains, filters and start/stop angles.\n"
   "  Keeps calibration, trims and vdiv. Save if you want to keep it."},
  {"factory",
   "factory: erases everything including calibration and saves immediately.\n"
   "  The cube must be lying down."},
  {"reboot",
   "reboot: restarts the ESP32. Keep the cube still afterwards while the gyro is zeroed."},
};

bool cmdIs(const char *a, const char *b) { return strcasecmp(a, b) == 0; }

float toFloat(const char *s) {
  char *end;
  float v = strtof(s, &end);
  return end == s ? NAN : v;
}

TuneParam *findParam(const char *name) {
  for (int i = 0; i < NPARAMS; i++)
    if (strcasecmp(name, params[i].name) == 0) return &params[i];
  return nullptr;
}

void printParam(const TuneParam *p, Print &out) {
  out.printf("  %-8s %10.5g   %s\n", p->name, *p->ptr, p->help);
}

void applyParam(TuneParam *p, float v, Print &out) {
  if (isnan(v)) { out.println("Not a number."); return; }
  if (v < p->minV || v > p->maxV) {
    v = constrain(v, p->minV, p->maxV);
    out.printf("  (limited to %g..%g)\n", p->minV, p->maxV);
  }
  if (p->ptr == &cfg.dlpf) { v = roundf(v); pendingDlpf = true; }
  *p->ptr = v;
  cfgDirty = true;
  printParam(p, out);
}

// "K1 200", "K1", "K1+", "K2---"
bool paramShortcut(int argc, char **argv, Print &out) {
  char name[24];
  strncpy(name, argv[0], sizeof(name) - 1);
  name[sizeof(name) - 1] = 0;
  int n = strlen(name), steps = 0;
  while (n > 0 && (name[n - 1] == '+' || name[n - 1] == '-')) {
    steps += name[n - 1] == '+' ? 1 : -1;
    name[--n] = 0;
  }
  TuneParam *p = findParam(name);
  if (!p) return false;
  if (steps) applyParam(p, *p->ptr + steps * p->step, out);
  else if (argc >= 2) applyParam(p, toFloat(argv[1]), out);
  else printParam(p, out);
  return true;
}

void printHelp(Print &out, const char *topic) {
  if (!topic) {
    for (const char *line : MENU) out.println(line);
    return;
  }
  if (cmdIs(topic, "s")) topic = "status";            // aliases
  else if (cmdIs(topic, "p")) topic = "params";
  else if (cmdIs(topic, "load")) topic = "save";
  else if (cmdIs(topic, "start")) topic = "stop";
  else if (cmdIs(topic, "led")) topic = "leds";

  TuneParam *p = findParam(topic);
  if (p) {
    out.printf("%s = %.5g   (range %g .. %g, step %g)\n", p->name, *p->ptr, p->minV, p->maxV, p->step);
    out.printf("  %s\n  Set: %s <value>   Nudge: %s+ or %s-\n", p->help, p->name, p->name, p->name);
    return;
  }
  for (auto &t : TOPICS) {
    if (cmdIs(topic, t[0])) { out.println(t[1]); return; }
  }
  out.printf("No help for '%s'. Type help for the menu.\n", topic);
}

void printStatus(Print &out) {
  static const char *const MODE_NAME[] = {"IDLE", "VERTEX", "EDGE"};
  int ax = abs(AcX);
  const char *pose = ax < VERTEX_ACX_MAX ? "near vertex"
                   : (ax > EDGE_ACX_MIN && ax < EDGE_ACX_MAX) ? "near edge" : "lying down";
  out.printf("Mode %s | motors %s | position %s | battery %.2f V\n",
             MODE_NAME[cubeMode], motorsEnabled ? "enabled" : "STOPPED", pose, vbat);
  out.printf("Calibrated: vertex %s, edge %s%s\n", cfg.vertexCal ? "yes" : "NO",
             cfg.edgeCal ? "yes" : "NO", cfgDirty ? "   (unsaved changes)" : "");
  out.printf("Angle X %.2f Y %.2f | accel angle X %.2f Y %.2f (starts within %.1f)\n",
             robot_angleX, robot_angleY, Acc_angleX, Acc_angleY,
             ax < VERTEX_ACX_MAX ? cfg.armV : cfg.armE);
  out.printf("Raw accel %d %d %d | gyro %.2f %.2f %.2f deg/s | gyro zero %.1f %.1f %.1f\n",
             AcX, AcY, AcZ, gyroX, gyroY, gyroZ, gyroOffX, gyroOffY, gyroOffZ);
  out.printf("Wheels %d %d %d counts/loop | encoder totals %ld %ld %ld\n",
             motor_speed[0], motor_speed[1], motor_speed[2],
             (long)encCount[0], (long)encCount[1], (long)encCount[2]);
  out.printf("Loop: %lu runs, worst period %lu us, worst compute %lu us, late %lu, I2C errors %lu\n",
             (unsigned long)loopCount, (unsigned long)periodMaxUs, (unsigned long)execMaxUs,
             (unsigned long)overruns, (unsigned long)i2cErrors);
  periodMaxUs = 0;        // worst-case values are "since the last status"
  execMaxUs = 0;
}

void scheduleJob(CubeJob j, int secs, Print &out) {
  if (cubeJob != JOB_NONE || pendingJob != JOB_NONE) { out.println("Busy - wait for the current job."); return; }
  static const char *const WHAT[] = {
    "",
    "Gyro zero: keep the cube completely still.",
    "Vertex: stand the cube on its corner at the balance point, hold it lightly. Blue LED ON = right position.",
    "Edge: stand the cube on its edge at the balance point, hold it lightly. Blue LED ON = right position.",
    "Motor test: lay the cube on a side so the wheels can spin freely."};
  out.println(WHAT[j]);
  secs = constrain(secs, 0, 30);
  if (secs) out.printf("Starting in %d s...\n", secs);
  ledResultUntil = 0;
  pendingAt = millis() + secs * 1000UL;
  nextChirp = millis();
  pendingJob = j;
}

void startJob(CubeJob j) {
  memset(&smp, 0, sizeof(smp));
  for (int k = 0; k < 3; k++) { smp.gMin[k] = 32767; smp.gMax[k] = -32768; }
  jobTick = 0;
  jobDone = false;
  cubeJob = j;                                  // the control task picks it up next cycle
  logAll(j == JOB_MTEST ? "Motor test running (~3 s)..." : "Sampling - hold still...");
}

void handleCountdown() {
  if (pendingJob == JOB_NONE) return;
  uint32_t now = millis();
  if ((int32_t)(now - pendingAt) >= 0) {
    CubeJob j = pendingJob;
    startJob(j);                                // set cubeJob before clearing pendingJob,
    pendingJob = JOB_NONE;                      // so the motors never get a gap to start
  } else if ((int32_t)(now - nextChirp) >= 0) {
    chirp();
    nextChirp += 1000;
  }
}

void finishGyroCal() {
  for (int k = 0; k < 3; k++) {
    if (smp.gMax[k] - smp.gMin[k] > GYRO_STILL_RANGE) {
      logAll("Gyro zero rejected: the cube moved. Lay it still and try again.");
      flashResult(false);
      return;
    }
  }
  float n = smp.n;
  gyroOffX = smp.sGx / n; gyroOffY = smp.sGy / n; gyroOffZ = smp.sGz / n;
  logAll("Gyro offsets %.1f %.1f %.1f", gyroOffX, gyroOffY, gyroOffZ);
  flashResult(true);
}

void finishBalanceCal(bool edge) {
  float n = smp.n;
  float ax = smp.sAx / n, ay = smp.sAy / n, az = smp.sAz / n;
  logAll("Average accel X %.0f Y %.0f Z %.0f (max rotation %.1f deg/s)", ax, ay, az, smp.maxDps);
  if (smp.maxDps > CAL_MAX_DPS) {
    logAll("Rejected: the cube moved too much. Hold it steadier and retry.");
    flashResult(false);
    return;
  }
  if (!edge) {
    if (fabsf(ax) >= VERTEX_ACX_MAX || fabsf(ay) >= CAL_ACY_MAX) {
      logAll("Rejected: not standing on the vertex (|X| and |Y| must be < %d).", VERTEX_ACX_MAX);
      flashResult(false);
      return;
    }
    vTaskSuspendAll();
    cfg.acXv = ax; cfg.acYv = ay; cfg.acZv = az + ACC_1G;
    cfg.tvx = 0; cfg.tvy = 0; cfg.vertexCal = 1;
    xTaskResumeAll();
    logAll("Vertex calibrated. Type save to keep it.");
  } else {
    if (fabsf(ax) <= EDGE_ACX_MIN || fabsf(ax) >= EDGE_ACX_MAX || fabsf(ay) >= CAL_ACY_MAX) {
      logAll("Rejected: not standing on the edge (|X| must be %d..%d, |Y| < %d).",
             EDGE_ACX_MIN, EDGE_ACX_MAX, CAL_ACY_MAX);
      flashResult(false);
      return;
    }
    vTaskSuspendAll();
    cfg.acXe = ax; cfg.acYe = ay; cfg.acZe = az + ACC_1G;
    cfg.te = 0; cfg.edgeCal = 1;
    xTaskResumeAll();
    logAll("Edge calibrated. Type save to keep it.");
  }
  cfgDirty = true;
  flashResult(true);
}

void finishMotorTest() {
  bool allOk = true;
  for (int m = 0; m < 3; m++) {
    long r = mtResult[m];
    if (r >= MTEST_OK_COUNTS) {
      logAll("Motor %d: %ld counts - OK", m + 1, r);
    } else if (r <= -MTEST_REV_COUNTS) {
      allOk = false;
      logAll("Motor %d: %ld counts - encoder counts backwards: swap ENC%d_1 and ENC%d_2 in the PINS section",
             m + 1, r, m + 1, m + 1);
    } else {
      allOk = false;
      logAll("Motor %d: %ld counts - not spinning or no encoder signal: check power, BRAKE/DIR/PWM and encoder wiring",
             m + 1, r);
    }
  }
  if (allOk) logAll("Motor test passed.");
  flashResult(allOk);
}

void handleJobResults() {
  if (cubeJob == JOB_NONE || !jobDone) return;
  CubeJob j = cubeJob;
  if (j == JOB_GYRO) finishGyroCal();
  else if (j == JOB_MTEST && mtAborted) { mtAborted = false; logAll("Motor test aborted (motors stopped)."); flashResult(false); }
  else if (j == JOB_MTEST) finishMotorTest();
  else finishBalanceCal(j == JOB_CAL_EDGE);
  cubeJob = JOB_NONE;
  jobDone = false;
  beep(); beep();
}

void handleEvents() {
  if (evtArmed) { evtArmed = false; logAll("Balancing on %s.", armedMode == MODE_VERTEX ? "VERTEX" : "EDGE"); }
  if (evtFell)  { evtFell = false;  logAll("Stopped balancing (X %.1f, Y %.1f deg).", fallX, fallY); }
  if (atAborted) {
    atAborted = false;
    logAll("autotrim aborted - the cube stopped balancing.");
    flashResult(false);
  }
  if (atDone) {
    atDone = false;
    float mx = atSumX / atN, my = atSumY / atN;
    if (atMode == MODE_VERTEX) {
      cfg.tvx = constrain(cfg.tvx + mx, -5.0f, 5.0f);
      cfg.tvy = constrain(cfg.tvy + my, -5.0f, 5.0f);
      logAll("autotrim: average lean X %.2f Y %.2f -> tvx %.2f tvy %.2f. Repeat once, then save.",
             mx, my, cfg.tvx, cfg.tvy);
    } else {
      cfg.te = constrain(cfg.te + mx, -5.0f, 5.0f);
      logAll("autotrim: average lean %.2f -> te %.2f. Repeat once, then save.", mx, cfg.te);
    }
    cfgDirty = true;
    flashResult(true);
  }
  if (evtImuInit) {
    evtImuInit = false;
    logAll("MPU6050 found - woken up and configured.");
    if (gyroOffX == 0 && gyroOffY == 0 && gyroOffZ == 0)
      logAll("The gyro was never zeroed: keep the cube still and type gyrocal (or reboot).");
  }
  static bool imuFault = false;
  bool f = i2cFailRun >= I2C_FAIL_LIMIT;
  if (f != imuFault) {
    imuFault = f;
    logAll(f ? "IMU read errors - motors stopped. Check the MPU6050 wiring." : "IMU reads OK again.");
  }
#if HAVE_BT
  static bool btWasConnected = false;
  static uint32_t btWelcomeAt = 0;
  bool btNow = SerialBT.hasClient();
  if (btNow && !btWasConnected) btWelcomeAt = millis() + 500;   // give the phone app a moment
  btWasConnected = btNow;
  if (!btNow) btWelcomeAt = 0;
  if (btWelcomeAt && (int32_t)(millis() - btWelcomeAt) >= 0) {
    btWelcomeAt = 0;
    SerialBT.printf("Connected to the cube. Battery %.2f V, vertex %s, edge %s.\n",
                    vbat, cfg.vertexCal ? "calibrated" : "NOT calibrated",
                    cfg.edgeCal ? "calibrated" : "NOT calibrated");
    printHelp(SerialBT, nullptr);
  }
#endif
}

void handleTelemetry() {
  if (!telOut || !telPeriod) return;
  uint32_t now = millis();
  if ((int32_t)(now - telNext) < 0) return;
  telNext = now + telPeriod;
#if HAVE_BT
  if (telOut == &SerialBT && !SerialBT.hasClient()) return;
#endif
  char b[200];
  snprintf(b, sizeof(b), "m:%d,ax:%.2f,ay:%.2f,gx:%.1f,gy:%.1f,w1:%d,w2:%d,w3:%d,px:%.0f,py:%.0f,pz:%.0f,ix:%.0f,iy:%.0f",
           (int)cubeMode, robot_angleX, robot_angleY, gyroXfilt, gyroYfilt,
           motor_speed[0], motor_speed[1], motor_speed[2], outX, outY, outZ,
           motors_speed_X, motors_speed_Y);
  telOut->println(b);
}

// "1234 ok" / "1234 HIGH (need <2000)" / "500 LOW (need >7000)"; lo < 0 = no lower limit
static const char *winTxt(char *o, size_t sz, int v, int lo, int hi) {
  if (lo >= 0 && v <= lo) snprintf(o, sz, "%d LOW (need >%d)", v, lo);
  else if (v >= hi)       snprintf(o, sz, "%d HIGH (need <%d)", v, hi);
  else                    snprintf(o, sz, "%d ok", v);
  return o;
}

static const char *angTxt(char *o, size_t sz, float a, float lim) {
  snprintf(o, sz, "%+.2f %s", a, fabsf(a) < lim ? "ok" : "off");
  return o;
}

void handleWatch() {
  if (!watchOut || !watchPeriod) return;
  uint32_t now = millis();
  if ((int32_t)(now - watchNext) < 0) return;
  watchNext = now + watchPeriod;
#if HAVE_BT
  if (watchOut == &SerialBT && !SerialBT.hasClient()) return;
#endif
  char b[240], s1[40], s2[40];
  int ax = abs(AcX), ay = abs(AcY);
  float rot = fmaxf(fabsf(gyroX), fmaxf(fabsf(gyroY), fabsf(gyroZ)));
  int n = snprintf(b, sizeof(b), "raw X %6d Y %6d Z %6d | ", AcX, AcY, AcZ);
  size_t r = sizeof(b) - n;
  CubeJob pj = pendingJob, cj = cubeJob;
  CubeJob j = pj != JOB_NONE ? pj : cj;

  if (j == JOB_CAL_VERTEX || j == JOB_CAL_EDGE) {
    bool edge = j == JOB_CAL_EDGE;
    snprintf(b + n, r, "%s %s: |X| %s, |Y| %s, rot %.1f deg/s%s",
             pj != JOB_NONE ? "COUNTDOWN" : "SAMPLING", edge ? "edge" : "vertex",
             edge ? winTxt(s1, sizeof(s1), ax, EDGE_ACX_MIN, EDGE_ACX_MAX)
                  : winTxt(s1, sizeof(s1), ax, -1, VERTEX_ACX_MAX),
             winTxt(s2, sizeof(s2), ay, -1, CAL_ACY_MAX),
             rot, rot > CAL_MAX_DPS ? " TOO FAST" : "");
  } else if (cubeMode != MODE_IDLE) {
    snprintf(b + n, r, "BALANCING %s: angle X %+.2f Y %+.2f (stops beyond %.1f)",
             cubeMode == MODE_VERTEX ? "vertex" : "edge", robot_angleX, robot_angleY, cfg.fall);
  } else if (i2cFailRun >= I2C_FAIL_LIMIT) {
    snprintf(b + n, r, "IMU read errors - check the MPU6050 wiring");
  } else if (ax < VERTEX_ACX_MAX) {
    if (!cfg.vertexCal) snprintf(b + n, r, "near vertex, NOT calibrated - run cal v");
    else snprintf(b + n, r, "vertex tilt X %s, Y %s (starts when both < %.2f) rot %.1f",
                  angTxt(s1, sizeof(s1), Acc_angleX, cfg.armV),
                  angTxt(s2, sizeof(s2), Acc_angleY, cfg.armV), cfg.armV, rot);
  } else if (ax > EDGE_ACX_MIN && ax < EDGE_ACX_MAX) {
    if (!cfg.edgeCal) snprintf(b + n, r, "near edge, NOT calibrated - run cal e");
    else if ((AcX > 0) != (cfg.acXe > 0))
      snprintf(b + n, r, "edge on the WRONG side: raw X must be %s like at cal e",
               cfg.acXe > 0 ? "positive" : "negative");
    else snprintf(b + n, r, "edge tilt X %s (starts when < %.2f) rot %.1f",
                  angTxt(s1, sizeof(s1), Acc_angleX, cfg.armE), cfg.armE, rot);
  } else {
    snprintf(b + n, r, "lying down: vertex needs |X| < %d, edge needs |X| %d..%d",
             VERTEX_ACX_MAX, EDGE_ACX_MIN, EDGE_ACX_MAX);
  }
  if (!motorsEnabled) {
    size_t len = strlen(b);
    snprintf(b + len, sizeof(b) - len, " [motors STOPPED - type start]");
  }
  watchOut->println(b);
}

void runLine(LineBuf &lb, Print &out) {
  if (!lb.len) return;
  lb.buf[lb.len] = 0;
  lb.len = 0;
  char *save = nullptr;
  for (char *cmd = strtok_r(lb.buf, ";", &save); cmd; cmd = strtok_r(nullptr, ";", &save))
    handleCommand(cmd, out);
}

// idleFlush: run a line after 300 ms of silence (phone apps that send no newline)
void pollStream(Stream &in, Print &out, LineBuf &lb, bool idleFlush) {
  while (in.available() > 0) {
    int ch = in.read();
    if (ch < 0) break;
    lb.lastMs = millis();
    if (ch == '\n' || ch == '\r') runLine(lb, out);
    else if (lb.len < sizeof(lb.buf) - 1) lb.buf[lb.len++] = (char)ch;
  }
  if (idleFlush && lb.len && millis() - lb.lastMs > 300) runLine(lb, out);
}

void handleCommand(char *line, Print &out) {
  char *argv[4];
  int argc = 0;
  char *save = nullptr;
  for (char *t = strtok_r(line, " \t=", &save); t && argc < 4; t = strtok_r(nullptr, " \t=", &save))
    argv[argc++] = t;
  if (argc == 0) return;
  const char *c = argv[0];

  if (cmdIs(c, "help") || cmdIs(c, "?") || cmdIs(c, "menu")) {
    printHelp(out, argc >= 2 ? argv[1] : nullptr);
  } else if (cmdIs(c, "status") || cmdIs(c, "s")) {
    printStatus(out);
  } else if (cmdIs(c, "params") || cmdIs(c, "p")) {
    for (int i = 0; i < NPARAMS; i++) printParam(&params[i], out);
    if (cfgDirty) out.println("  (unsaved changes - type save)");
  } else if (cmdIs(c, "set")) {
    TuneParam *p = argc >= 3 ? findParam(argv[1]) : nullptr;
    if (p) applyParam(p, toFloat(argv[2]), out);
    else out.println("Usage: set <param> <value>  (params lists them)");
  } else if (cmdIs(c, "save")) {
    cfgWrite();
    out.println("Saved.");
    flashResult(true);
  } else if (cmdIs(c, "load")) {
    CubeConfig t = cfg;
    int r = cfgRead(t);
    cfgReplace(t);
    cfgDirty = (r != 2);
    out.println(r == 2 ? "Loaded saved settings." : r == 1 ? "Imported original-firmware calibration."
                                                           : "Nothing saved - defaults loaded.");
  } else if (cmdIs(c, "defaults")) {
    CubeConfig t = cfg;
    cfgDefaults(t, true);
    cfgReplace(t);
    cfgDirty = true;
    out.println("Gains, filters and limits reset; calibration kept. Type save to keep.");
  } else if (cmdIs(c, "factory")) {
    if (cubeMode != MODE_IDLE) { out.println("Lay the cube down first."); return; }
    CubeConfig t = {};
    cfgDefaults(t, false);
    cfgReplace(t);
    cfgWrite();
    out.println("Everything erased. Calibrate again with cal v / cal e.");
  } else if (cmdIs(c, "cal")) {
    char w = argc >= 2 ? (char)tolower(argv[1][0]) : 0;
    if (w != 'v' && w != 'e') { out.println("Usage: cal v | cal e  [countdown seconds]"); return; }
    scheduleJob(w == 'v' ? JOB_CAL_VERTEX : JOB_CAL_EDGE, argc >= 3 ? atoi(argv[2]) : 3, out);
  } else if (cmdIs(c, "gyrocal")) {
    scheduleJob(JOB_GYRO, argc >= 2 ? atoi(argv[1]) : 1, out);
  } else if (cmdIs(c, "mtest")) {
    if (cubeMode != MODE_IDLE) { out.println("Lay the cube on a side first."); return; }
    if (!motorsEnabled) { out.println("Motors are stopped - type start first."); return; }
    scheduleJob(JOB_MTEST, 2, out);
  } else if (cmdIs(c, "autotrim")) {
    if (cubeMode == MODE_IDLE) { out.println("Get it balancing first, then run autotrim."); return; }
    if (atActive) { out.println("autotrim is already running."); return; }
    atSumX = 0; atSumY = 0; atN = 0; atMode = cubeMode;
    atDone = false;
    atAborted = false;
    atActive = true;
    out.println("Measuring the balance point for ~9 s - don't touch the cube...");
  } else if (cmdIs(c, "stop")) {
    motorsEnabled = false;
    pendingJob = JOB_NONE;
    out.println("Motors disabled (start to enable).");
  } else if (cmdIs(c, "start")) {
    motorsEnabled = true;
    out.println("Motors enabled - hold the cube at its balance point to start.");
  } else if (cmdIs(c, "tel")) {
    int hz = argc >= 2 ? atoi(argv[1]) : 20;
    if (hz <= 0) { telOut = nullptr; telPeriod = 0; out.println("Telemetry off."); return; }
    hz = constrain(hz, 1, 66);
    telPeriod = 1000 / hz; telNext = millis(); telOut = &out;
    watchOut = nullptr; watchPeriod = 0;               // one live stream at a time
    out.printf("Telemetry %d Hz (help tel explains the fields, tel off to stop)\n", hz);
  } else if (cmdIs(c, "watch") || cmdIs(c, "w")) {
    int hz = argc >= 2 ? atoi(argv[1]) : 4;            // "watch off" -> atoi = 0
    if (hz <= 0) { watchOut = nullptr; watchPeriod = 0; out.println("Watch off."); return; }
    hz = constrain(hz, 1, 10);
    watchPeriod = 1000 / hz; watchNext = millis(); watchOut = &out;
    telOut = nullptr; telPeriod = 0;
    out.printf("Watch %d Hz (help watch explains it, w off to stop)\n", hz);
  } else if (cmdIs(c, "vcal")) {
    float v = argc >= 2 ? toFloat(argv[1]) : NAN;
    if (!(v > 5.0f && v < 30.0f)) { out.println("Usage: vcal <volts from a multimeter>, e.g. vcal 11.8"); return; }
    uint32_t sum = 0;
    for (int i = 0; i < 64; i++) sum += analogRead(VBAT);
    float div = (sum / 64.0f) / v;
    TuneParam *vp = findParam("vdiv");
    if (div < vp->minV || div > vp->maxV) {
      out.printf("Rejected: ADC reads %.0f counts for %.2f V (vdiv %.1f, allowed %g..%g). Check the battery divider on GPIO %d.\n",
                 sum / 64.0f, v, div, vp->minV, vp->maxV, VBAT);
      return;
    }
    cfg.vbatDiv = div;
    vbat = v;
    cfgDirty = true;
    out.printf("vdiv = %.1f. Type save to keep.\n", cfg.vbatDiv);
  } else if (cmdIs(c, "buzz")) {
    out.println("Buzzer test 1: steady ON for 1 s (an ACTIVE buzzer beeps here)");
    digitalWrite(BUZZER, HIGH); delay(1000); digitalWrite(BUZZER, LOW);
    delay(500);
    out.println("Buzzer test 2: 2 kHz tone for 1 s (a PASSIVE buzzer beeps here)");
    for (int i = 0; i < 2000; i++) {
      digitalWrite(BUZZER, HIGH); delayMicroseconds(250);
      digitalWrite(BUZZER, LOW);  delayMicroseconds(250);
    }
    out.println("Buzzer test done. Type help buzz to read the result.");
  } else if (cmdIs(c, "reboot")) {
    out.println("Rebooting...");
    delay(100);
    ESP.restart();
  } else if (!paramShortcut(argc, argv, out)) {
    out.printf("Unknown command '%s' - type help\n", c);
  }
}
