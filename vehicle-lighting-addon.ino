// File: src/main.cpp
// Build target: ESP8266 (Arduino core). No external libs required.

// -------- Includes
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

// -------- Pins (Wemos D1 mini / NodeMCU typical)
#ifndef SDA_PIN
#define SDA_PIN D2
#endif
#ifndef SCL_PIN
#define SCL_PIN D1
#endif
#ifndef LED_PWM_PIN
#define LED_PWM_PIN D6  // PWM-capable
#endif

// -------- ADC config
// If your dev board's A0 expects 0..3.3V, set to true. Bare ESP8266 is ~0..1.0V.
static const bool ADC_RANGE_33V = true;

// -------- ADXL345 Registers
namespace ADXL345 {
  const uint8_t I2C_ADDR     = 0x53;
  const uint8_t REG_DEVID    = 0x00;
  const uint8_t REG_BW_RATE  = 0x2C;
  const uint8_t REG_POWERCTL = 0x2D; // MEASURE=bit3
  const uint8_t REG_DATAFMT  = 0x31; // FULL_RES=bit3, Range=bits[1:0]
  const uint8_t REG_DATAX0   = 0x32; // to 0x37
  const uint8_t DEVID_VALUE  = 0xE5;
}

// -------- Utility
static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
  // Simple CRC-32 (poly 0xEDB88320)
  crc = ~crc;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int k = 0; k < 8; ++k) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

// -------- Config persisted in EEPROM
struct Config {
  float minDecel_g;        // threshold to start lighting
  float maxDecel_g;        // at/above this = 100% (before ambient scaling)
  float gamma_curve;       // perceptual curve
  float pwm_floor_day;     // min duty in bright conditions [0..1]
  float pwm_floor_night;   // min duty in dark conditions [0..1]
  float ambient_min;       // min ambient scale
  float ambient_max;       // max ambient scale
  float jerk_g_per_s;      // max change rate for decel filter (g/s)
  float slew_per_s;        // max duty rate [0..1]/s
  float ldr_ema_tau_s;     // ambient EMA time constant
  float grav_tau_s;        // gravity LPF time constant
  uint32_t crc;
};

static Config cfg;
static const Config kDefaults = {
  /*minDecel_g*/      0.03f,   // ~0.3 m/s^2
  /*maxDecel_g*/      0.50f,   // ~4.9 m/s^2 hard brake
  /*gamma_curve*/     2.2f,
  /*pwm_floor_day*/   0.18f,
  /*pwm_floor_night*/ 0.10f,
  /*ambient_min*/     0.35f,
  /*ambient_max*/     1.00f,
  /*jerk_g_per_s*/    1.50f,
  /*slew_per_s*/      1.50f,
  /*ldr_ema_tau_s*/   3.0f,
  /*grav_tau_s*/      1.2f,
  /*crc*/             0
};

static void saveConfig() {
  Config tmp = cfg;
  tmp.crc = 0;
  uint32_t c = crc32_update(0, reinterpret_cast<const uint8_t*>(&tmp), sizeof(tmp));
  tmp.crc = c;
  EEPROM.put(0, tmp);
  EEPROM.commit();
}

static bool loadConfig() {
  Config tmp{};
  EEPROM.get(0, tmp);
  uint32_t c = tmp.crc;
  tmp.crc = 0;
  uint32_t calc = crc32_update(0, reinterpret_cast<const uint8_t*>(&tmp), sizeof(tmp));
  if (c != calc) return false;
  cfg = tmp;
  return true;
}

// -------- ADXL345 minimal driver
static bool adxl_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADXL345::I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool adxl_read(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(ADXL345::I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ADXL345::I2C_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; ++i) buf[i] = Wire.read();
  return true;
}
static bool adxl_init() {
  uint8_t devid = 0;
  if (!adxl_read(ADXL345::REG_DEVID, &devid, 1) || devid != ADXL345::DEVID_VALUE) return false;
  if (!adxl_write(ADXL345::REG_DATAFMT, 0x0B)) return false;   // FULL_RES + ±16g
  if (!adxl_write(ADXL345::REG_BW_RATE, 0x0B)) return false;   // 200 Hz ODR
  if (!adxl_write(ADXL345::REG_POWERCTL, 0x08)) return false;  // Measure
  return true;
}
static bool adxl_read_g(float& ax, float& ay, float& az) {
  uint8_t buf[6];
  if (!adxl_read(ADXL345::REG_DATAX0, buf, 6)) return false;
  int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
  int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
  int16_t z = (int16_t)((buf[5] << 8) | buf[4]);
  // Full-res scale ~3.9 mg/LSB
  const float scale = 0.0039f;
  ax = x * scale;
  ay = y * scale;
  az = z * scale;
  return true;
}

// -------- State
static float gX = 0, gY = 0, gZ = 0;       // gravity LPF
static float ambientNorm = 1.0f;           // 0..1 (dark..bright)
static float ambientScale = 1.0f;          // cfg.ambient_min..max
static float decel_g = 0.0f;               // filtered decel
static float duty = 0.0f;                  // 0..1

// -------- Timing
static const float IMU_HZ = 200.0f;
static const float CTRL_HZ = 20.0f;
static uint32_t tIMU, tCTRL;

// -------- Helpers
static float ema_next(float prev, float input, float dt, float tau_s) {
  // Why: time-constant form is robust across variable dt
  float a = (tau_s <= 0.0f) ? 1.0f : (dt / (tau_s + dt));
  return prev + a * (input - prev);
}

static float mapDecelToDuty(float g) {
  if (g <= cfg.minDecel_g) return 0.0f;
  float x = (g - cfg.minDecel_g) / max(1e-6f, (cfg.maxDecel_g - cfg.minDecel_g));
  x = clampf(x, 0.0f, 1.0f);
  // Perceptual gamma
  float y = powf(x, max(0.5f, cfg.gamma_curve));
  // Floor depends on ambient scale (night/day)
  float floor = (ambientScale <= 0.5f) ? cfg.pwm_floor_night : cfg.pwm_floor_day;
  return clampf(floor + (1.0f - floor) * y, floor, 1.0f);
}

static float rateLimit(float prev, float target, float maxRatePerS, float dt) {
  float maxStep = maxRatePerS * dt;
  float delta = target - prev;
  if (delta > maxStep) delta = maxStep;
  else if (delta < -maxStep) delta = -maxStep;
  return prev + delta;
}

static void setPWM01(float v01) {
  v01 = clampf(v01, 0.0f, 1.0f);
  const int resolution = 1023;
  int pwm = (int)roundf(v01 * resolution);
  analogWrite(LED_PWM_PIN, pwm);
}

// -------- Ambient processing
static float readAmbientNorm() {
  int raw = analogRead(A0); // 0..1023
  // Why: normalize regardless of hardware divider
  float maxAdc = ADC_RANGE_33V ? 1023.0f : 1023.0f; // both map to 0..1023 in Arduino core
  float n = clampf(raw / maxAdc, 0.0f, 1.0f);
  // Invert if your divider produces higher voltage in dark; adjust if needed.
  // For LDR (3.3V—LDR—NODE—10k→GND), brighter => lower resistance => higher NODE voltage => larger ADC.
  return n; // 0=dark, 1=bright (with this topology)
}

static void updateAmbient(float dt, bool allowUpdate) {
  float n = readAmbientNorm();
  static float ambientEMA = 1.0f;
  if (allowUpdate) {
    ambientEMA = ema_next(ambientEMA, n, dt, cfg.ldr_ema_tau_s);
  }
  // Mild rate limit to avoid yo-yo from lights behind
  ambientNorm = rateLimit(ambientNorm, ambientEMA, 0.50f /*per s*/, dt);

  // Map to scale with gentle compression (sqrt to lift lows)
  float s = sqrtf(clampf(ambientNorm, 0.0f, 1.0f));
  ambientScale = cfg.ambient_min + (cfg.ambient_max - cfg.ambient_min) * s;
}

// -------- Setup & loop
static void printHelp() {
  Serial.println(F("\nCommands:"));
  Serial.println(F("  p = print config,  s = save defaults"));
  Serial.println(F("  t <min> <max> <gamma>   (g, g, unitless)"));
  Serial.println(F("  f <floor_day> <floor_night> (0..1)"));
  Serial.println(F("  a <amin> <amax> <tau_s> (0..1, 0..1, sec)"));
  Serial.println(F("  r <jerk_gps> <slew_per_s> <grav_tau_s>"));
}

void setup() {
  pinMode(LED_PWM_PIN, OUTPUT);
  analogWriteFreq(3000);         // Why: avoid visible flicker vs 1 kHz
  analogWriteRange(1023);

  Serial.begin(115200);
  Serial.println(); Serial.println(F("Decel->PWM ESP8266 starting..."));

  EEPROM.begin(512);
  if (!loadConfig()) {
    cfg = kDefaults;
    saveConfig();
    Serial.println(F("EEPROM defaulted."));
  } else {
    Serial.println(F("EEPROM loaded."));
  }

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  bool ok = adxl_init();
  Serial.println(ok ? F("ADXL345 OK") : F("ADXL345 FAIL"));
  if (!ok) {
    // Why: fail-safe -> keep output off; user can still tune ambient
  }

  // Seed gravity LPF from initial samples to reduce first-second wobble
  float sx=0, sy=0, sz=0;
  int N = 50;
  for (int i = 0; i < N; ++i) {
    float ax, ay, az;
    if (adxl_read_g(ax, ay, az)) {
      sx += ax; sy += ay; sz += az;
    }
    delay(5);
  }
  gX = sx / max(1, N); gY = sy / max(1, N); gZ = sz / max(1, N);

  tIMU  = micros();
  tCTRL = micros();

  printHelp();
}

void loop() {
  // ---- IMU @ ~200 Hz
  if ((int32_t)(micros() - tIMU) >= (int32_t)(1e6 / IMU_HZ)) {
    uint32_t now = micros();
    float dt = (now - tIMU) / 1e6f;
    tIMU = now;

    float ax, ay, az;
    bool ok = adxl_read_g(ax, ay, az);
    if (ok) {
      // Gravity LPF
      gX = ema_next(gX, ax, dt, cfg.grav_tau_s);
      gY = ema_next(gY, ay, dt, cfg.grav_tau_s);
      gZ = ema_next(gZ, az, dt, cfg.grav_tau_s);

      // Dynamic accel
      float dx = ax - gX;
      // Decel is negative longitudinal; assume +X is forward
      float decelInstant_g = (-dx > 0.0f) ? (-dx) : 0.0f;

      // Jerk-limit the decel estimate
      float maxStep = cfg.jerk_g_per_s * dt;
      float delta = decelInstant_g - decel_g;
      if (delta > maxStep) delta = maxStep;
      else if (delta < -maxStep) delta = -maxStep;
      decel_g += delta;
    }
    ESP.wdtFeed();
  }

  // ---- Control/output @ ~20 Hz
  if ((int32_t)(micros() - tCTRL) >= (int32_t)(1e6 / CTRL_HZ)) {
    uint32_t now = micros();
    float dt = (now - tCTRL) / 1.0e6f;
    tCTRL = now;

    // Ambient update only when nearly steady to reduce headlight influence
    bool allowAmbient = (decel_g < (cfg.minDecel_g * 0.5f));
    updateAmbient(dt, allowAmbient);

    // Map decel -> duty; apply ambient scaling and output slew
    float baseDuty = mapDecelToDuty(decel_g);
    float target = clampf(baseDuty * ambientScale, 0.0f, 1.0f);
    duty = rateLimit(duty, target, cfg.slew_per_s, dt);
    setPWM01(duty);

    // Optional debug every 0.2 s
    static uint8_t dbgDiv = 0;
    if (++dbgDiv >= 4) {
      dbgDiv = 0;
      Serial.print(F("g=")); Serial.print(decel_g, 3);
      Serial.print(F(" amb=")); Serial.print(ambientNorm, 3);
      Serial.print(F(" scale=")); Serial.print(ambientScale, 2);
      Serial.print(F(" duty=")); Serial.println(duty, 2);
    }
    ESP.wdtFeed();
  }

  // ---- Serial config (tiny CLI)
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'p') {
      Serial.println(F("\n--- Config ---"));
      Serial.printf("min/max g: %.3f / %.3f\n", cfg.minDecel_g, cfg.maxDecel_g);
      Serial.printf("gamma: %.2f\n", cfg.gamma_curve);
      Serial.printf("floors day/night: %.2f / %.2f\n", cfg.pwm_floor_day, cfg.pwm_floor_night);
      Serial.printf("ambient scale: %.2f..%.2f, tau=%.2fs\n", cfg.ambient_min, cfg.ambient_max, cfg.ldr_ema_tau_s);
      Serial.printf("jerk g/s: %.2f, slew/s: %.2f, grav tau: %.2fs\n", cfg.jerk_g_per_s, cfg.slew_per_s, cfg.grav_tau_s);
      printHelp();
    } else if (cmd == 's') {
      cfg = kDefaults;
      saveConfig();
      Serial.println(F("Saved defaults."));
    } else if (cmd == 't') {
      float a = Serial.parseFloat();
      float b = Serial.parseFloat();
      float c = Serial.parseFloat();
      if (a > 0 && b > a) { cfg.minDecel_g = a; cfg.maxDecel_g = b; cfg.gamma_curve = max(0.5f, c); saveConfig(); Serial.println(F("OK t")); }
    } else if (cmd == 'f') {
      float fd = Serial.parseFloat();
      float fn = Serial.parseFloat();
      if (fd >= 0 && fn >= 0) { cfg.pwm_floor_day = clampf(fd,0,1); cfg.pwm_floor_night = clampf(fn,0,1); saveConfig(); Serial.println(F("OK f")); }
    } else if (cmd == 'a') {
      float mn = Serial.parseFloat();
      float mx = Serial.parseFloat();
      float tau = Serial.parseFloat();
      if (mx >= mn) { cfg.ambient_min = clampf(mn,0,1); cfg.ambient_max = clampf(mx,0,1); cfg.ldr_ema_tau_s = max(0.1f,tau); saveConfig(); Serial.println(F("OK a")); }
    } else if (cmd == 'r') {
      float j = Serial.parseFloat();
      float s = Serial.parseFloat();
      float gt = Serial.parseFloat();
      cfg.jerk_g_per_s = max(0.1f,j);
      cfg.slew_per_s = max(0.1f,s);
      cfg.grav_tau_s = max(0.1f,gt);
      saveConfig();
      Serial.println(F("OK r"));
    } else {
      printHelp();
    }
  }
}
