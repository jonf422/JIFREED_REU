#include <Arduino.h>
#include "driver/rmt.h"
#include "driver/ledc.h"
#include <EEPROM.h>
#include <Wire.h>
#include <SparkFun_VL53L1X.h>
#include "esp_system.h"

SFEVL53L1X distanceSensor;
// ------------------------------------------------------------
// CURRENT CHECK
// ------------------------------------------------------------
#define CURR_PIN            34
#define CURR_CHECK_INTERVAL 100
#define CURR_TRIP_MARGIN    600
#define CURR_TRIP_CONSEC    3

// ============================================================
//  PIN / PWM CONFIG
// ============================================================
#define EN_PIN        32
#define PWM_GPIO_CW   26
#define PWM_GPIO_CCW  27
#define HOME_PIN      25
#define PWM_FREQ      1000
#define PWM_RES_BITS  8
#define PWM_CH_CW     LEDC_CHANNEL_0
#define PWM_CH_CCW    LEDC_CHANNEL_1

// ============================================================
//  STEPPER CONFIG
// ============================================================
#define STEP_GPIO          16
#define DIR_GPIO           17
#define RMT_CHANNEL        RMT_CHANNEL_0
#define RMT_CLK_DIV        80

#define LOW_SPEED          500
#define HIGH_SPEED         6500
#define ACC                25
#define ACC_DISC_INTERVAL  10

#define HOMING_SPEED       1000
#define BACKOFF_SPEED      500

#define FB_TOL             10

// ============================================================
//  DRILL CONFIG
// ============================================================
#define DRILL_STARTUP_MS   2000   // ms to spin up/down drill before/after move
#define DRILL_DUTY_PCT     15     // PWM duty cycle for drill motor (0-100)

// ============================================================
//  EEPROM CONFIG
// ============================================================
#define EEPROM_SIZE            24
#define EEPROM_ADDR_MAGIC       0
#define EEPROM_ADDR_OFFSET      4
#define EEPROM_ADDR_RANGE       8
#define EEPROM_ADDR_FB_MAGIC   12
#define EEPROM_ADDR_FB_M       16
#define EEPROM_ADDR_FB_B       20
#define EEPROM_MAGIC           0xDEADBEEF
#define EEPROM_FB_MAGIC        0xCAFEF00D

// ============================================================
//  RETURN CODES
// ============================================================
#define RC_OK              0
#define RC_MOVE_UNSAFE     1
#define RC_OVERCURRENT     2
// codes 2-9 reserved

// ============================================================
//  CALIBRATION DATA
// ============================================================
struct CalData {
    uint32_t      offset;
    unsigned long range;
};

struct FbCal {
    float m;
    float b;
};

CalData calData    = { 0, 0 };
bool    calValid   = false;
FbCal   fbCal      = { 0.0f, 0.0f };
bool    fbCalValid = false;

// ============================================================
//  MACHINE STATE
// ============================================================
volatile long currentPos = 0;
bool          isHomed    = false;

volatile bool overcurrent      = false;
bool          currentArmed     = false;
uint16_t      currentBaseline  = 0;
uint16_t      currentTripCount = 0;

// ------------------------------------------------------------
// CURRENT READINGS
// ------------------------------------------------------------
uint16_t getCurrPeak(){
    uint16_t peakCurr = 0;
    uint32_t t0 = micros();
    while (micros() - t0 < 1500){
        uint16_t v = analogRead(CURR_PIN);
        if (v > peakCurr) peakCurr = v;
    }
    return peakCurr;
}

void checkCurr(){
    static uint16_t steps = 0;
    if (!currentArmed || overcurrent) return;
    if (++steps < CURR_CHECK_INTERVAL) return;
    steps = 0;

    if (getCurrPeak() > currentBaseline + CURR_TRIP_MARGIN){
        if (++currentTripCount >= CURR_TRIP_CONSEC) overcurrent = true;
    }
    else{
        currentTripCount = 0;
    }
}

// ============================================================
//  EEPROM HELPERS
// ============================================================
void eepromWriteU32(int addr, uint32_t val) {
    EEPROM.writeByte(addr + 0, (val >> 24) & 0xFF);
    EEPROM.writeByte(addr + 1, (val >> 16) & 0xFF);
    EEPROM.writeByte(addr + 2, (val >>  8) & 0xFF);
    EEPROM.writeByte(addr + 3, (val >>  0) & 0xFF);
}

uint32_t eepromReadU32(int addr) {
    uint32_t val = 0;
    val |= (uint32_t)EEPROM.readByte(addr + 0) << 24;
    val |= (uint32_t)EEPROM.readByte(addr + 1) << 16;
    val |= (uint32_t)EEPROM.readByte(addr + 2) <<  8;
    val |= (uint32_t)EEPROM.readByte(addr + 3) <<  0;
    return val;
}

float eepromReadFloat(int addr) {
    uint32_t bits = eepromReadU32(addr);
    float val;
    memcpy(&val, &bits, 4);
    return val;
}

// ============================================================
//  CALIBRATION LOAD
// ============================================================
void loadCalibration() {
    uint32_t magic = eepromReadU32(EEPROM_ADDR_MAGIC);
    if (magic == EEPROM_MAGIC) {
        calData.offset = eepromReadU32(EEPROM_ADDR_OFFSET);
        calData.range  = (unsigned long)eepromReadU32(EEPROM_ADDR_RANGE);
        calValid = true;
    } else {
        calValid = false;
    }

    uint32_t fbMagic = eepromReadU32(EEPROM_ADDR_FB_MAGIC);
    if (fbMagic == EEPROM_FB_MAGIC) {
        fbCal.m    = eepromReadFloat(EEPROM_ADDR_FB_M);
        fbCal.b    = eepromReadFloat(EEPROM_ADDR_FB_B);
        fbCalValid = true;
    } else {
        fbCalValid = false;
    }
}

// ============================================================
//  PWM HELPERS
// ============================================================
void setDuty(ledc_channel_t ch, uint32_t duty) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
}

void bothOff() {
    setDuty(PWM_CH_CW,  0);
    setDuty(PWM_CH_CCW, 0);
}

// ============================================================
//  STOP FLAG
// ============================================================
volatile bool stopRequested = false;

// Call inside any blocking loop. Drains serial looking for "stop";
// if found, kills outputs and resets the chip immediately.
void checkStop() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == 's') {
            // Read rest of "stop\n" non-blocking
            delay(2);
            String tail = "";
            while (Serial.available()) tail += (char)Serial.read();
            tail.trim();
            if (tail == "top") {
                bothOff();
                Serial.flush();
                Serial.println("ESTOP");
                esp_restart();
            }
        }
    }
}


void pwmSetChannel(ledc_channel_t ch, uint8_t pct) {
    uint32_t duty  = (uint32_t)(pct * (1 << PWM_RES_BITS) / 100);
    ledc_channel_t other = (ch == PWM_CH_CW) ? PWM_CH_CCW : PWM_CH_CW;
    setDuty(other, 0);
    delay(1);
    setDuty(ch, duty);
}

// ============================================================
//  HOME SWITCH
// ============================================================
bool homeTriggered() {
    return digitalRead(HOME_PIN) == LOW;
}

// ============================================================
//  HARDWARE INIT
// ============================================================
void pwmInit() {
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)PWM_RES_BITS,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = PWM_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t chCW = {
        .gpio_num   = PWM_GPIO_CW,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = PWM_CH_CW,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&chCW);

    ledc_channel_config_t chCCW = {
        .gpio_num   = PWM_GPIO_CCW,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = PWM_CH_CCW,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&chCCW);
}

void rmtStepperInit() {
    rmt_config_t config = {};
    config.rmt_mode      = RMT_MODE_TX;
    config.channel       = RMT_CHANNEL;
    config.gpio_num      = (gpio_num_t)STEP_GPIO;
    config.clk_div       = RMT_CLK_DIV;
    config.mem_block_num = 1;

    config.tx_config.loop_en        = false;
    config.tx_config.carrier_en     = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level     = RMT_IDLE_LEVEL_LOW;

    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
}

void feedbackSensorInit() {
    Wire.begin(21, 22);

    if (distanceSensor.begin() != 0) {
        // sensor not found — continue anyway
    } else {
        distanceSensor.setDistanceModeShort();
        distanceSensor.setTimingBudgetInMs(50);
        distanceSensor.setIntermeasurementPeriod(55);
        distanceSensor.setROI(4, 4, 199);
        distanceSensor.startRanging();
    }
}

// ============================================================
//  LOW-LEVEL STEP PRIMITIVE
// ============================================================
void sendStep(uint32_t freq_hz) {
    uint32_t half = (1000000UL / freq_hz) / 2;
    if (half < 1) half = 1;

    rmt_item32_t item;
    item.level0    = 0;
    item.duration0 = half;
    item.level1    = 1;
    item.duration1 = half;

    rmt_write_items(RMT_CHANNEL, &item, 1, true);
    checkStop();
    checkCurr();
}

// ============================================================
//  TRAPEZOIDAL RELATIVE MOVE
// ============================================================
void moveRelative(long steps) {
    if (steps == 0) return;

    bool     cw = (steps > 0);
    uint32_t n  = (uint32_t)abs(steps);

    digitalWrite(DIR_GPIO, cw ? HIGH : LOW);

    uint16_t f            = LOW_SPEED;
    uint32_t curve_length = n / 2;
    uint32_t acc_steps    = min((int)curve_length,
                                (HIGH_SPEED - LOW_SPEED) / (ACC / ACC_DISC_INTERVAL));
    uint32_t vel_steps    = acc_steps / ACC_DISC_INTERVAL;
    uint32_t coast_steps  = n - 2 * (vel_steps * ACC_DISC_INTERVAL);

    for (uint32_t i = 0; i < vel_steps && !overcurrent; i++) {
        for (uint16_t j = 0; j < ACC_DISC_INTERVAL; j++) sendStep(f);
        f += ACC;
    }
    for (uint32_t i = 0; i < coast_steps && !overcurrent; i++) sendStep(f);
    for (uint32_t i = 0; i < vel_steps && !overcurrent; i++) {
        for (uint16_t j = 0; j < ACC_DISC_INTERVAL; j++) sendStep(f);
        f -= ACC;
    }

    rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY);
    currentPos += cw ? -(long)n : (long)n;
}

// ============================================================
//  FEEDBACK HELPERS
// ============================================================
int stepsToMm(unsigned long steps) {
    return steps * fbCal.m + fbCal.b;
}

bool toolNotStuck(unsigned long pos) {
    float idealMM = stepsToMm(pos);
    float dist    = distanceSensor.getDistance();
    return abs(dist - idealMM) < FB_TOL;
}

// ============================================================
//  ABSOLUTE MOVE (with stuck check)
// ============================================================
bool moveAbsoluteCheck(long targetPos) {
    if (!isHomed)                               return false;
    if ((unsigned long)targetPos > calData.range) return false;

    long delta = targetPos - currentPos;
    if (delta == 0) return true;

    moveRelative(-delta);
    currentPos = targetPos;

    delay(50);
    return toolNotStuck(currentPos);
}

void moveAbsolute(long targetPos) {
    if (!isHomed)                               return;
    if ((unsigned long)targetPos > calData.range) return;

    long delta = targetPos - currentPos;
    if (delta == 0) return;

    moveRelative(-delta);
    currentPos = targetPos;
}

// ============================================================
//  HOMING
// ============================================================
void doHome() {
    digitalWrite(DIR_GPIO, HIGH);
    while (!homeTriggered()) { sendStep(HOMING_SPEED); checkStop(); }

    delay(50);

    digitalWrite(DIR_GPIO, LOW);
    while (homeTriggered()) { sendStep(BACKOFF_SPEED); checkStop(); }

    delay(50);

    digitalWrite(DIR_GPIO, HIGH);
    while (!homeTriggered()) { sendStep(BACKOFF_SPEED); checkStop(); }

    delay(50);

    rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY);

    currentPos = 0;
    isHomed    = true;

    if (calValid) {
        moveAbsolute(calData.offset);
    }
}

// ============================================================
//  DRILL SEQUENCE
// ============================================================
// Returns RC_OK (0) on success, RC_MOVE_UNSAFE (1) if stuck at any point.
//
// Sequence:
//   1. Spin up drill (CW) for DRILL_STARTUP_MS
//   2. Feed to max range (checked move) -- abort on stuck
//   3. Reverse drill (CCW) for DRILL_STARTUP_MS
//   4. Return to home offset (checked move) -- abort on stuck
//   5. Stop drill
//
int doDrill() {
    if (!isHomed || !calValid) return RC_MOVE_UNSAFE;

    // 1. Spin up drill CW
    pwmSetChannel(PWM_CH_CW, DRILL_DUTY_PCT);
    delay(DRILL_STARTUP_MS);

    currentBaseline = getCurrPeak();
    currentArmed    = true;

    // 2. Feed to max range
    bool ok = moveAbsoluteCheck(calData.range);

    if (overcurrent) {
        currentArmed = false;
        pwmSetChannel(PWM_CH_CCW, DRILL_DUTY_PCT);
        delay(DRILL_STARTUP_MS);
        moveAbsolute(calData.offset);
        bothOff();
        isHomed = false;
        return RC_OVERCURRENT;
    }

    if (!ok) {
        bothOff();
        return RC_MOVE_UNSAFE;
    }

    // 3. Reverse drill CCW
    pwmSetChannel(PWM_CH_CCW, DRILL_DUTY_PCT);
    delay(DRILL_STARTUP_MS);

    // 4. Return to home offset
    ok = moveAbsoluteCheck(calData.offset);
    if (!ok) {
        bothOff();
        return RC_MOVE_UNSAFE;
    }

    // 5. Stop drill
    bothOff();

    return RC_OK;
}

// ============================================================
//  COMMAND HANDLER
// ============================================================
void handleCommand(const String &raw) {
    String cmd = raw;
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd.equalsIgnoreCase("home")) {
        doHome();
        Serial.println(RC_OK);

    } else if (cmd.equalsIgnoreCase("drill")) {
        int rc = doDrill();
        Serial.println(rc);

    } else if (cmd.equalsIgnoreCase("stop")) {
        bothOff();
        Serial.flush();
        esp_restart();
    }
    // unknown commands silently ignored
}

// ============================================================
//  SETUP & LOOP
// ============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    EEPROM.begin(EEPROM_SIZE);

    pinMode(EN_PIN,   OUTPUT);
    pinMode(DIR_GPIO, OUTPUT);
    pinMode(HOME_PIN, INPUT_PULLUP);

    digitalWrite(EN_PIN,   HIGH);
    digitalWrite(DIR_GPIO, HIGH);

    rmtStepperInit();
    pwmInit();
    bothOff();
    feedbackSensorInit();
    loadCalibration();
}

String inputBuf = "";

void loop() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            handleCommand(inputBuf);
            inputBuf = "";
        } else {
            inputBuf += c;
        }
    }
}



