#include <Arduino.h>
#include "driver/rmt.h"
#include <EEPROM.h>
#include <Wire.h>
#include <SparkFun_VL53L1X.h>
#include "esp_system.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"

SFEVL53L1X distanceSensor;
// ------------------------------------------------------------
// CURRENT CHECK
// ------------------------------------------------------------
#define CURR_PIN               34
#define CURR_CHECK_WINDOW_MS   10 //ms
#define CURR_TRIP_LIMIT        1000 //analogRead counts
#define CURR_TRIPS_MAX         2

// ============================================================
//  PIN / PWM CONFIG
// ============================================================
#define EN_PIN        32
#define PWM_GPIO_CW   26
#define PWM_GPIO_CCW  27
#define HOME_PIN      25

#define PWM_TIMEBASE_HZ       1000000 // 1MHZ -> 1 tick = 1us
#define PWM_FREQ_HZ           1000 // 1kHZ
#define PWM_PERIOD_TICKS      (PWM_TIMEBASE_HZ/PWM_FREQ_HZ) // 1000 ticks = 1ms
#define DUTY_PCT              15
#define DUTY_TICKS            (PWM_PERIOD_TICKS*DUTY_PCT/100) // duty cycle lasts 150/1000 ticks
#define SAMPLE_TICKS          100 // sample at tick 100

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

#define HOMING_SPEED       2000
#define BACKOFF_SPEED      500

#define FB_TOL             10

// ============================================================
//  DRILL CONFIG
// ============================================================
#define DRILL_STARTUP_MS   1000   // ms to spin up/down drill before/after move

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

//mcpwm handles
mcpwm_timer_handle_t timer = NULL;
mcpwm_oper_handle_t oper = NULL;
mcpwm_cmpr_handle_t duty_comparator = NULL;
mcpwm_cmpr_handle_t sample_comparator = NULL;
mcpwm_gen_handle_t cw_generator = NULL;
mcpwm_gen_handle_t ccw_generator = NULL;

volatile bool sample_flag = false;
volatile uint16_t last_curr = 0;

uint32_t curr_window = 0;
uint8_t curr_trips = 0;

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

// ------------------------------------------------------------
// CURRENT READINGS
// ------------------------------------------------------------
static bool IRAM_ATTR current_flag(mcpwm_cmpr_handle_t comparator, const mcpwm_compare_event_data_t *edata, void *user_ctx){
  sample_flag = true;
  return false;
}

void checkCurr(){
    if (!currentArmed || overcurrent) return;

    if (sample_flag){
        sample_flag = false;
        last_curr = analogRead(CURR_PIN);
        //Serial.printf("%d, ", last_curr);
        if (last_curr > CURR_TRIP_LIMIT) curr_trips++;
    }
       
    if (millis() - curr_window >= CURR_CHECK_WINDOW_MS){
        if (curr_trips >= CURR_TRIPS_MAX) overcurrent = true;
        
        curr_trips = 0;
        curr_window = millis();
        //Serial.println();
    }
}

void doSense(){
    // drill OFF baseline
    uint32_t sum = 0; uint16_t n = 0;
    uint32_t t0 = millis();
    while (n < 200 && millis() - t0 < 2000){
        if (sample_flag){
            sample_flag = false;
            uint16_t v = analogRead(CURR_PIN);
            sum += v; n++;
            if (n % 20 == 0) Serial.printf("%d, ", v);
        }
    }
    Serial.printf("\ndrill OFF avg: %lu (n=%u)\n", n ? sum/n : 0, n);

    drillCW(DUTY_PCT);
    delay(DRILL_STARTUP_MS);

    // drill ON
    sum = 0; n = 0; t0 = millis();
    while (n < 200 && millis() - t0 < 2000){
        if (sample_flag){
            sample_flag = false;
            uint16_t v = analogRead(CURR_PIN);
            sum += v; n++;
            if (n % 20 == 0) Serial.printf("%d, ", v);
        }
    }
    Serial.printf("\ndrill ON avg: %lu (n=%u)\n", n ? sum/n : 0, n);

    pwmOff();
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
void setDuty(uint8_t pct) {
    uint32_t duty_ticks = (uint32_t)pct * PWM_PERIOD_TICKS / 100;
    mcpwm_comparator_set_compare_value(duty_comparator, duty_ticks);
}

//spin CW: CCW pin forced low, CW pin follows PWM
void drillCW(uint8_t pct){
    setDuty(pct);
    mcpwm_generator_set_force_level(ccw_generator, 0, true);   // force CCW low
    mcpwm_generator_set_force_level(cw_generator, -1, true);   // release CW (follow actions)
}

//spin CCW: CW pin forced low, CCW pin follows PWM
void drillCCW(uint8_t pct){
    setDuty(pct);
    mcpwm_generator_set_force_level(cw_generator, 0, true);    // force CW low
    mcpwm_generator_set_force_level(ccw_generator, -1, true);  // release CCW
}

void pwmOff(){
    mcpwm_generator_set_force_level(cw_generator, 0, true);    // both forced low
    mcpwm_generator_set_force_level(ccw_generator, 0, true);
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
                pwmOff();
                Serial.flush();
                Serial.println("ESTOP");
                esp_restart();
            }
        }
    }
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
void mcpwm_init(){

  //configure and create timer
  mcpwm_timer_config_t timer_cfg = {
    .group_id       = 0,
    .clk_src        = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz  = PWM_TIMEBASE_HZ,
    .count_mode     = MCPWM_TIMER_COUNT_MODE_UP,
    .period_ticks    = PWM_PERIOD_TICKS,
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));

  //configure and create operator
  mcpwm_operator_config_t operator_cfg = {
    .group_id = 0,
  };
  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_cfg, &oper));

  //connect operator to timer
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

  //configure and create comparator
  mcpwm_comparator_config_t comparator_cfg = {
    .flags = { .update_cmp_on_tez = true },
  };
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_cfg, &duty_comparator)); //comparator to indicate end of duty cycle
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(duty_comparator, DUTY_TICKS));

  //configure and create CW generator
  mcpwm_generator_config_t generator_cw_cfg = {
    .gen_gpio_num = PWM_GPIO_CW,
  };
  ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_cw_cfg, &cw_generator));
  //configure and create CCW generator
  mcpwm_generator_config_t generator_ccw_cfg = {
    .gen_gpio_num = PWM_GPIO_CCW,
  };
  ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_ccw_cfg, &ccw_generator));

  //set generator actions
  //cw
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(cw_generator, 
    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH))); //set pin high on timer empty
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(cw_generator,
    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, duty_comparator, MCPWM_GEN_ACTION_LOW))); //set pin low on duty comparator (end of duty cycle)
  //ccw
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(ccw_generator, 
    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH))); //set pin high on timer empty
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(ccw_generator,
    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, duty_comparator, MCPWM_GEN_ACTION_LOW))); //set pin low on duty comparator (end of duty cycle)

  //create sample comparator to call current_sense callback at half duty on ticks
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_cfg, &sample_comparator)); //comparator for half duty cycle (sense current)
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(sample_comparator, SAMPLE_TICKS));
  //register sample callback function
  mcpwm_comparator_event_callbacks_t sample_cb_cfg = {
    .on_reach = current_flag,
  };
  ESP_ERROR_CHECK(mcpwm_comparator_register_event_callbacks(sample_comparator, &sample_cb_cfg, NULL));

  //start timer
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
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
    
    overcurrent = false;
    curr_trips = 0;
    curr_window = millis();
    currentArmed = false;
    
    // 1. Spin up drill CW
    drillCW(DUTY_PCT);
    delay(DRILL_STARTUP_MS);
    currentArmed = true;

    // 2. Feed to max range
    bool ok = moveAbsoluteCheck(calData.range);

    if (overcurrent) {
        currentArmed = false;
        pwmOff();
        isHomed = false;
        return RC_OVERCURRENT;
    }

    if (!ok) {
        pwmOff();
        return RC_MOVE_UNSAFE;
    }

    // 3. Reverse drill CCW
    drillCCW(DUTY_PCT);
    delay(DRILL_STARTUP_MS);

    // 4. Return to home offset
    ok = moveAbsoluteCheck(calData.offset);
    if (!ok) {
        pwmOff();
        return RC_MOVE_UNSAFE;
    }

    // 5. Stop drill
    pwmOff();

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
        pwmOff();
        Serial.flush();
        esp_restart();
    } else if (cmd.equalsIgnoreCase("sense")) {
        doSense();
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
    mcpwm_init();
    pwmOff();
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