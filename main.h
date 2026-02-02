#pragma once

#ifndef MAIN_H
#define MAIN_H

#include <Adafruit_TinyUSB.h>
#include <vl53l4cd_class.h>
#include <SCServo.h>

#define HZ_TO_US(hz) (1000000UL / (hz))
#define M_TO_US(m) ((m)*60 * 1000000UL)
#define S_TO_US(s) ((s)*1000000UL)

#define POWER_SAVE_TIMEOUT_M M_TO_US(1)
#define SHUTDOWN_TIMEOUT_M M_TO_US(5)

#define LDO_ENABLE_PIN 13
#define SERIAL_BAUDRATE_1M 1000000
#define I2C_FREQUENCY_400K 400000
#define I2C_CLK_PIN 36  // P1.04
#define I2C_SDA_PIN 38  // P1.06

#define SENSOR_DISTANCE_MM 30
#define SENSOR_DEBOUNCE_SAMPLES 5
#define SENSOR_DEBOUNCE_Msk ((1 << SENSOR_DEBOUNCE_SAMPLES) - 1)

#define BUTTON_DEBOUNCE_SAMPLES 5
#define BUTTON_DEBOUNCE_Msk ((1 << BUTTON_DEBOUNCE_SAMPLES) - 1)

#define GPIO_STATUS_PIN 11    // P0.11
#define GPIO_RIGHT_BUTTON 24  // P0.24
#define GPIO_LEFT_BUTTON 0    // P1.00

#define SERVO_DEFAULT_ID 1

#define PRESS_UP_POSITION 0    // ?
#define PRESS_DOWN_POSITION 0  // ?
#define PRESS_LOAD_LIMIT 0     // ?

#define PRESS_STATE_COUNT 5
#define PRESS_UP_SPEED 150
#define PRESS_DOWN_SPEED 300

VL53L4CD sensor(&Wire, -1);
SCSCL servo;

typedef void (*ModeSelect)(void);
typedef void (*StepSelect)(void);

typedef enum {
  MODE_BOOT = 0,
  MODE_AUTO = 1,
  MODE_MANUAL = 2,
} StepMode_t;

typedef enum {
  STEP_IDLE = 0,
  STEP_DOWN = 1,
  STEP_UP = 2,
  STEP_RESET = 3,
  STEP_HALT = 4
} StepState_t;

typedef enum {
  PRESS_OPEN = 0,
  PRESS_CLOSED = 1,
  PRESS_FAULT = 2,
} PressState_t;

typedef enum {
  LATCH_OFF = 0,
  LATCH_ON = 1
} LatchState_t;

typedef enum {
  POWER_SAVE = 0,
  POWER_ACTIVE = 1
} PowerState_t;

typedef enum {
  RANGE_IDLE = 0,
  RANGE_MEASURING = 1
} RangeState_t;

static inline void mode_boot(void);
static inline void mode_auto(void);
static inline void mode_manual(void);

static inline void state_idle(void);
static inline void state_down(void);
static inline void state_up(void);
static inline void state_reset(void);
static inline void state_halt(void);

static const ModeSelect change_mode[] = {
  [MODE_BOOT] = mode_boot,
  [MODE_AUTO] = mode_auto,
  [MODE_MANUAL] = mode_manual
};

static const StepSelect execute_step[] = {
  [STEP_IDLE] = state_idle,
  [STEP_DOWN] = state_down,
  [STEP_UP] = state_up,
  [STEP_RESET] = state_reset,
  [STEP_HALT] = state_halt
};

typedef struct {
  uint32_t time_us;
  uint32_t idle_us;
  uint32_t blink_us;
  StepMode_t mode;
  StepState_t step;
  StepState_t buttons;
  PressState_t press;
  LatchState_t latch;
  PowerState_t active;
  RangeState_t ranging;
} SystemState_t;

static SystemState_t state = {
  .time_us = 0,
  .idle_us = 0,
  .blink_us = 0,
  .mode = MODE_BOOT,
  .step = STEP_UP,
  .buttons = STEP_IDLE,
  .press = PRESS_CLOSED,
  .latch = LATCH_OFF,
  .active = POWER_SAVE,
  .ranging = RANGE_IDLE
};

static inline void idle_detect(void);
static inline void idle_power_save(void);
static inline void idle_shutdown(void);
static void idle_disconnect_gpio(void);

static inline void active_enable_power(void);

#endif  // MAIN_H
