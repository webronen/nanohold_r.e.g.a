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
#define I2C_CLK_PIN 36
#define I2C_SDA_PIN 38

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
typedef void (*StateSelect)(void);

typedef enum {
  BOOT = 0,
  AUTO = 1,
  MANUAL = 2,
} PressMode_t;

typedef enum {
  IDLE = 0,
  DOWN = 1,
  UP = 2,
  RESET = 3,
} PressStep_t;

static inline void mode_boot(void);
static inline void mode_manual(void);
static inline void mode_auto(void);

static inline void state_idle(void);
static inline void state_down(void);
static inline void state_up(void);
static inline void state_reset(void);

static const ModeSelect mode_select[] = {
  [BOOT] = mode_boot,
  [AUTO] = mode_auto,
  [MANUAL] = mode_manual
};

static const StateSelect step_select[] = {
  [IDLE] = state_idle,
  [DOWN] = state_down,
  [UP] = state_up,
  [RESET] = state_reset
};

typedef struct {
  uint32_t time_us;
  uint32_t idle_us;
  uint32_t blink_us;
  PressMode_t mode;
  PressStep_t step;
  PressStep_t buttons;
  bool open;
  bool active;
  bool ranging;
} PressState_t;

static PressState_t state = {
  .time_us = 0,
  .idle_us = 0,
  .blink_us = 0,
  .mode = BOOT,
  .step = UP,
  .buttons = IDLE,
  .open = false,
  .active = false,
  .ranging = false
};

static inline void idle_detect(void);
static inline void idle_power_save(void);
static inline void idle_shutdown(void);

static inline void boot_disconnect_gpio(void);
static inline void active_enable_power(void);
static inline void active_blink_status(void);

#endif  // MAIN_H
