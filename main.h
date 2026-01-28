#pragma once

#ifndef MAIN_H
#define MAIN_H

#include <Adafruit_TinyUSB.h>
#include <vl53l4cd_class.h>
#include <SCServo.h>

#define HZ_TO_US(hz) (1000000UL / (hz))
#define M_TO_US(m) ((m)*60 * 1000000UL)
#define S_TO_US(s) ((s)*1000000UL)

#define POWER_TIMEOUT_M M_TO_US(5)
#define LDO_ENABLE_PIN 13
#define SERIAL_BAUDRATE_1M 1000000
#define I2C_FREQUENCY_400K 400000
#define I2C_CLK_PIN 36
#define I2C_SDA_PIN 38

#define AUTO_DISTANCE_MM 30
#define AUTO_TRIGGER_SAMPLES 5

#define GPIO_STATUS_PIN 11   // P0.11
#define GPIO_RIGHT_BUTTON 0  // P1.00
#define GPIO_LEFT_BUTTON 24  // P0.24

#define SERVO_DEFAULT_ID 1

#define PRESS_UP_POSITION 0    // ?
#define PRESS_DOWN_POSITION 0  // ?
#define PRESS_LOAD_LIMIT 0     // ?

#define PRESS_UP_SPEED 150
#define PRESS_DOWN_SPEED 300

VL53L4CD sensor(&Wire, -1);
SCSCL servo;

typedef enum {
  MANUAL = 0,
  AUTO = 1,
} PressMode;

typedef enum {
  IDLE = 0,
  UP = 1,
  DOWN = 2,
  RESET = 3
} PressStep;

typedef struct {
  PressMode mode;
  PressStep step;
  PressStep buttons;
  uint32_t time_us;
  uint32_t idle_us;
  bool open;
  bool latch;
} PressState;

static PressState state = { .mode = AUTO };

static void blink_status_leds(const uint32_t interval_us);
static inline void handle_reset_state(void);
static inline void prepare_active_state(void);
static inline void handle_idle_state(void);
static inline void handle_press_up(void);
static inline void handle_press_down(void);

#endif  // MAIN_H
