#pragma once

#ifndef MAIN_H
#define MAIN_H

#include <Adafruit_TinyUSB.h>
#include <vl53l4cd_class.h>

#define HZ_TO_US(hz) (1000000UL / (hz))
#define M_TO_US(m) ((m)*60 * 1000000UL)
#define S_TO_US(s) ((s)*1000000UL)
#define JSON_RESPONSE_TEMPLATE "{\"time_us\":%lu,\"idle_us\":%lu,\"blink_us\":%lu,\"distance_mm\":%u,\"mode\":%d,\"step\":%d,\"buttons\":%d,\"press\":%d,\"latch\":%d,\"power\":%d,\"range\":%d}\r\n"

#define POWER_SAVE_TIMEOUT_M M_TO_US(1)
#define SHUTDOWN_TIMEOUT_M M_TO_US(5)

// (Port 0: 0 .. 31, Port 1: 32 + (0 .. 15)
#define LDO_ENABLE_PIN 13         // P0.13
#define SERVO_RX_PULLUP_PIN 7     // P1.07
#define I2C_CLK_PIN 36            // P1.04
#define I2C_SDA_PIN 38            // P1.06
#define GPIO_STATUS_PIN 11        // P0.11
#define GPIO_MONITOR_PIN 6        // P0.06
#define GPIO_RIGHT_BUTTON_PIN 24  // P0.24
#define GPIO_LEFT_BUTTON_PIN 0    // P1.00

#define TIMER_PRESCALER_PRESCALER_1MHZ 4  // (1us Tick)
#define SERIAL_BAUDRATE_1M 1000000
#define I2C_FREQUENCY_400K 400000

#define RANGE_STATUS_VALID 9
#define AUTO_DISTANCE_MM 35
#define DISTANCE_MM_LPF 0.75f

#define SERVO_DEFAULT_ID 1
#define SERVO_UP_POSITION 80
#define SERVO_DOWN_POSITION 300
#define SERVO_UP_SPEED 150
#define SERVO_DOWN_SPEED 150
#define SERVO_OVERLOAD_TORQUE 10   // 0 - 100%
#define SERVO_PROTECTION_TIME 25   // 25 * 40ms = 1s (max. 10s)
#define SERVO_PROTECTION_TORQUE 1  // 0 - 100%

VL53L4CD sensor(&Wire, -1);

typedef void (*ModeSelect_t)(void);
typedef void (*StepSelect_t)(void);

typedef enum StepMode {
  MODE_BOOT = 0,
  MODE_AUTO = 1,
  MODE_MANUAL = 2,
} StepMode_t;

typedef enum StepState {
  STEP_IDLE = 0,
  STEP_DOWN = 1,
  STEP_UP = 2,
  STEP_RESET = 3,
} StepState_t;

typedef enum PressState {
  PRESS_CLOSED = 0,
  PRESS_OPEN = 1,
} PressState_t;

typedef enum LatchState {
  LATCH_OFF = 0,
  LATCH_ON = 1
} LatchState_t;

typedef enum PowerState {
  POWER_IDLE = 0,
  POWER_ACTIVE = 1
} PowerState_t;

typedef enum RangeState {
  RANGE_IDLE = 0,
  RANGE_ACTIVE = 1
} RangeState_t;

static inline void mode_boot(void);
static inline void mode_auto(void);
static inline void mode_manual(void);

static inline void state_idle(void);
static inline void state_down(void);
static inline void state_up(void);
static inline void state_reset(void);
static inline void state_blink(void);

static const ModeSelect_t change_mode[] = {
  [MODE_BOOT] = mode_boot,
  [MODE_AUTO] = mode_auto,
  [MODE_MANUAL] = mode_manual,
};

static const StepSelect_t execute_step[] = {
  [STEP_IDLE] = state_idle,
  [STEP_DOWN] = state_down,
  [STEP_UP] = state_up,
  [STEP_RESET] = state_reset,
};

typedef struct __attribute__((packed)) ServoRequest {
  uint8_t header[2];    // 0xFF, 0xFF
  uint8_t id;           // Servo ID (0xFE for broadcast)
  uint8_t length;       // 0x04 for single byte operations
  uint8_t instruction;  // 0x02=read, 0x03=write, 0x06=sync write
  uint8_t address;      // Register address (e.g., 0x24 for LED, 0x2A for position)
  uint8_t data;         // For read: number of bytes to read, for write: value to write
  uint8_t checksum;     // ~(id + length + instruction + address + data)
} ServoRequest_t;

typedef struct __attribute__((packed)) ServoResponse {
  uint8_t header[2];
  uint8_t id;
  uint8_t length;
  uint8_t error;
  uint16_t data;
  uint8_t checksum;
} ServoResponse_t;

typedef struct __attribute__((packed)) ServoWritePosition {
  uint8_t header[2];
  uint8_t id;
  uint8_t length;
  uint8_t instruction;
  uint8_t address;
  uint16_t position;
  uint16_t time;
  uint16_t speed;
  uint8_t checksum;
} ServoWritePosition_t;

typedef struct __attribute__((packed)) SystemState {
  uint32_t time_us;
  uint32_t idle_us;
  uint32_t blink_us;
  uint16_t distance_mm;
  StepMode_t mode;
  StepState_t step;
  StepState_t buttons;
  PressState_t press;
  LatchState_t latch;
  PowerState_t power;
  RangeState_t range;
} SystemState_t;

static SystemState_t state = {
  .time_us = 0,
  .idle_us = 0,
  .blink_us = 0,
  .distance_mm = 0,
  .mode = MODE_BOOT,
  .step = STEP_UP,
  .buttons = STEP_IDLE,
  .press = PRESS_CLOSED,
  .latch = LATCH_OFF,
  .power = POWER_ACTIVE,
  .range = RANGE_ACTIVE
};

static inline void idle_power_wakeup(void);
static inline void idle_detect(void);
static inline void idle_power_save(void);
static inline void idle_shutdown(void);

static void servo_target_position(const uint8_t id, const uint16_t position, const uint16_t speed);
static bool servo_move_flag(const uint8_t id);
static void servo_flush_clear(void);

static inline void handle_serial_commands(void);

#endif  // MAIN_H
