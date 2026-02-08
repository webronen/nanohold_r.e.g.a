#include "main.h"

void setup() {

  idle_disconnect_gpio();
  idle_power_save();

  NRF_P1->PIN_CNF[GPIO_LEFT_BUTTON_PIN] = ((GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                                           | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos));

  NRF_P0->PIN_CNF[GPIO_RIGHT_BUTTON_PIN] = ((GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                                            | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos));

  NRF_CLOCK->TASKS_HFCLKSTART = CLOCK_TASKS_HFCLKSTART_TASKS_HFCLKSTART_Trigger;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    ;

  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->PRESCALER = TIMER_PRESCALER_PRESCALER_1Mhz;
  NRF_TIMER0->TASKS_START = TIMER_TASKS_START_TASKS_START_Trigger;
}

void loop() {

  NRF_TIMER0->TASKS_CAPTURE[0] = TIMER_TASKS_CAPTURE_TASKS_CAPTURE_Trigger;
  state.time_us = NRF_TIMER0->CC[0];

  if ((state.step != STEP_IDLE) && (state.power == POWER_IDLE)) idle_power_wakeup();

  if (state.mode > MODE_HALT) (state.mode = MODE_HALT);

  change_mode[state.mode]();
}

static inline void mode_boot(void) {

  if (state.step > STEP_HALT) (state.step = STEP_HALT);
  else (state.step = STEP_UP);

  execute_step[state.step]();

  if (state.press == PRESS_OPEN) {
    state.mode = MODE_MANUAL;
    state.step = STEP_IDLE;
  }
}

static inline void mode_auto(void) {

  if (state.step > STEP_HALT) (state.step = STEP_HALT);
  else if (state.press == PRESS_OPEN) (state.step = STEP_DOWN);
  else (state.step = STEP_UP);

  execute_step[state.step]();

  if (state.press == PRESS_CLOSED) {
    state.mode = MODE_MANUAL;
    state.step = STEP_IDLE;
  }
}

static inline void mode_manual(void) {

  static uint8_t button_history = 0;

  state.buttons = (StepState_t)(((!(NRF_P1->IN & (1 << GPIO_LEFT_BUTTON_PIN))) << 1)
                                | ((!(NRF_P0->IN & (1 << GPIO_RIGHT_BUTTON_PIN))) << 0));

  button_history = ((button_history << 1) | (!!state.buttons));

  if (button_history == UINT8_MAX) {
    state.step = state.buttons;
    state.idle_us = state.time_us;
  }

  if (state.step > STEP_HALT) (state.step = STEP_HALT);

  execute_step[state.step]();
}

static inline void state_idle(void) {
  const int32_t idle_us = (state.time_us - state.idle_us);
  if (idle_us >= SHUTDOWN_TIMEOUT_M) idle_shutdown();
  else if (idle_us >= POWER_SAVE_TIMEOUT_M) idle_power_save();
  else if ((state.power == POWER_ACTIVE) && (state.press == PRESS_OPEN)) idle_detect();
}

static inline void state_down(void) {

  servo_read_load(1);
  state.press = (PressState_t)(state.load <= PRESS_LOAD_LIMIT);

  if ((state.latch == LATCH_OFF) && (state.press == PRESS_OPEN)) {
    servo_write_position(SERVO_DEFAULT_ID, PRESS_DOWN_POSITION, PRESS_DOWN_SPEED);
    state.latch = LATCH_ON;
  } else if (state.press == PRESS_CLOSED) {
    NRF_P0->OUTCLR = (1 << GPIO_STATUS_PIN);
    state.latch = LATCH_OFF;
    state.step = STEP_IDLE;
  }
}

static inline void state_up(void) {

  servo_read_position(1);
  state.press = (PressState_t)(state.position <= PRESS_UP_POSITION);

  if ((state.latch == LATCH_OFF) && (state.press == PRESS_CLOSED)) {
    servo_write_position(SERVO_DEFAULT_ID, PRESS_UP_POSITION, PRESS_UP_SPEED);
    state.latch = LATCH_ON;
  } else if (state.press == PRESS_OPEN) {
    NRF_P0->OUTSET = (1 << GPIO_STATUS_PIN);
    state.latch = LATCH_OFF;
    state.step = STEP_IDLE;
  }
}

static inline void state_reset(void) {

  static uint32_t history_us = 0;
  uint32_t pressed_us = 0;

  if (state.buttons != STEP_RESET) (history_us = 0, state.blink_us = 0);
  else if (history_us == 0) (history_us = state.time_us);
  else pressed_us = (state.time_us - history_us);

  if ((history_us == 0) || (pressed_us < S_TO_US(4))) (state.blink_us = HZ_TO_US(12));
  else if (pressed_us < S_TO_US(5)) (state.blink_us = HZ_TO_US(120));
  else {

    idle_end_buses();

    NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = ((GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                                       | (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos));

    NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    delay(1000);

    __disable_irq();

    __DMB();
    __DSB();
    __ISB();

    __NVIC_SystemReset();
    while (true)
      ;
  }

  state_blink();
}

static inline void state_blink(void) {
  static uint32_t history_us = 0;
  if ((int32_t)(state.time_us - history_us) >= 0) {
    NRF_P0->OUT ^= (1UL << GPIO_STATUS_PIN);
    if (state.blink_us) (history_us += state.blink_us);
    else (history_us += HZ_TO_US(12));
  }
}

static inline void idle_power_wakeup(void) {

  NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = ((GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                                     | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos));

  NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

  NRF_P1->PIN_CNF[SERVO_RX_PULLUP_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  NRF_P1->OUTSET = (1UL << SERVO_RX_PULLUP_PIN);

  Wire.setPins(I2C_SDA_PIN, I2C_CLK_PIN);
  Wire.begin();
  Wire.setClock(I2C_FREQUENCY_400K);

  Serial1.begin(SERIAL_BAUDRATE_1M);  // RX: P1.01, TX: P1.02

  for (uint8_t i = 0; i < 120; i++) {
    NRF_P0->OUT ^= (1UL << GPIO_STATUS_PIN);
    delayMicroseconds(8333);
  }

  sensor.VL53L4CD_SensorInit();
  sensor.VL53L4CD_StartRanging();

  state.range = RANGE_ACTIVE;
  state.power = POWER_ACTIVE;
  state.idle_us = state.time_us;
}

static inline void idle_detect(void) {

  static VL53L4CD_RawResult_t raw_result = { 0 };
  static uint8_t detect_history = 0;

  if (state.range == RANGE_IDLE) {
    sensor.VL53L4CD_StartRanging();
    state.range = RANGE_ACTIVE;
  }

  uint8_t is_data_ready;
  if (!sensor.VL53L4CD_CheckForDataReady(&is_data_ready) && is_data_ready) {
    sensor.VL53L4CD_GetRawResult(&raw_result);
    sensor.VL53L4CD_ClearInterrupt();

    const bool object_detected = ((raw_result.range_status == 9) && (__builtin_bswap16(raw_result.distance) < SENSOR_DISTANCE_MM));
    detect_history = ((detect_history << 1) | (!!object_detected));

    if (detect_history == UINT8_MAX) {
      sensor.VL53L4CD_ClearInterruptAndStopRanging();
      state.range = RANGE_IDLE;
      state.mode = MODE_AUTO;
      detect_history = 0;
    }
  }
}

static inline void idle_power_save(void) {

  idle_end_buses();

  NRF_P1->PIN_CNF[SERVO_RX_PULLUP_PIN] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

  NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = ((GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                                     | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                     | (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos));

  NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

  state.power = POWER_IDLE;
}

static inline void idle_shutdown(void) {

  idle_end_buses();
  idle_disconnect_gpio();

  NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = ((GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                                     | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                     | (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos));

  NRF_P1->PIN_CNF[GPIO_LEFT_BUTTON_PIN] = ((GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                                           | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                           | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)
                                           | (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos));

  NRF_TIMER0->TASKS_STOP = TIMER_TASKS_STOP_TASKS_STOP_Trigger;
  NRF_CLOCK->TASKS_HFCLKSTOP = CLOCK_TASKS_HFCLKSTOP_TASKS_HFCLKSTOP_Trigger;
  NRF_CLOCK->TASKS_LFCLKSTOP = CLOCK_TASKS_LFCLKSTOP_TASKS_LFCLKSTOP_Trigger;

  __disable_irq();

  __DMB();
  __DSB();
  __ISB();

  NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
  while (true)
    ;
}

static void idle_disconnect_gpio(void) {
  for (uint8_t i = 0; i < 32; i++) NRF_P0->PIN_CNF[i] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
  for (uint8_t i = 0; i < 16; i++) NRF_P1->PIN_CNF[i] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
}

static void idle_end_buses(void) {
  Wire.endTransmission(true);
  Wire.flush();
  while (Wire.read() != -1)
    ;
  Wire.end();

  Serial1.flush();
  while (Serial1.read() != -1)
    ;
  Serial1.end();
}

static inline void servo_read_position(const uint8_t id) {

  static ServoReadRequest_t request = {
    .header = { 0xFF, 0xFF },
    .id = id,
    .length = 0x04,
    .instruction = 0x02,
    .address = 0x38,
    .read_length = 0x02,
    .checksum = 0
  };

  request.id = id;
  request.checksum = ~(id + 0x40);

  Serial1.write((uint8_t*)&request, sizeof(ServoReadRequest_t));
  Serial1.flush();

  while (Serial1.read() != -1)
    ;

  static ServoReadResponse_t response;
  Serial1.readBytes((uint8_t*)&response, sizeof(ServoReadResponse_t));

  if (response.error == 0) state.position = __builtin_bswap16(response.data);
}

static inline void servo_write_position(const uint8_t id, const uint16_t position, const uint16_t speed) {

  static ServoWritePosition_t request = {
    .header = { 0xFF, 0xFF },
    .id = 0,
    .length = 0x09,
    .instruction = 0x03,
    .address = 0x2A,
    .position = 0,
    .time = 0x0000,
    .speed = 0,
    .checksum = 0
  };

  request.id = id;
  request.position = __builtin_bswap16(position);
  request.speed = __builtin_bswap16(speed);
  request.checksum = ~(id + 0x36 + (position >> 8) + (position & 0xFF) + (speed >> 8) + (speed & 0xFF));

  Serial1.write((uint8_t*)&request, sizeof(ServoWritePosition_t));
  Serial1.flush();

  while (Serial1.read() != -1)
    ;
}

static inline void servo_read_load(const uint8_t id) {

  static ServoReadRequest_t request = {
    .header = { 0xFF, 0xFF },
    .id = 0,
    .length = 0x04,
    .instruction = 0x02,
    .address = 0x3C,
    .read_length = 0x02,
    .checksum = 0
  };

  request.id = id;
  request.checksum = ~(id + 0x40);

  Serial1.write((uint8_t*)&request, sizeof(ServoReadRequest_t));
  Serial1.flush();

  while (Serial1.read() != -1)
    ;

  static ServoReadResponse_t response;
  Serial1.readBytes((uint8_t*)&response, sizeof(ServoReadResponse_t));

  if (response.error == 0) {
    const uint16_t raw = __builtin_bswap16(response.data) & 0x07FF;
    if (raw & (1 << 10)) state.load = -(raw & 0x03FF);
    else state.load = (raw & 0x03FF);
  }
}
