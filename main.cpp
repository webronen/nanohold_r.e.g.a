#include "main.h"

void setup() {

  idle_shutdown_gpio();
  idle_power_save();

  NRF_P1->PIN_CNF[GPIO_LEFT_BUTTON] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                                      | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

  NRF_P0->PIN_CNF[GPIO_RIGHT_BUTTON] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                                       | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

  NRF_CLOCK->TASKS_HFCLKSTART = CLOCK_TASKS_HFCLKSTART_TASKS_HFCLKSTART_Trigger;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    ;

  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->PRESCALER = 4;  // 1 MHz (1us Tick)
  NRF_TIMER0->TASKS_START = TIMER_TASKS_START_TASKS_START_Trigger;
}

void loop() {

  NRF_TIMER0->TASKS_CAPTURE[0] = TIMER_TASKS_CAPTURE_TASKS_CAPTURE_Trigger;
  state.time_us = NRF_TIMER0->CC[0];

  if ((state.step != STEP_IDLE) && (state.power == POWER_IDLE)) idle_power_wakeup();

  change_mode[state.mode]();
}

static inline void mode_boot(void) {

  if (state.press == PRESS_FAULT) state.step = STEP_HALT;
  else state.step = STEP_UP;

  execute_step[state.step]();

  if (state.press == PRESS_OPEN) {
    state.mode = MODE_MANUAL;
    state.step = STEP_IDLE;
  }
}

static inline void mode_auto(void) {

  if (state.press == PRESS_FAULT) state.step = STEP_HALT;
  else if (state.press == PRESS_OPEN) state.step = STEP_DOWN;
  else state.step = STEP_UP;

  execute_step[state.step]();

  if (state.press == PRESS_CLOSED) {
    state.mode = MODE_MANUAL;
    state.step = STEP_IDLE;
  }
}

static inline void mode_manual(void) {

  static uint8_t button_history = 0;

  state.buttons = (StepState_t)(((!(NRF_P1->IN & (1 << GPIO_LEFT_BUTTON))) << 1)
                                | ((!(NRF_P0->IN & (1 << GPIO_RIGHT_BUTTON))) << 0));

  button_history = ((button_history << 1) | (!!state.buttons));

  if (button_history == UINT8_MAX) {
    state.step = state.buttons;
    state.idle_us = state.time_us;
  }

  execute_step[state.step]();
}

static inline void state_idle(void) {
  const int32_t idle_us = (state.time_us - state.idle_us);
  if (idle_us >= SHUTDOWN_TIMEOUT_M) idle_shutdown();
  else if (idle_us >= POWER_SAVE_TIMEOUT_M) idle_power_save();
  else if (state.power == POWER_ACTIVE && state.press == PRESS_OPEN) idle_detect();
}

static inline void state_down(void) {

  NRF_P0->OUTCLR = (1 << GPIO_STATUS_PIN);

  state.press = (PressState_t)(servo.ReadLoad(SERVO_DEFAULT_ID) < PRESS_LOAD_LIMIT);

  if ((state.latch == LATCH_OFF) && (state.press == PRESS_OPEN)) {
    servo.WritePos(SERVO_DEFAULT_ID, PRESS_DOWN_POSITION, 0, PRESS_DOWN_SPEED);
    state.latch = LATCH_ON;
  } else {
    state.latch = LATCH_OFF;
    state.step = STEP_IDLE;
  }
}

static inline void state_up(void) {

  NRF_P0->OUTSET = (1 << GPIO_STATUS_PIN);

  state.press = (PressState_t)(servo.ReadPos(SERVO_DEFAULT_ID) >= PRESS_UP_POSITION);

  if ((state.latch == LATCH_OFF) && (state.press == PRESS_CLOSED)) {
    servo.WritePos(SERVO_DEFAULT_ID, PRESS_UP_POSITION, 0, PRESS_UP_SPEED);
    state.latch = LATCH_ON;
  } else {
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

    NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                                      | (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

    NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    delay(S_TO_MS(1));

    __disable_irq();

    __DMB();
    __DSB();
    __ISB();

    __NVIC_SystemReset();
    while (true)
      ;
  }

  state_halt();
}

static inline void state_halt(void) {
  static uint32_t history_us = 0;
  if ((int32_t)(state.time_us - history_us) >= 0) {
    NRF_P0->OUT ^= (1UL << GPIO_STATUS_PIN);
    if (state.blink_us) (history_us += state.blink_us);
    else (history_us += HZ_TO_US(12));
  }
}

static inline void idle_power_wakeup(void) {

  NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                                    | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

  NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

  Wire.setPins(I2C_SDA_PIN, I2C_CLK_PIN);
  Wire.begin();
  Wire.setClock(I2C_FREQUENCY_400K);

  Serial1.setPins(UART_RX_PIN, UART_TX_PIN);
  Serial1.begin(UART_BAUDRATE_1M);
  servo.pSerial = &Serial1;

  for (uint8_t i = 0; i < 120; i++) {
    NRF_P0->OUT ^= (1UL << GPIO_STATUS_PIN);
    delay(HZ_TO_MS(12));
  }

  sensor.VL53L4CD_SensorInit();
  sensor.VL53L4CD_StartRanging();

  servo.EnableTorque(SERVO_DEFAULT_ID, true);

  state.range = RANGE_ACTIVE;
  state.power = POWER_ACTIVE;
  state.idle_us = state.time_us;
}

static inline void idle_detect(void) {

  static VL53L4CD_RawResult_t result = { 0 };
  static uint8_t detect_history = 0;

  if (state.range == RANGE_IDLE) {
    sensor.VL53L4CD_StartRanging();
    state.range = RANGE_ACTIVE;
  }

  uint8_t data_ready;
  if (!sensor.VL53L4CD_CheckForDataReady(&data_ready) && data_ready) {
    sensor.VL53L4CD_GetRawResult(&result);
    sensor.VL53L4CD_ClearInterrupt();

    const bool object_detected = (result.range_status == 9 && __builtin_bswap16(result.distance) < SENSOR_DISTANCE_MM);
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
  
  Wire.end();
  Serial1.end();

  NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                                    | (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

  NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

  state.power = POWER_IDLE;
}

static inline void idle_shutdown(void) {

  Wire.end();
  Serial1.end();

  idle_shutdown_gpio();

  NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                                    | (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

  NRF_P1->PIN_CNF[GPIO_LEFT_BUTTON] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                                      | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)
                                      | (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);

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

static void idle_shutdown_gpio(void) {
  for (uint8_t i = 0; i < 32; i++) NRF_P0->PIN_CNF[i] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
  for (uint8_t i = 0; i < 16; i++) NRF_P1->PIN_CNF[i] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
}
