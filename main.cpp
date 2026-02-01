#include "main.h"

void setup() {

  gpio_disconnect_system();

  NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |  //
                                    (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

  NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

  NRF_P1->PIN_CNF[GPIO_LEFT_BUTTON] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |  //
                                      (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

  NRF_P0->PIN_CNF[GPIO_RIGHT_BUTTON] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |  //
                                       (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

  NRF_CLOCK->TASKS_HFCLKSTART = CLOCK_TASKS_HFCLKSTART_TASKS_HFCLKSTART_Trigger;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    ;

  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->PRESCALER = 4;  // 1 MHz
  NRF_TIMER0->TASKS_START = TIMER_TASKS_START_TASKS_START_Trigger;
}

void loop() {

  NRF_TIMER0->TASKS_CAPTURE[0] = TIMER_TASKS_CAPTURE_TASKS_CAPTURE_Trigger;
  state.time_us = NRF_TIMER0->CC[0];

  if (state.step != IDLE && !state.active) active_enable_power();

  mode_select[state.mode]();
}

static inline void mode_boot(void) {
  state.step = UP;
  step_select[state.step]();
  if (state.open) {
    state.mode = MANUAL;
    state.step = IDLE;
  }
}

static inline void mode_manual(void) {

  state.buttons = (PressStep_t)(((!(NRF_P1->IN & (1 << GPIO_LEFT_BUTTON))) << 1) |  //
                                ((!(NRF_P0->IN & (1 << GPIO_RIGHT_BUTTON))) << 0));

  static uint8_t button_debounce = 0;
  button_debounce = ((button_debounce << 1) | (!!state.buttons));

  if ((button_debounce & BUTTON_DEBOUNCE_Msk) == BUTTON_DEBOUNCE_Msk) {
    state.step = state.buttons;
    state.idle_us = state.time_us;
  }

  step_select[state.step]();
}

static inline void mode_auto(void) {
  if (!state.open) {
    state.step = UP;
    step_select[state.step]();
  } else {
    state.step = DOWN;
    step_select[state.step]();
    if (!state.open) {
      state.mode = MANUAL;
      state.step = IDLE;
    }
  }
}

static inline void state_idle(void) {

  if (state.active) {
    if (state.open) idle_detect();
    if ((int32_t)(state.time_us - state.idle_us) >= POWER_SAVE_TIMEOUT_M) idle_power_save();
  } else if ((int32_t)(state.time_us - state.idle_us) >= SHUTDOWN_TIMEOUT_M) idle_shutdown();
}

static inline void state_down(void) {

  static bool latch = false;

  NRF_P0->OUTCLR = (1 << GPIO_STATUS_PIN);

  // state.open = (servo.ReadLoad(SERVO_DEFAULT_ID) <= PRESS_LOAD_LIMIT);

  if (!latch && state.open) {
    // servo.WritePos(SERVO_DEFAULT_ID, PRESS_DOWN_POSITION, 0, PRESS_DOWN_SPEED);
    debug_print_status("[STATE] -> The press moves down.\r\n", 1);

    latch = true;
    state.open = false;
    delay(1000);
  } else {
    latch = false;
    state.step = IDLE;
  }
}

static inline void state_up(void) {

  static bool latch = false;

  NRF_P0->OUTSET = (1 << GPIO_STATUS_PIN);

  // state.open = (servo.ReadPos(SERVO_DEFAULT_ID) >= PRESS_UP_POSITION);

  if (!latch && !state.open) {

    // servo.WritePos(SERVO_DEFAULT_ID, PRESS_UP_POSITION, 0, PRESS_UP_SPEED);
    debug_print_status("[STATE] -> The press moves up.\r\n", 1);

    latch = true;
    state.open = true;
    delay(1000);
  } else {
    latch = false;
    state.step = IDLE;
  }
}

static inline void state_reset(void) {

  static uint32_t previous_us = 0;

  if (state.buttons != RESET) previous_us = 0;
  else if (previous_us == 0) previous_us = state.time_us;

  const uint32_t elapsed_us = previous_us ? (state.time_us - previous_us) : 0;

  if ((previous_us == 0) || (elapsed_us < S_TO_US(4))) state.blink_us = HZ_TO_US(12);
  else if (elapsed_us < S_TO_US(5)) state.blink_us = HZ_TO_US(120);
  else {

    NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |  //
                                      (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

    NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    debug_print_status("[RESET] -> The system is reset.\r\n", 1000);

    __disable_irq();

    __DMB();
    __DSB();
    __ISB();

    __NVIC_SystemReset();
    while (true)
      ;
  }

  active_blink_status();
}

static inline void idle_detect(void) {

  if (!state.ranging) {
    sensor.VL53L4CD_StartRanging();
    state.ranging = true;
  }

  static VL53L4CD_RawResult_t result = { 0 };
  static uint8_t sensor_debounce = 0;

  uint8_t data_ready;
  if (!sensor.VL53L4CD_CheckForDataReady(&data_ready) && data_ready) {
    sensor.VL53L4CD_GetRawResult(&result);
    sensor.VL53L4CD_ClearInterrupt();

    const bool object_detected = (result.range_status == 9 && __builtin_bswap16(result.distance) < SENSOR_DISTANCE_MM);
    sensor_debounce = ((sensor_debounce << 1) | (!!object_detected));

    if ((sensor_debounce & SENSOR_DEBOUNCE_Msk) == SENSOR_DEBOUNCE_Msk) {
      sensor.VL53L4CD_ClearInterruptAndStopRanging();
      state.ranging = false;
      state.mode = AUTO;
      sensor_debounce = 0;

      debug_print_status("[MODE] -> Object detected. Changed to auto mode.\r\n", 1);
    }
  }
}

static inline void idle_power_save(void) {

  debug_print_status("[IDLE] -> The system saves power.\r\n", 1);

  Serial.flush();
  Serial.end();

  Wire.end();

  NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |  //
                                    (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

  NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
}

static inline void idle_shutdown(void) {

  debug_print_status("[IDLE] -> The system is turned off.\r\n", 1);

  Serial.flush();
  Serial.end();

  Wire.end();

  gpio_disconnect_system();

  NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |  //
                                    (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

  NRF_P1->PIN_CNF[GPIO_LEFT_BUTTON] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |     //
                                      (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |  //
                                      (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);

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

static void gpio_disconnect_system(void) {
  for (uint8_t i = 0; i < 32; i++) NRF_P0->PIN_CNF[i] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
  for (uint8_t i = 0; i < 16; i++) NRF_P1->PIN_CNF[i] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
}

static inline void active_enable_power(void) {

  NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                                    | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

  NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

  Wire.setPins(32 + I2C_SDA_PIN, 32 + I2C_CLK_PIN);
  Wire.begin();
  Wire.setClock(I2C_FREQUENCY_400K);

  Serial.begin(SERIAL_BAUDRATE_1M);

  for (uint8_t i = 0; i < 120; i++) {
    NRF_P0->OUT ^= (1UL << GPIO_STATUS_PIN);
    delayMicroseconds(8333);
  }

  sensor.VL53L4CD_SensorInit();
  sensor.VL53L4CD_StartRanging();

  // servo.EnableTorque(SERVO_DEFAULT_ID, true);

  state.ranging = true;
  state.active = true;
  state.idle_us = state.time_us;

  debug_print_status("[MODE] -> Changed to active mode.\r\n", 1);
}

static inline void active_blink_status(void) {
  static uint32_t time_us = 0;
  if ((int32_t)(state.time_us - time_us) >= 0) {
    NRF_P0->OUT ^= (1UL << GPIO_STATUS_PIN);
    if (state.blink_us) time_us += state.blink_us;
    else time_us += HZ_TO_US(12);
  }
}

static void debug_print_status(const char* status, const uint32_t delay_ms) {
  Serial.write(status);
  delay(delay_ms);
}
