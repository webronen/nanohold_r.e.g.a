#include "main.h"

void setup() {

  NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |  //
                                    (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

  NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

  NRF_P1->PIN_CNF[GPIO_LEFT_BUTTON] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |     //
                                      (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |  //
                                      (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);

  NRF_P0->PIN_CNF[GPIO_RIGHT_BUTTON] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |  //
                                       (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    ;

  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->PRESCALER = 4;
  NRF_TIMER0->TASKS_START = 1;

  Wire.setPins(I2C_SDA_PIN, I2C_CLK_PIN);
  Wire.begin();
  Wire.setClock(I2C_FREQUENCY_400K);

  Serial.begin(SERIAL_BAUDRATE_1M);
  while (!Serial)
    ;
}

void loop() {

  NRF_TIMER0->TASKS_CAPTURE[0] = 1;
  state.time_us = NRF_TIMER0->CC[0];

  state.buttons = (PressStep_t)(((!(NRF_P1->IN & (1 << GPIO_LEFT_BUTTON))) << 1) |  //
                                ((!(NRF_P0->IN & (1 << GPIO_RIGHT_BUTTON))) << 0));

  static uint8_t button_debounce = 0;
  button_debounce = (button_debounce << 1) | (state.buttons ? 1 : 0);
  if ((button_debounce & BUTTON_DEBOUNCE_MASK) != BUTTON_DEBOUNCE_MASK) state.buttons = IDLE;

  if (state.buttons != IDLE) state.mode = MANUAL, state.step = state.buttons;
  else if (state.mode == AUTO && state.step == IDLE) state.step = UP;
  if (state.step != IDLE) prepare_active_state();

  state_handle[state.step < HALT ? state.step : HALT]();
}

static inline void prepare_active_state(void) {

  state.idle_us = 0;

  if (!(NRF_P0->DIR & (1 << GPIO_STATUS_PIN))) {

    NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |  //
                                      (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

    NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

    for (uint8_t i = 0; i < 120; i++) {
      NRF_P0->OUT ^= (1 << GPIO_STATUS_PIN);
      delayMicroseconds(8333);
    }

    // sensor.VL53L4CD_SensorInit();
    // sensor.VL53L4CD_StartRanging();

    // servo.EnableTorque(SERVO_DEFAULT_ID, true);
    Serial.println("[ACTIVE] -> Switched to active mode.");
  }
}

static inline void handle_idle_state(void) {

  // if (state.open) {
  //   static VL53L4CD_RawResult_t result = { 0 };
  //   static uint8_t sensor_debounce = 0;

  //   uint8_t data_ready;
  //   if (!sensor.VL53L4CD_CheckForDataReady(&data_ready) && data_ready) {
  //     sensor.VL53L4CD_GetRawResult(&result);
  //     sensor.VL53L4CD_ClearInterruptAndStopRanging();
  //     sensor.VL53L4CD_StartRanging();

  //     const bool object_detected = (result.range_status == 9 && __builtin_bswap16(result.distance) < SENSOR_DISTANCE_MM);
  //     sensor_debounce = (sensor_debounce << 1) | (object_detected ? 1 : 0);

  //     if ((sensor_debounce & SENSOR_DEBOUNCE_MASK) == SENSOR_DEBOUNCE_MASK) {
  //       state.mode = AUTO;
  //       sensor_debounce = 0;
  //       Serial.println("Auto: Object detected!");
  //     }
  //   }
  // }

  if (state.idle_us == 0) {
    state.idle_us = state.time_us;
    Serial.println("[IDLE] -> Switched to idle mode.");
  }

  if ((int32_t)(state.time_us - state.idle_us) >= (int32_t)IDLE_TIMEOUT_M &&  //
      (NRF_P0->DIR & (1 << GPIO_STATUS_PIN))) {

    NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |  //
                                      (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

    NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    Serial.println("[IDLE] -> Turning off connected devices.");
  }

  if ((int32_t)(state.time_us - state.idle_us) >= (int32_t)SLEEP_TIMEOUT_M &&  //
      (NRF_P0->PIN_CNF[GPIO_STATUS_PIN] & GPIO_PIN_CNF_INPUT_Msk) == GPIO_PIN_CNF_INPUT_Msk) {
    // TODO: Implement full system shutdown (~3µA)

    Serial.println("[IDLE] -> Shutting down the system.");
    Serial.flush();
    delay(1000);

    __DMB();
    __DSB();
    __ISB();
    NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
    while (true)
      ;
  }
}

static inline void handle_down_state(void) {

  NRF_P0->OUTCLR = (1 << GPIO_STATUS_PIN);

  if (!state.open) {
    state.mode = MANUAL;
    state.step = IDLE;
    state.latch = false;
  } else if (!state.latch) {
    state.latch = true;
    state.open = false;
    printf("[%s DOWN] -> Moving press down.\n", (state.mode == AUTO) ? "AUTO" : "MANUAL");
    delay(1);
  }

  // if (!(state.open = !(servo.ReadLoad(SERVO_DEFAULT_ID) >= PRESS_LOAD_LIMIT)))
  //   state.mode = MANUAL, state.step = IDLE, state.latch = false;
  // else if (!state.latch)
  //   state.latch = true, servo.WritePos(SERVO_DEFAULT_ID, PRESS_DOWN_POSITION, 0, PRESS_DOWN_SPEED);
}

static inline void handle_up_state(void) {

  NRF_P0->OUTSET = (1 << GPIO_STATUS_PIN);

  if (state.open) {
    state.step = (state.mode == AUTO) ? DOWN : IDLE;
    state.latch = false;
  } else if (!state.latch) {
    state.latch = true;
    state.open = true;
    printf("[%s UP] -> Moving press up.\n", (state.mode == AUTO) ? "AUTO" : "MANUAL");
    delay(1);
  }

  // if ((state.open = (servo.ReadPos(SERVO_DEFAULT_ID) >= PRESS_UP_POSITION)))
  //   state.step = (state.mode == AUTO) ? DOWN : IDLE, state.latch = false;
  // else if (!state.latch)
  //   state.latch = true, servo.WritePos(SERVO_DEFAULT_ID, PRESS_UP_POSITION, 0, PRESS_UP_SPEED);
}

static inline void handle_reset_state(void) {

  static uint32_t previous_us = 0;

  if (state.buttons != RESET) {
    previous_us = 0;
  } else if (previous_us == 0)
    previous_us = state.time_us;

  const uint32_t elapsed_us = previous_us ? (state.time_us - previous_us) : 0;

  if ((previous_us == 0) || (elapsed_us < S_TO_US(4)))
    state.halt_us = HZ_TO_US(12);
  else if (elapsed_us < S_TO_US(5))
    state.halt_us = HZ_TO_US(120);
  else {
    NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |  //
                                      (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

    NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    Serial.println("[RESET] -> Resetting the system.");
    Serial.flush();
    delay(1000);

    __DMB();
    __DSB();
    __ISB();
    __NVIC_SystemReset();
    while (true)
      ;
  }

  handle_halt_state();
}

static void handle_halt_state(void) {

  static uint32_t previous_us = 0;

  if ((int32_t)(state.time_us - previous_us) >= 0) {
    NRF_P0->OUT ^= (1 << GPIO_STATUS_PIN);
    previous_us += state.halt_us == 0 ? HZ_TO_US(12) : state.halt_us;
  }
}
