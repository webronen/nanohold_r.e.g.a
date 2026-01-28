#include "main.h"

void setup() {

  NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |  //
                                    (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

  NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

  NRF_P1->PIN_CNF[GPIO_RIGHT_BUTTON] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |  //
                                       (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

  NRF_P0->PIN_CNF[GPIO_LEFT_BUTTON] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |  //
                                      (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    ;

  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->PRESCALER = 4;
  NRF_TIMER0->TASKS_START = 1;

  Wire.setPins(I2C_CLK_PIN, I2C_SDA_PIN);
  Wire.begin();
  Wire.setClock(I2C_FREQUENCY_400K);

  Serial.begin(SERIAL_BAUDRATE_1M);
  while (!Serial)
    ;
}

void loop() {

  NRF_TIMER0->TASKS_CAPTURE[0] = 1;
  state.time_us = NRF_TIMER0->CC[0];

  state.buttons = (PressStep)(((!(NRF_P0->IN & (1 << GPIO_LEFT_BUTTON))) << 1) |  //
                              ((!(NRF_P1->IN & (1 << GPIO_RIGHT_BUTTON))) << 0));

  if (state.open) {

    // static VL53L4CD_RawResult_t result = { 0 };
    // vl53l4cd.VL53L4CD_GetRawResult(&result);

    // static uint8_t object_samples = 0;

    // const bool object_detected =
    //   (result.range_status == 9u) && (__builtin_bswap16(result.distance) < AUTO_DISTANCE_MM);

    // if (object_detected) {
    //   vl53l4cd.VL53L4CD_ClearInterruptAndStopRanging();
    //   vl53l4cd.VL53L4CD_StartRanging();
    //   object_samples++;
    // } else
    //   object_samples = 0;

    // if (object_samples >= 5) {
    //   object_samples = 0;
    //   state.mode == AUTO;
    // }
  }

  // Priority: MANUAL > AUTO > IDLE
  if (state.buttons != IDLE) state.mode = MANUAL, state.step = state.buttons;
  else if (state.mode == AUTO && state.step == IDLE) state.step = UP;
  if (state.step != IDLE) prepare_active_state();

  switch (state.step) {
    case IDLE: handle_idle_state(); break;
    case UP: handle_press_up(); break;
    case DOWN: handle_press_down(); break;
    case RESET: handle_reset_state(); break;
    default: blink_status_leds(HZ_TO_US(12));
  }
}

static void blink_status_leds(const uint32_t interval_us) {

  static uint32_t previous_us = 0;

  if ((int32_t)(state.time_us - previous_us) >= 0) {
    NRF_P0->OUT ^= (1 << GPIO_STATUS_PIN);
    previous_us += interval_us;
  }
}

static inline void handle_reset_state(void) {

  static uint32_t previous_us = 0;

  if (state.buttons != RESET)
    previous_us = 0;
  else if (previous_us == 0)
    previous_us = state.time_us;

  const uint32_t elapsed_us = previous_us ? (state.time_us - previous_us) : 0;

  uint32_t interval_us;
  if ((previous_us == 0) || (elapsed_us < S_TO_US(4)))
    interval_us = HZ_TO_US(12);
  else if (elapsed_us < S_TO_US(5))
    interval_us = HZ_TO_US(120);
  else {
    NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |  //
                                      (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

    NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
    delay(1000);
    __DMB();
    __NVIC_SystemReset();
  }

  blink_status_leds(interval_us);
}

static inline void prepare_active_state(void) {

  state.idle_us = 0;

  if (!(NRF_P0->DIR & (1 << GPIO_STATUS_PIN))) {

    NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |  //
                                      (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

    NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

    // vl53l4cd.VL53L4CD_SensorInit();
    // vl53l4cd.VL53L4CD_StartRanging();

    Serial.println("Torque enabled!");
    // scs0009.EnableTorque(SCS0009_DEFAULT_ID, true);
  }
}

static inline void handle_idle_state(void) {

  if (state.idle_us == 0)
    state.idle_us = state.time_us;

  if ((int32_t)(state.time_us - state.idle_us) >= (int32_t)M_TO_US(1) &&  //
      NRF_P0->DIR & (1 << GPIO_STATUS_PIN)) {

    NRF_P0->PIN_CNF[LDO_ENABLE_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |  //
                                      (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

    NRF_P0->PIN_CNF[GPIO_STATUS_PIN] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
  }
}

static inline void handle_press_up(void) {

  NRF_P0->OUTSET = (1 << GPIO_STATUS_PIN);

  if (state.open)
    state.step = (state.mode == AUTO) ? DOWN : IDLE, state.latch = false;
  else if (!state.latch)
    state.latch = true, state.open = true, delay(1000);

  // if ((state.open = (scs0009.ReadPos(SCS0009_DEFAULT_ID) >= PRESS_UP_POSITION)))
  //   state.step = (state.mode == AUTO) ? DOWN : IDLE, state.latch = false;
  // else if (!state.latch)
  //   state.latch = true, scs0009.WritePos(SCS0009_DEFAULT_ID, PRESS_UP_POSITION, 0, PRESS_UP_SPEED);
}

static inline void handle_press_down(void) {

  NRF_P0->OUTCLR = (1 << GPIO_STATUS_PIN);

  if (!state.open)
    state.mode = MANUAL, state.step = IDLE, state.latch = false;
  else if (!state.latch)
    state.latch = true, state.open = false, delay(1000);

  // if (!(state.open = !(scs0009.ReadLoad(SCS0009_DEFAULT_ID) >= PRESS_LOAD_LIMIT)))
  //   state.mode = MANUAL, state.step = IDLE, state.latch = false;
  // else if (!state.latch)
  //   state.latch = true, scs0009.WritePos(SCS0009_DEFAULT_ID, PRESS_DOWN_POSITION, 0, PRESS_DOWN_SPEED);
}
