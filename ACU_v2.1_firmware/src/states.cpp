#include "states.h"

// State machine variables
volatile ACU_STATE_t current_state = STATE_INIT;  // Current state of the ACU
volatile ACU_STATE_t previous_state = STATE_INIT; // Previous state of the ACU

volatile AS_STATE_t as_state = AS_STATE_OFF; // Autonomous system state

// INITIAL_SEQUENCE_STATE_t initial_sequence_state = WDT_TOOGLE_CHECK;
// INITIAL_SEQUENCE_STATE_t initial_sequence_state = PNEUMATIC_CHECK;
INITIAL_SEQUENCE_STATE_t initial_sequence_state = WDT_TOOGLE_CHECK;
current_mission_t current_mission = MANUAL; // Current mission state
current_mission_t jetson_mission = MANUAL;  // Mission state from Jetson

/**
 * @brief Timeout for WDT relay check.
 *
 * Used in initial_sequence() for timing WDT relay state.
 */
unsigned long wdt_relay_timout = 0;

/**
 * @brief Flag to enable WDT (Watchdog Timer) toggle.
 *
 * Used in loop() and initial_sequence() to control WDT toggling.
 */
volatile bool wdt_togle_enable = true;



/**
 * @brief Update the state of the ACU based on inputs and conditions
 */
void UpdateState(void) {
    // State transitions
  switch (current_state)
  {
  case STATE_INIT:
    as_state = AS_STATE_OFF; // Autonomous system state
    jetson_mission = MANUAL;
    break;

  case STATE_MISSION_SELECT:
    as_state = AS_STATE_OFF; // Autonomous system state
    break;

  case STATE_JETSONWAITING:
    as_state = AS_STATE_OFF; // Autonomous system state
    break;

  case STATE_INITIAL_SEQUENCE:
    break;

  case STATE_EBS_ERROR:
    as_state = AS_STATE_EMERGENCY; // Autonomous system state
    break;

  case STATE_READY:

    break;
  case STATE_DRIVING:
    as_state = AS_STATE_DRIVING;        // Autonomous system state
    digitalWrite(SOLENOID_REAR, HIGH);  // Activate rear solenoid
    digitalWrite(SOLENOID_FRONT, HIGH); // Activate front solenoid
    break;

  case STATE_EMERGENCY:
    emergency_flag = 1; // Set emergency flag
    as_state = AS_STATE_EMERGENCY;
    digitalWrite(SOLENOID_REAR, LOW); // Activate rear solenoid
    digitalWrite(SOLENOID_FRONT, LOW);
    break;

  case STATE_FINISHED:
    // as_state = AS_STATE_FINISHED; // Autonomous system state
    // Handle finished state if needed
    break;

  case STATE_MANUAL:
    as_state = AS_STATE_OFF;  // Manual state uses OFF autonomous system state
    current_mission = MANUAL; // Set current mission to MANUAL
    // Keep brakes applied in manual mode
    digitalWrite(SOLENOID_REAR, HIGH);
    digitalWrite(SOLENOID_FRONT, HIGH);
    break;
  }

  // Execute entry actions when state has changed
  if (current_state != previous_state)
  {
#ifdef print_state
    print_state_transition(previous_state, current_state);
#endif
    // State entry actions
    switch (current_state)
    {
    case STATE_INIT:
      as_state = AS_STATE_OFF; // Autonomous system state
      wdt_togle_enable = true; // Enable WDT toggle
      break;

    case STATE_MISSION_SELECT:
      as_state = AS_STATE_OFF; // Autonomous system state
      wdt_togle_enable = true; // Enable WDT toggle
      break;

    case STATE_JETSONWAITING:
      as_state = AS_STATE_OFF; // Autonomous system state
      Serial.println("Waiting for Jetson response (AS_STATE_OFF)");
      break;

    case STATE_INITIAL_SEQUENCE:
      // initial_sequence_state = WDT_TOOGLE_CHECK;
      //  reset all inital sequence variables
      break;

    case STATE_EBS_ERROR:
      as_state = AS_STATE_EMERGENCY;    // Autonomous system state
      digitalWrite(SOLENOID_REAR, LOW); // Activate rear solenoid
      digitalWrite(SOLENOID_FRONT, LOW);
      break;

    case STATE_READY:
      // as_state = AS_STATE_READY;
      /// digitalWrite(SOLENOID_REAR, LOW);  // Activate rear solenoid
      // digitalWrite(SOLENOID_FRONT, LOW); // Activate front solenoid
      break;

    case STATE_DRIVING:
      as_state = AS_STATE_DRIVING;
      // digitalWrite(SOLENOID_REAR, HIGH);  // Deactivate rear solenoid
      // digitalWrite(SOLENOID_FRONT, HIGH); // Deactivate front solenoid
      break;

    case STATE_EMERGENCY:
      ignition_enable = 0;
      as_state = AS_STATE_EMERGENCY;
      digitalWrite(SOLENOID_REAR, LOW); // Activate rear solenoid
      digitalWrite(SOLENOID_FRONT, LOW);
      emergency_timestamp = millis(); // Record the time of entering emergency state
      break;

    case STATE_FINISHED:
      // Handle finished state actions if needed
      break;

    case STATE_MANUAL:
      as_state = AS_STATE_OFF;           // Set autonomous system state to OFF
      digitalWrite(SOLENOID_REAR, LOW);  // Apply brakes
      digitalWrite(SOLENOID_FRONT, LOW); // Apply brakes
      Serial.println("Entered MANUAL state via CAN ignition signal");
      break;
    }

    // Store current state for change detection
    previous_state = current_state;
  }
}

/**
 * @brief Handle actions specific to the current state
 */
void HandleState(void) {
    /*if (asms_flag == LOW && current_state > STATE_MISSION_SELECT && current_state != STATE_EMERGENCY)
  {
    ignition_enable = 0;                  // Reset ignition enable flag
    current_state = STATE_MISSION_SELECT; // Transition to mission select state
  }*/

  if (asms_flag == LOW && current_state > STATE_MISSION_SELECT && current_state != STATE_MANUAL)
  {
    ignition_enable = 0;                  // Reset ignition enable flag
    current_state = STATE_MISSION_SELECT; // Transition to mission select state
    jetson_ready = 0;                     // Reset jetson ready flag
  }

  if (res_emergency == 1)
  {
    current_state = STATE_EMERGENCY;
    as_state = AS_STATE_EMERGENCY; // Autonomous system state
    return;
  }
  if (initial_sequence_state != WDT_STP_TOOGLE_CHECK && current_state >= STATE_INITIAL_SEQUENCE && current_state != STATE_MANUAL)
  {
    if (sdc_signal == 0)
    {
      current_state = STATE_EMERGENCY;
      emergency_flag = 1; // Set emergency flag
    }
  }

  if (current_state > STATE_INITIAL_SEQUENCE && current_state < STATE_EMERGENCY && current_state != STATE_MANUAL)
  {
    if (digitalRead(ASMS) == LOW && ignition_enable == 0)
    {
      ignition_enable = 1; // Enable ignition if ASMS is LOW
      Serial.println("ASMS is LOW, enabling ignition");
    }
  }
  if (current_state >= STATE_JETSONWAITING && current_state < STATE_EMERGENCY && current_state != STATE_MANUAL)
  {
    continuous_monitoring();
  }

  switch (current_state)
  {
  case STATE_INIT:
    // peripheral_init();
    as_state = AS_STATE_OFF; // Autonomous system state
    ignition_enable = 0;     // Reset ignition enable flag
    emergency_flag = 0;      // Reset emergency flag
    ignition_flag = 0;
    asms_flag = 0;
    jetson_mission = MANUAL;
    res_emergency = 0;
    wdt_togle_enable = true;

    // current_state = STATE_INITIAL_SEQUENCE; // Transition to initial sequence state
    current_state = STATE_MISSION_SELECT; // Transition to mission select state
                                          // current_state = STATE_INIT;
    break;

  case STATE_MISSION_SELECT:

    // Check for manual ignition from CAN
    emergency_flag = 0;
    if (IGN_manual == 1)
    {
      current_state = STATE_MANUAL;
      break;
    }

    static int last_button = 0;            // Start with LOW (pulldown default)
    static unsigned long ms_last_time = 0; // last timestamp

    if (digitalRead(ASMS) == LOW /*&& !res_active*/)
    {
      uint8_t current_button = digitalRead(MS_BUTTON1); // HIGH when pressed

      // 100 ms debunce
      if (last_button == LOW && current_button == HIGH && (millis() - ms_last_time) >= 400)
      {
        current_mission = (current_mission_t)(((int)current_mission + 1) % 7);
        ms_last_time = millis(); // start new lockout
        Serial.print("Mission changed to: ");
        Serial.println(current_mission);
      }

      last_button = current_button; // remember raw state
    }
    else
    {
      current_state = STATE_JETSONWAITING; // Transition to Jetson waiting state
      // initial_sequence_state = IGNITON; // Reset initial sequence state
      initial_sequence_state = WDT_TOOGLE_CHECK; // Reset initial sequence state
      // digitalWrite(SOLENOID_FRONT, LOW); // Activate front solenoid
      // digitalWrite(SOLENOID_REAR, LOW); // Activate rear solenoid
      digitalWrite(Debug_LED4, HIGH); // Indicate mission selection
    }
    break;

  case STATE_JETSONWAITING:
    // Wait for Jetson to respond with AS_STATE_OFF on CAN ID 0x503
    // The transition to STATE_INITIAL_SEQUENCE is handled in canISR() when AS_STATE_OFF is received
    // If ASMS is removed while waiting, return to mission select
    if (digitalRead(ASMS) == LOW)
    {
      current_state = STATE_MISSION_SELECT;
      Serial.println("ASMS removed while waiting for Jetson, returning to mission select");
    }
    break;

  case STATE_INITIAL_SEQUENCE:

    initial_sequence(); // Execute initial sequence actions
    break;

  case STATE_EBS_ERROR:
    current_state = STATE_EMERGENCY;
    break;

  case STATE_READY:
    /***TODO: Destava o cao
     *  Solenoids a 0
     *
     * ***/
    if (jetson_ready)
    {
      as_state = AS_STATE_READY; // Autonomous system state
      digitalWrite(SOLENOID_REAR, LOW);
      digitalWrite(SOLENOID_FRONT, LOW);
    }

    break;

  case STATE_DRIVING:
    /**
     * TODO: Release brakes
     * Solenoids a 0
     * AS_state = AS_STATE_DRIVING;
     */

  //  digitalWrite(SOLENOID_FRONT, HIGH);
  //  digitalWrite(SOLENOID_REAR, HIGH);
    as_state = AS_STATE_DRIVING;
    break;

  case STATE_EMERGENCY:
    // Handle emergency actions
    digitalWrite(SOLENOID_FRONT, LOW); // Activate front solenoid
    digitalWrite(SOLENOID_REAR, LOW);  // Activate rear solenoid
    emergency_flag = 1;                // Set emergency flag
    if (res_emergency == 0 && TANK_PRESSURE_FRONT < 1 && TANK_PRESSURE_REAR < 1 && ignition_flag == 0 && millis() - emergency_timestamp > 9000)
    {
      as_state = AS_STATE_OFF;
      current_state = STATE_INIT;
      Serial2.println("Emergency state timeout, returning to INIT state");
      emergency_flag = 0; // Reset emergency flag
    }
    else
    {
      Serial2.println("Emergency state active, waiting for AS response");
    }

    break;
  case STATE_FINISHED:
  #if FINISHED_TIME_FLAG
    // TODO: Make sue the ca is stopped
    if (first_finished_flag == 0)
    {
      first_finished_flag = 1;       // Set flag to indicate finished state has been entered
      finished_timestamp = millis(); // Record the time of entering finished state
    }
    if (millis() - finished_timestamp > FINISHED_TIMEOUT && first_finished_flag == 1)
    {
      digitalWrite(SOLENOID_REAR, LOW); // Activate rear solenoid
      digitalWrite(SOLENOID_FRONT, LOW);
      ignition_enable = 0; // Reset ignition enable flag
      if (millis() - finished_timestamp > FINISHED_TIMEOUT + 5000)
      {
        if (HYDRAULIC_PRESSURE_FRONT >= 9 * TANK_PRESSURE_FRONT && HYDRAULIC_PRESSURE_REAR >= 3.8 * TANK_PRESSURE_REAR)
        {
          as_state = AS_STATE_FINISHED;
        }
        else
        {
          current_state = STATE_EMERGENCY;
        }
      }
    }



#else
    static unsigned long rpm_zero_time = 0;
    static unsigned long state_change_time = 0;
    static volatile int flag_as_state = 0;

    if (rpm_vcu <= 500)
    {
      if (rpm_zero_time == 0)
      {
        rpm_zero_time = millis();
      }

      if (millis() - rpm_zero_time > 1000)
      {
        digitalWrite(SOLENOID_REAR, LOW); // Activate rear solenoid
        digitalWrite(SOLENOID_FRONT, LOW);
        ignition_enable = 0; // Reset ignition enable flag
        if (flag_as_state == 0)
        {
          flag_as_state = 1;
          state_change_time = millis(); // Record the time of entering finished state
        }
        if (millis() - state_change_time > 5000)
        {
          if (HYDRAULIC_PRESSURE_FRONT >= 9 * TANK_PRESSURE_FRONT && HYDRAULIC_PRESSURE_REAR >= 3.8 * TANK_PRESSURE_REAR)
          {
            as_state = AS_STATE_FINISHED;
          }
          else
          {
            current_state = STATE_EMERGENCY;
          }
        }
      }
    }
    else
    {
      rpm_zero_time = 0;
      state_change_time = 0;
    }
#endif // FINISHED_TIME_FLAG
    break;

  case STATE_MANUAL:
    // Handle manual state - return to mission select when IGN_manual becomes 0
    if (IGN_manual == 0)
    {
      current_state = STATE_MISSION_SELECT;
    }
    // In manual state, keep solenoids deactivated (brakes applied)
    digitalWrite(SOLENOID_FRONT, LOW);
    digitalWrite(SOLENOID_REAR, LOW);
    break;
  }
}

void initial_sequence() {
  switch (initial_sequence_state)
  {
  case WDT_TOOGLE_CHECK:
    if (digitalRead(SDC_FEEDBACK) == LOW)
    {
      initial_sequence_state = WDT_STP_TOOGLE_CHECK;
      wdt_togle_enable = false;    // Disable WDT toggle for initial sequence
      wdt_relay_timout = millis(); // Reset WDT toggle counter
    }
    break;

  case WDT_STP_TOOGLE_CHECK:
    if (SKIP_SDC_FEEDBACK)
    {
      initial_sequence_state = PNEUMATIC_CHECK; // Skip SDC feedback check
      wdt_togle_enable = true;                  // Enable WDT toggle
      break;
    }

    if (digitalRead(SDC_FEEDBACK) == HIGH)
    {
      initial_sequence_state = PNEUMATIC_CHECK;
      wdt_togle_enable = true;
      Serial.println("SDC feedback went HIGH, proceeding to PNEUMATIC_CHECK");
    }
    else
    {
      unsigned long elapsed = millis() - wdt_relay_timout;
      if (elapsed >= 5000)
      {
        Serial.println("SDC feedback timeout after " + String(elapsed) + "ms, going to ERROR");
        initial_sequence_state = ERROR;
      }
      else if (elapsed % 1000 == 0) // Print every second
      {
        Serial.println("Waiting for SDC feedback, elapsed: " + String(elapsed) + "ms, SDC: " + String(digitalRead(SDC_FEEDBACK)));
      }
    }
    break;

  case PNEUMATIC_CHECK:
    if (SKIP_PNEUMATIC_CHECK)
    {
      initial_sequence_state = PRESSURE_CHECK1;
      break;
    }

    if (TANK_PRESSURE_REAR > 6.0 && TANK_PRESSURE_FRONT > 6.0)
    {
      if (TANK_PRESSURE_REAR < 10.0 && TANK_PRESSURE_FRONT < 10.0)
      {
        initial_sequence_state = PRESSURE_CHECK1; // Transition to pressure check state
      }
      else
      {
        initial_sequence_state = ERROR; // Transition to pressure check state
      }
    }
    else
    {
      initial_sequence_state = ERROR; // Transition to pressure check state
    }
    break;

  case PRESSURE_CHECK1:
    if (SKIP_PRESSURE_CHECK1)
    {
      initial_sequence_state = IGNITON;
      break;
    }

    if (HYDRAULIC_PRESSURE_FRONT >= 9 * TANK_PRESSURE_FRONT && HYDRAULIC_PRESSURE_REAR >= 3.8 * TANK_PRESSURE_REAR)
    {
      initial_sequence_state = IGNITON;
    }
    else
    {
      Serial2.println("Pressure check failed: Front pressure: " + String(HYDRAULIC_PRESSURE_FRONT) + " bar, Rear pressure: " + String(HYDRAULIC_PRESSURE_REAR) + " bar");
      Serial2.println("Tank pressure front: " + String(TANK_PRESSURE_FRONT) + " bar, Rear pressure: " + String(TANK_PRESSURE_REAR) + " bar");
      initial_sequence_state = ERROR;
    }

    break;

  case IGNITON:
    ignition_enable = 1; // Enable ignition
    if (SKIP_IGNITION_CHECK)
    {
      // current_state = STATE_READY;
      initial_sequence_state = PRESSURE_CHECK_FRONT;
      pressure_check_delay = millis();
      break;
    }

    if (ignition_vcu == 1 && ignition_flag == 1)
    {
      // current_state = STATE_READY; //no final da initial sequence
      initial_sequence_state = PRESSURE_CHECK_FRONT; // Transition to pressure check state
      pressure_check_delay = millis();               // Reset pressure check delay
    }
    break;

  case PRESSURE_CHECK_REAR:
    digitalWrite(SOLENOID_REAR, HIGH); // Deactivate rear solenoid
    digitalWrite(SOLENOID_FRONT, LOW); // Activate front solenoid

    if (SKIP_PRESSURE_REAR_CHECK)
    {
      initial_sequence_state = PRESSURE_CHECK2;
      pressure_check_delay = millis();
      break;
    }

    if (HYDRAULIC_PRESSURE_REAR >= TANK_PRESSURE_REAR * 3 && HYDRAULIC_PRESSURE_FRONT <= 1 && millis() - pressure_check_delay >= 1000)
    {
      initial_sequence_state = PRESSURE_CHECK2;
      pressure_check_delay = millis(); // Reset pressure check delay
    }
    if (millis() - pressure_check_delay >= 5000)
    {                                 // Check if 500 ms has passed
      initial_sequence_state = ERROR; // Transition to error state if pressure check takes too long
      Serial.println("Pressure check failed: Front pressure: " + String(HYDRAULIC_PRESSURE_FRONT) + " bar, Rear pressure: " + String(HYDRAULIC_PRESSURE_REAR) + " bar");
      Serial.println("rear pressure check");
    }
    break;

  case PRESSURE_CHECK_FRONT:
    digitalWrite(SOLENOID_REAR, LOW);   // Deactivate rear solenoid
    digitalWrite(SOLENOID_FRONT, HIGH); // Activate front solenoid

    if (SKIP_PRESSURE_FRONT_CHECK)
    {
      initial_sequence_state = PRESSURE_CHECK_REAR;
      pressure_check_delay = millis();
      break;
    }

    if (HYDRAULIC_PRESSURE_FRONT >= TANK_PRESSURE_FRONT * 9 && HYDRAULIC_PRESSURE_REAR <= 1 && millis() - pressure_check_delay >= 1000)
    {
      // initial_sequence_state = PRESSURE_CHECK_REAR;
      initial_sequence_state = PRESSURE_CHECK_REAR; // Transition to pressure check state
      pressure_check_delay = millis();              // Reset pressure check delay
    }
    if (millis() - pressure_check_delay >= 5000)
    {                                 // Check if 500 ms has passed
      initial_sequence_state = ERROR; // Transition to error state if pressure check takes too long
      Serial.println("Pressure check failed: Front pressure: " + String(HYDRAULIC_PRESSURE_FRONT) + " bar, Rear pressure: " + String(HYDRAULIC_PRESSURE_REAR) + " bar");
      Serial.println("Tank pressure front: " + String(TANK_PRESSURE_FRONT) + " bar, Rear pressure: " + String(TANK_PRESSURE_REAR) + " bar");
      Serial.println("Front pressure check");
    }
    break;

  case PRESSURE_CHECK2:
    digitalWrite(SOLENOID_REAR, LOW);  // Deactivate rear solenoid
    digitalWrite(SOLENOID_FRONT, LOW); // Deactivate front solenoid

    if (SKIP_PRESSURE_CHECK2)
    {
      current_state = STATE_READY; // Transition to ready state
      // initial_sequence_state = STATE_READY; // Reset initial sequence state
      break;
    }

    if (HYDRAULIC_PRESSURE_REAR >= 3 * TANK_PRESSURE_REAR && HYDRAULIC_PRESSURE_FRONT >= 9 * TANK_PRESSURE_FRONT)
    {
      current_state = STATE_READY; // Transition to ready state
    }
    if (millis() - pressure_check_delay >= 5000)
    { // Check if 5000 ms has passed
      Serial2.println("Pressure check 2 failed: Front pressure: " + String(HYDRAULIC_PRESSURE_FRONT) + " bar, Rear pressure: " + String(HYDRAULIC_PRESSURE_REAR) + " bar");
      Serial2.println("Tank pressure front: " + String(TANK_PRESSURE_FRONT) + " bar, Rear pressure: " + String(TANK_PRESSURE_REAR) + " bar");
      Serial2.println("Initial sequence error: Pressure check 2 failed or timeout occurred");
      initial_sequence_state = ERROR; // Transition to error state if pressure check takes too long
    }
    break;

  case ERROR:
    current_state = STATE_EBS_ERROR; // Transition to EBS error state
    Serial2.println("Initial sequence error: Pressure check failed or timeout occurred");

    break;

  default:
    Serial.println("Unknown initial sequence state");
    break;
  }
}

void continuous_monitoring() {
  bool sdc_opened = digitalRead(SDC_FEEDBACK) == HIGH;
  if (sdc_opened)
  {
    delay(50);
    if (!(HYDRAULIC_PRESSURE_FRONT >= 60 && HYDRAULIC_PRESSURE_FRONT <= 120) || !(HYDRAULIC_PRESSURE_REAR >= 60 && HYDRAULIC_PRESSURE_REAR <= 120))
    {
      current_state = STATE_EBS_ERROR;
    }
    // stop monitoring
    current_state = STATE_INIT;
  }
  else if (!(TANK_PRESSURE_FRONT >= 4 && TANK_PRESSURE_FRONT <= 10) || !(TANK_PRESSURE_REAR >= 4 && TANK_PRESSURE_REAR <= 10))
  {
    current_state = STATE_EBS_ERROR;
  }
  if (current_state == STATE_DRIVING)
  {
    if (millis() - RES_timeout > CAN_TIMEOUT_TIME && millis() - JETSON_timeout > CAN_TIMEOUT_TIME && millis() - VCU_timeout > CAN_TIMEOUT_TIME /*&& millis()- MAXON_timeout > CAN_TIMEOUT_TIME*/)
    {
      current_state = STATE_EMERGENCY;
    }
  }
}

/**
 * @brief Print state transition for debugging
 * @param from Previous state
 * @param to New state
 */
void print_state_transition(ACU_STATE_t from, ACU_STATE_t to)
{
  Serial.println("\n\rState transition: " + String(state_names[from]) + " -> " + String(state_names[to]));
}

// State names for debug output
const char *state_names[] = {
    "STATE_INIT",
    "STATE_Mission_Select",
    "STATE_JETSONWAITING",
    "STATE_INITIAL_SEQUENCE",
    "STATE_READY",
    "STATE_DRIVING",
    "STATE_EBS_ERROR",
    "STATE_EMERGENCY",
    "STATE_FINISHED",
    "STATE_MANUAL"
};