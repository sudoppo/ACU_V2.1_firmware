#include <Arduino.h>
#include "peripherals.h"
#include "canbus.h"
#include "sensors.h"
#include "definitions.h"
#include "globals.h"
#include "FlexCAN_T4_.h"
#include "IntervalTimer.h"
#include "autonomous_temporary.h"

void peripheral_init() {
    pinMode(YELLOW_LEDS, OUTPUT);
  pinMode(BLUE_LEDS, OUTPUT);

  pinMode(MS_BUTTON1, INPUT);

  pinMode(MS_LED_TRACKD, OUTPUT);
  pinMode(MS_LED_ACCL, OUTPUT);
  pinMode(MS_LED_SKIDPAD, OUTPUT);
  pinMode(MS_LED_MANUEL, OUTPUT);
  pinMode(MS_LED_INSPCT, OUTPUT);
  pinMode(MS_LED_AUTOCRSS, OUTPUT);
  pinMode(MS_LED_EBS, OUTPUT);
  digitalWrite(MS_LED_TRACKD, 1);
  digitalWrite(MS_LED_ACCL, 1);
  digitalWrite(MS_LED_SKIDPAD, 1);
  digitalWrite(MS_LED_MANUEL, 1);
  digitalWrite(MS_LED_INSPCT, 1);
  digitalWrite(MS_LED_AUTOCRSS, 1);
  digitalWrite(MS_LED_EBS, 1);
  pinMode(AS_SW, INPUT);

  pinMode(HB_LED, OUTPUT);
  pinMode(Debug_LED2, OUTPUT);
  pinMode(Debug_LED3, OUTPUT);
  pinMode(Debug_LED4, OUTPUT);
  pinMode(Debug_LED5, OUTPUT);
  pinMode(Debug_LED6, OUTPUT);

  pinMode(EBS_TANK_PRESSURE_A, INPUT);
  pinMode(EBS_TANK_PRESSURE_B, INPUT);
  pinMode(EBS_VALLVE_A, INPUT);
  pinMode(EBS_VALLVE_B, INPUT);

  pinMode(R2D_PIN, INPUT);

  // pinMode(LED_PIN, OUTPUT);
  pinMode(WDT, OUTPUT);

  pinMode(ASMS, INPUT);
  pinMode(IGN_PIN, INPUT);

  pinMode(SOLENOID_FRONT, OUTPUT);
  pinMode(SOLENOID_REAR, OUTPUT);

  digitalWrite(SOLENOID_FRONT, 0); // 1 equals braking off
  digitalWrite(SOLENOID_REAR, 0);

  pinMode(SDC_FEEDBACK, INPUT);

  Serial2.begin(115200);
  Serial.begin(115200);

  CAN.begin();
  CAN.setBaudRate(1000000); // Set CAN baud rate to 1 Mbps
  CAN.setMaxMB(16);         // Set maximum number of mailboxes
  // CAN.setMB(MB4,RX,STD);
  // CAN.setMB(MB5,RX,STD);
  // CAN.setMBFilter(REJECT_ALL);

  /*CAN.setMBFilter(MB0, AUTONOMOUS_TEMPORARY_JETSON_MS_FRAME_ID);
  CAN.setMBFilter(MB1, AUTONOMOUS_TEMPORARY_AS_STATE_FRAME_ID);
  CAN.setMBFilter(MB2, AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID);
  CAN.setMBFilter(MB3, AUTONOMOUS_TEMPORARY_RES_FRAME_ID);
  CAN.setMBFilter(MB4, 0x446);
  CAN.setMBFilter(MB5, 0x546);*/

  CAN.enableMBInterrupts(); // Enable mailbox interrupts added 10 july 2025
  CAN.onReceive(canISR);

  PRESSURE_TIMER.begin(Pressure_readings, 100000); // 100ms

  CAN_TIMER.begin(send_can_msg, 100000); // 100ms

  Serial.println("Peripheral initialization complete");
  digitalWrite(Debug_LED2, 1); // Indicate initialization complete

    HANDBOOK_MESSAGE_TIMER.begin(send_handbook_variables, 100000); // 100ms
}

void led_heartbit()
{
  if (HeartBit + 500 <= millis())
  {
    digitalWrite(HB_LED, !digitalRead(HB_LED)); // Toggle LED state
    HeartBit = millis();
    Serial.print("current_state: ");
    Serial.println(current_state);
    Serial.print("as_state: ");
    Serial.println(as_state);
    Serial.print("initial_sequence_state: ");
    Serial.println(initial_sequence_state);
    Serial.print("SDC feedback: ");
    Serial.println(digitalRead(SDC_FEEDBACK));
    Serial.print("EBS Tank Pressure rear: ");
    Serial.println(TANK_PRESSURE_REAR);
    Serial.print("EBS Tank Pressure front: ");
    Serial.println(TANK_PRESSURE_FRONT);
    Serial.print("Hydraulic Pressure rear: ");
    Serial.println(HYDRAULIC_PRESSURE_REAR);
    Serial.print("Hydraulic Pressure front: ");
    Serial.println(HYDRAULIC_PRESSURE_FRONT);
  }
}

/**
 * @brief Controls the state of the YELLOW_LEDS and BLUE_LEDS based on the current as_state.
 *
 * This function manages the visual indication of the system's state by toggling or setting
 * the YELLOW_LEDS and BLUE_LEDS according to the value of the as_state variable. The behavior
 * for each state is as follows:
 * - AS_STATE_OFF: Turns off both YELLOW_LEDS and BLUE_LEDS.
 * - AS_STATE_READY: Turns on YELLOW_LEDS and turns off BLUE_LEDS.
 * - AS_STATE_DRIVING: Toggles YELLOW_LEDS every 500 ms, keeps BLUE_LEDS off.
 * - AS_STATE_EMERGENCY: Toggles BLUE_LEDS every 500 ms, keeps YELLOW_LEDS off.
 * - AS_STATE_FINISHED: Turns off YELLOW_LEDS and turns on BLUE_LEDS.
 * - Default: Turns off both YELLOW_LEDS and BLUE_LEDS.
 *
 * Timing for toggling is managed using ASSI_YELLOW_time and ASSI_BLUE_time variables.
 *
 * @note This function assumes that as_state, ASSI_YELLOW_time, and ASSI_BLUE_time are
 *       defined and accessible in the current scope, and that digitalWrite, digitalRead,
 *       and millis functions are available (e.g., in an Arduino environment).
 */
void ASSI()
{

  switch (as_state)
  {
  case AS_STATE_OFF:
    digitalWrite(YELLOW_LEDS, LOW);
    digitalWrite(BLUE_LEDS, LOW);

    break;
  case AS_STATE_READY:

    digitalWrite(YELLOW_LEDS, HIGH);
    digitalWrite(BLUE_LEDS, LOW);
    break;
  case AS_STATE_DRIVING:
    digitalWrite(BLUE_LEDS, LOW);
    if (millis() - ASSI_YELLOW_time >= 500)
    {
      ASSI_YELLOW_time = millis();
      digitalWrite(YELLOW_LEDS, !digitalRead(YELLOW_LEDS));
    }
    break;
  case AS_STATE_EMERGENCY:
    digitalWrite(YELLOW_LEDS, LOW);
    if (millis() - ASSI_BLUE_time >= 500)
    {
      ASSI_BLUE_time = millis();
      digitalWrite(BLUE_LEDS, !digitalRead(BLUE_LEDS));
    }
    break;
  case AS_STATE_FINISHED:
    digitalWrite(YELLOW_LEDS, LOW);
    digitalWrite(BLUE_LEDS, HIGH);
    break;

  default:
    digitalWrite(YELLOW_LEDS, LOW);
    digitalWrite(BLUE_LEDS, LOW);

    break;
  }
}

/**
 * @brief Updates the mission indicator LEDs based on the current mission state.
 *
 * This function sets the state of each mission status LED (MS_LED1 to MS_LED7)
 * to indicate the currently active mission. Each mission mode corresponds to a unique
 * LED pattern, where one LED is turned on to represent the active mission,
 * and the others are turned off (logic HIGH). If the mission state is not recognized,
 * all LEDs are turned off by default.
 *
 * When the current state is >= STATE_INITIAL_SEQUENCE, the active mission LED blinks
 * at 1 Hz frequency. When the state is below STATE_INITIAL_SEQUENCE, the LED stays solid.
 */
void Mission_Indicator()
{
  // Determine if we should blink (state >= STATE_INITIAL_SEQUENCE)
  bool should_blink = (current_state >= STATE_INITIAL_SEQUENCE);
  bool led_output = true; // Default to ON for solid state

  if (should_blink)
  {
    // Check if 500ms have passed (for 1 Hz blinking: 500ms ON, 500ms OFF)
    if (millis() - mission_LED_time >= 500)
    {
      mission_LED_time = millis();
      mission_LED_state = !mission_LED_state; // Toggle the LED state
    }
    led_output = mission_LED_state;
  }
  else
  {
    // For solid state, always ON and reset blinking state
    led_output = true;
    mission_LED_state = true;
    mission_LED_time = millis();
  }

  // First turn off all LEDs
  digitalWrite(MS_LED_TRACKD, 0);
  digitalWrite(MS_LED_ACCL, 0);
  digitalWrite(MS_LED_SKIDPAD, 0);
  digitalWrite(MS_LED_MANUEL, 0);
  digitalWrite(MS_LED_INSPCT, 0);
  digitalWrite(MS_LED_AUTOCRSS, 0);
  digitalWrite(MS_LED_EBS, 0);

  // Then turn on the appropriate LED based on current mission
  switch (current_mission)
  {
  case MANUAL:
    digitalWrite(MS_LED_MANUEL, led_output ? 1 : 0);
    break;
  case ACCELERATION:
    digitalWrite(MS_LED_ACCL, led_output ? 1 : 0);
    break;
  case SKIDPAD:
    digitalWrite(MS_LED_SKIDPAD, led_output ? 1 : 0);
    break;
  case TRACKDRIVE:
    digitalWrite(MS_LED_TRACKD, led_output ? 1 : 0);
    break;
  case EBS_TEST:
    digitalWrite(MS_LED_EBS, led_output ? 1 : 0);
    break;
  case INSPECTION:
    digitalWrite(MS_LED_INSPCT, led_output ? 1 : 0);
    break;
  case AUTOCROSS:
    digitalWrite(MS_LED_AUTOCRSS, led_output ? 1 : 0);
    break;
  default:
    // All LEDs already turned off above
    break;
  }
}

/**
 * @brief Debounces and checks the ignition input signal.
 *
 * This function reads the current state of the ignition pin (IGN_PIN) and applies
 * a debounce algorithm to filter out spurious changes due to mechanical switch noise.
 * It updates the ignition_flag based on the debounced state and the ignition_enable flag.
 *
 *@note Variables used:
 *@note - last_debounce_time (static): Stores the last time the ignition input changed, used for debouncing.
 *@note - debounce_delay (const): The debounce interval in milliseconds.
 *@note - current_state: The current raw reading from the ignition pin.
 *@note - last_ign_state (external): The last raw state read from the ignition pin.
 *@note - debounced_ign_state (external): The last debounced state of the ignition pin.
 *@note - ignition_flag (external): Set to 1 if ignition is ON and enabled, otherwise 0.
 *@note - ignition_enable (external): Enables or disables the ignition logic.
 *
 * The function ensures that ignition_flag is set only if the ignition input is HIGH
 * and ignition_enable is true, providing reliable ignition state detection.
 */
void check_ignition()
{
  static unsigned long last_debounce_time = 0;
  const unsigned long debounce_delay = 30; // 30 ms debounce

  uint8_t current_state = digitalRead(IGN_PIN);

  if (current_state != last_ign_state)
  {
    last_debounce_time = millis();
    last_ign_state = current_state;
  }

  if ((millis() - last_debounce_time) > debounce_delay)
  {
    if (debounced_ign_state != current_state)
    {
      debounced_ign_state = current_state;
      ignition_flag = (debounced_ign_state == HIGH) ? 1 : 0;
    }
  }
  ignition_flag = ignition_flag && ignition_enable; // Ensure ignition flag is set only if ignition is enabled
}