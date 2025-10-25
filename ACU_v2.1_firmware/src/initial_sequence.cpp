#include <Arduino.h>
#include "initial_sequence.h"
#include "definitions.h"
#include "globals.h"

void initial_sequence()
{
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