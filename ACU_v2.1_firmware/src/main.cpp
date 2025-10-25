/**
 * @file main.cpp
 * @brief Main firmware file for the Actuation Control Unit (ACU) V2.1.
 *
 * This file implements the main logic, state machine, and peripheral handling for the ACU,
 * which manages the actuation and safety logic for an autonomous vehicle.
 * The ACU interfaces with pressure sensors, solenoids, CAN bus, and various status indicators.
 *
 * Key features:
 * - Implements a robust state machine for ACU operation, including initialization, mission selection,
 *   initial safety checks, ready, driving, emergency, and finished states.
 * - Handles CAN communication for receiving commands and sending status updates.
 * - Reads and processes pressure sensor data with averaging and conversion to engineering units.
 * - Manages ignition and emergency logic with debounce and safety checks.
 * - Controls visual indicators (LEDs) for system and mission status.
 * - Provides detailed documentation for each function and state.
 *
 * @note Key global variables used throughout this file include:
 *   - current_state, previous_state: ACU state machine tracking.
 *   - as_state: Autonomous system state.
 *   - initial_sequence_state: State for initial safety sequence.
 *   - current_mission, jetson_mission: Mission selection tracking.
 *   - EBS_TANK_PRESSURE_A_values, EBS_TANK_PRESSURE_B_values: Pressure sensor readings.
 *   - TANK_PRESSURE_FRONT, TANK_PRESSURE_REAR: Calculated tank pressures.
 *   - HYDRAULIC_PRESSURE_FRONT, HYDRAULIC_PRESSURE_REAR: Hydraulic pressures.
 *   - ignition_flag, ignition_vcu, ignition_enable: Ignition logic.
 *   - asms_flag: Autonomous system master switch state.
 *   - emergency_flag, res_emergency, res_active: Emergency logic.
 *   - wdt_togle_enable, wdt_togle_counter, wdt_relay_timout: Watchdog timer logic.
 *   - pressure_check_delay: Timing for pressure checks.
 *   - emergency_timestamp: Timing for emergency state.
 *   - ASSI_YELLOW_time, ASSI_BLUE_time: Timing for LED indicators.
 *   - last_button_time_ms: Debounce for mission selection button.
 *   - last_ign_state, debounced_ign_state: Debounce for ignition input.
 *
 * @author (Bruno Vicente - LART)
 * @date (2025)
 */
#include <Arduino.h>
#include "definitions.h"
#include "globals.h"
#include "state_machine.h"
#include "peripherals.h"
#include "sensors.h"
#include "canbus.h"
#include "initial_sequence.h"

void setup()
{
  peripheral_init(); // Inicializa periféricos e timers
  // delay(3000);
}

void loop()
{
  UpdateState();

  /* Handle actions specific to the current state */
  HandleState();

  led_heartbit();

  Mission_Indicator();

  if (update_median_flag)
  {
    median_pressures();         // Read pressure values
    update_median_flag = false; // Reset flag after reading
  }
  asms_flag = digitalRead(ASMS); // Read ASMS state
  ASSI();                        // Update ASSI state

  if (wdt_togle_enable)
  {
    if (millis() - wdt_togle_counter >= 10)
    {                                       // Check if 10 ms has passed
      digitalWrite(WDT, !digitalRead(WDT)); // Toggle WDT pin
      wdt_togle_counter = millis();         // Reset counter
    }
  }
  check_ignition(); // Check ignition state
}