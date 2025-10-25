#include <Arduino.h>
#include "sensors.h"
#include "definitions.h"
#include "globals.h"

/**
 * @brief Read pressure sensors and update system state
 */
void Pressure_readings()
{

  EBS_TANK_PRESSURE_A_values[adc_pointer] = analogRead(EBS_TANK_PRESSURE_A);
  EBS_TANK_PRESSURE_B_values[adc_pointer] = analogRead(EBS_TANK_PRESSURE_B);
  adc_pointer++;
  if (adc_pointer >= PRESSURE_READINGS)
  {
    adc_pointer = 0;
  }
  update_median_flag = true; // Set flag to update median values
}

void median_pressures()
{

  // Calculate median for tank pressure B
  float sum = 0;
  for (int i = 0; i < PRESSURE_READINGS; i++)
  {
    sum += EBS_TANK_PRESSURE_B_values[i];
  }
  TANK_PRESSURE_FRONT = sum / PRESSURE_READINGS;

  // Store raw voltage for debugging (convert ADC to voltage)
  float rawVoltage = TANK_PRESSURE_FRONT * 3.3 / 1023; // Read raw voltage from the analog pin

  // Use corrected divider value (0.85 instead of 0.66) prev val 0.476
  float actualVoltage = rawVoltage / 0.66;

  TANK_PRESSURE_FRONT = (actualVoltage - 0.5) / 0.4;

  // tank pressure A

  sum = 0;
  for (int i = 0; i < PRESSURE_READINGS; i++)
  {
    sum += EBS_TANK_PRESSURE_A_values[i];
  }
  TANK_PRESSURE_REAR = sum / PRESSURE_READINGS;

  // Store raw voltage for debugging (convert ADC to voltage)
  rawVoltage = TANK_PRESSURE_REAR * 3.3 / 1023; // Read raw voltage from the analog pin

  // Use corrected divider value (0.85 instead of 0.66) prev val 0.476
  actualVoltage = rawVoltage / 0.66;

  // Apply formula ONCE with corrected divider
  TANK_PRESSURE_REAR = (actualVoltage - 0.5) / 0.4;

  // Serial.println("Tank pressure front: " + String(TANK_PRESSURE_FRONT) + " bar");
  // Serial.println("Tank pressure rear: " + String(TANK_PRESSURE_REAR) + " bar");
}

void continuous_monitoring()
{
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