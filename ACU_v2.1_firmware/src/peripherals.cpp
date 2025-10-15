#include "IntervalTimer.h"
#include "peripherals.h"
#include "definitions.h"

/**
 * @brief Timestamp for mission LED blinking.
 *
 * Used in Mission_Indicator() for timing mission LED blinking at 1 Hz when state >= STATE_INITIAL_SEQUENCE.
 */
unsigned long mission_LED_time = 0;

/**
 * @brief Current state of mission LED for blinking.
 *
 * Used in Mission_Indicator() to track whether the LED should be ON or OFF during blinking.
 */
bool mission_LED_state = true;

IntervalTimer PRESSURE_TIMER;
IntervalTimer CAN_TIMER;
IntervalTimer HANDBOOK_MESSAGE_TIMER;

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