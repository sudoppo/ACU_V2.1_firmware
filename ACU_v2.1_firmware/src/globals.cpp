#include "globals.h"
#include "IntervalTimer.h"
#include "FlexCAN_T4_.h"

int first_finished_flag = 0; // Flag to indicate if the finished state has been entered
unsigned long finished_timestamp = 0; // Timestamp for finished state

IntervalTimer PRESSURE_TIMER;
IntervalTimer CAN_TIMER;
IntervalTimer HANDBOOK_MESSAGE_TIMER;

FlexCAN_T4<CAN2, RX_SIZE_1024, TX_SIZE_1024> CAN;

// State machine variables
volatile ACU_STATE_t current_state = STATE_INIT;  // Current state of the ACU
volatile ACU_STATE_t previous_state = STATE_INIT; // Previous state of the ACU
volatile AS_STATE_t as_state = AS_STATE_OFF; // Autonomous system state

// INITIAL_SEQUENCE_STATE_t initial_sequence_state = WDT_TOOGLE_CHECK;
// INITIAL_SEQUENCE_STATE_t initial_sequence_state = PNEUMATIC_CHECK;
INITIAL_SEQUENCE_STATE_t initial_sequence_state = WDT_TOOGLE_CHECK;
current_mission_t current_mission = MANUAL; // Current mission state
current_mission_t jetson_mission = MANUAL;  // Mission state from Jetson


// VARIABLES
unsigned long HeartBit = 0;

/**
 * @brief Array storing recent pressure readings from EBS Tank A.
 *
 * This array holds the last PRESSURE_READINGS number of float values,
 * representing the sampled pressure values from the EBS (Emergency Braking System) Tank A.
 * Updated in Pressure_readings(), used in median_pressures() for filtering/averaging.
 *
 * @see EBS_TANK_PRESSURE_B_values
 * @see PRESSURE_READINGS
 */
float EBS_TANK_PRESSURE_A_values[PRESSURE_READINGS];

/**
 * @brief Array storing recent pressure readings from EBS Tank B.
 *
 * This array holds the last PRESSURE_READINGS number of float values,
 * representing the sampled pressure values from the EBS (Emergency Braking System) Tank B.
 * Updated in Pressure_readings(), used in median_pressures() for filtering/averaging.
 *
 * @see EBS_TANK_PRESSURE_A_values
 * @see PRESSURE_READINGS
 */
float EBS_TANK_PRESSURE_B_values[PRESSURE_READINGS];

/**
 * @brief Pressure value for front tank (in bar).
 *
 * Calculated in median_pressures() from EBS_TANK_PRESSURE_B_values.
 * Used in initial_sequence(), HandleState(), send_can_msg(), and for state transitions.
 */
float TANK_PRESSURE_FRONT = 0;

/**
 * @brief Pressure value for rear tank (in bar).
 *
 * Calculated in median_pressures() from EBS_TANK_PRESSURE_A_values.
 * Used in initial_sequence(), HandleState(), send_can_msg(), and for state transitions.
 */
float TANK_PRESSURE_REAR = 0;

/**
 * @brief Hydraulic pressure value for front brakes (in bar).
 *
 * Updated in canISR() from CAN message AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID.
 * Used in initial_sequence() for pressure checks.
 */
float HYDRAULIC_PRESSURE_FRONT = 0;

/**
 * @brief Hydraulic pressure value for rear brakes (in bar).
 *
 * Updated in canISR() from CAN message AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID.
 * Used in initial_sequence() for pressure checks.
 */
float HYDRAULIC_PRESSURE_REAR = 0;

/**
 * @brief Pointer for pressure readings buffer.
 *
 * Used in Pressure_readings() to cycle through EBS_TANK_PRESSURE_A_values and EBS_TANK_PRESSURE_B_values.
 */
uint8_t adc_pointer = 0;

/**
 * @brief Flag to indicate if median pressure update is needed.
 *
 * Set in Pressure_readings(), checked in loop() to call median_pressures().
 */
volatile bool update_median_flag = false;

/**
 * @brief Flag to indicate ignition signal.
 *
 * Updated in check_ignition(), used in send_can_msg(), HandleState(), and initial_sequence().
 */
uint8_t ignition_flag = 0;

/**
 * @brief Ignition signal state from VCU.
 *
 * Updated in canISR() from CAN message AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID.
 * Used in initial_sequence().
 */
uint8_t ignition_vcu = 0;

/**
 * @brief Current ASMS (Autonomous System Master Switch) signal state.
 *
 * Updated in loop() from digitalRead(ASMS), used in send_can_msg().
 */
uint8_t asms_flag = 0;

/**
 * @brief Flag to indicate emergency state.
 *
 * Set in HandleState() and UpdateState(), used in send_can_msg().
 */
uint8_t emergency_flag = 0;

/**
 * @brief Emergency response from AS (Autonomous System).
 *
 * Updated in canISR() from CAN message AUTONOMOUS_TEMPORARY_RES_FRAME_ID.
 * Used in HandleState() and UpdateState().
 */
volatile uint8_t res_emergency = 0;

/**
 * @brief Flag to enable WDT (Watchdog Timer) toggle.
 *
 * Used in loop() and initial_sequence() to control WDT toggling.
 */
volatile bool wdt_togle_enable = true;

/**
 * @brief Counter for WDT toggle timing.
 *
 * Used in loop() and initial_sequence() to measure elapsed time for WDT toggling.
 */
unsigned long wdt_togle_counter = 0;

/**
 * @brief Timeout for WDT relay check.
 *
 * Used in initial_sequence() for timing WDT relay state.
 */
unsigned long wdt_relay_timout = 0;

/**
 * @brief Delay for pressure check timing.
 *
 * Used in initial_sequence() to measure elapsed time for pressure checks.
 */
unsigned long pressure_check_delay = 0;

/**
 * @brief Flag to enable ignition logic.
 *
 * Set in initial_sequence(), used in check_ignition().
 */
uint8_t ignition_enable = 0;

/**
 * @brief Flag to indicate if response from AS is active.
 *
 * Updated in canISR(), used in HandleState() and mission selection logic.
 */
volatile bool res_active = false;

/**
 * @brief Timestamp for entering emergency state.
 *
 * Set in UpdateState() when entering STATE_EMERGENCY, used in HandleState() for timeout.
 */
unsigned long emergency_timestamp = 0;

/**
 * @brief Timestamp for ASSI yellow LED blinking.
 *
 * Used in ASSI() for timing yellow LED blinking in AS_STATE_DRIVING.
 */
unsigned long ASSI_YELLOW_time = 0;

/**
 * @brief Timestamp for ASSI blue LED blinking.
 *
 * Used in ASSI() for timing blue LED blinking in AS_STATE_EMERGENCY.
 */
unsigned long ASSI_BLUE_time = 0;

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

/**
 * @brief Timestamp for initialization delay.
 *
 * Used in HandleState() for implementing 1-second delay before transitioning from STATE_INIT to STATE_MISSION_SELECT.
 */
unsigned long init_delay_time = 0;

/**
 * @brief Last button press time in milliseconds.
 *
 * Used in HandleState() for mission selection button debounce.
 */
unsigned long last_button_time_ms = 0;

/**
 * @brief Last raw state read from the ignition pin.
 *
 * Used in check_ignition() for debounce logic.
 */
static uint8_t last_ign_state = LOW;

/**
 * @brief Last debounced state of the ignition pin.
 *
 * Used in check_ignition() for debounce logic.
 */
static uint8_t debounced_ign_state = LOW;

int speed = 0;

/**
 * @brief Handbook variables that are sent to can bus -> DV driving dynamics 1
 * @showrefs FSG Handbook 2025  page 20
 * @param speed_actual Current speed of the vehicle 0.5 scale
 * @param speed_target Target speed of the vehicle 0.5 scale
 * @param steering_angle_actual Current steering angle of the vehicle 0.5 scale
 * @param steering_angle_target Target steering angle of the vehicle 0.5 scale
 * @param brake_hydr_actual Current hydraulic brake pressure 0.5 scale
 * @param brake_hydr_target Target hydraulic brake pressure 0.5 scale
 * @param motor_moment_actual Current motor moment 0.5 scale
 * @param motor_moment_target Target motor moment 0.5 scale
 * @note 0,5 scale
 * @note ID 0x500
 */
uint8_t Speed_actual = 0;
uint8_t Speed_target = 0;
int8_t Steering_angle_actual = 0;
int8_t Steering_angle_target = 0;
uint8_t Brake_hydr_actual = 0;
uint8_t Brake_hydr_target = 0;
uint8_t Motor_moment_actual = 0;
uint8_t Motor_moment_target = 0;

float wheel_speed_fl = 0;
float wheel_speed_fr = 0;
float wheel_speed_rl = 0;
float wheel_speed_rr = 0;

/**
 * @brief Handbook variables that are sent to can bus -> DV system status
 * @showrefs FSG Handbook 2025  page 20
 * @param as_status Autonomous system status (3 bits) -> @see AS_STATE_t
 * @param EBS_status Emergency Braking System status (2 bits)
 * @param AMI_status Autonomous Mission Indicator status (3 bits) -> @see current_mission
 * @param Steering_state Current steering state (1 bit) -> true if steering is engaged
 * @param ASB_redundancy Autonomous System Backup status (2 bits)
 * @param Lap_counter Current lap counter value (4 bits)
 * @param cones_count_actual Current count of cones detected 8 bits)
 * @param cones_count Total count of cones detected (16 bits)
 *
 * @note ID 0x501
 */
uint8_t as_status = 1;
uint8_t EBS_status = 0;
uint8_t AMI_status = 0;
bool Steering_state = false;
uint8_t ASB_redundancy = 0;
uint8_t Lap_counter = 0;
uint8_t cones_count_actual = 0;
uint16_t cones_count = 0;

uint8_t Brake_pressure_front = 0;
int8_t Brake_pressure_rear = 0;

int16_t dynamics_steering_angle = 0;
int16_t rpm_vcu = 0;

volatile int IGN_manual = 0; // Manual ignition control from Jetson

unsigned long RES_timeout = 0;
unsigned long JETSON_timeout = 0; // Last time a CAN message was received
unsigned long VCU_timeout = 0;    // Last time a CAN message was received
unsigned long MAXON_timeout = 0;  // Last time a CAN message was received

volatile int jetson_ready = 0;
volatile int sdc_signal = 1;