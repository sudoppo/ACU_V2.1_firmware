#pragma once
#include <Arduino.h>
#include "definitions.h"
#include "FlexCAN_T4_.h"
#include "IntervalTimer.h"

#define print_state 1

#define PRESSURE_READINGS 8 // Number of pressure readings to average

#define FINISHED_TIMEOUT 5000 // Timeout for finished state in milliseconds
#define FINISHED_TIME_FLAG 0

extern int first_finished_flag; // Flag to indicate if the finished state has been entered
extern unsigned long finished_timestamp; // Timestamp for finished state

extern unsigned long HeartBit;

extern IntervalTimer PRESSURE_TIMER;
extern IntervalTimer CAN_TIMER;
extern IntervalTimer HANDBOOK_MESSAGE_TIMER;

extern FlexCAN_T4<CAN2, RX_SIZE_1024, TX_SIZE_1024> CAN;

/**
 * @enum ACU_STATE_t
 * @brief Represents the various operational states of the ACU (Actuation Control Unit).
 *
 * This enumeration defines the possible states in which the ACU can exist during its lifecycle.
 * Each state corresponds to a specific phase or condition of the system.
 *
 * @var STATE_INIT
 *      The initial state after power-up or reset, where system initialization occurs.
 * @var STATE_MISSION_SELECT
 *      State where the mission or operational mode is selected.
 * @var STATE_INITIAL_SEQUENCE
 *      State for executing the initial sequence before becoming ready.
 * @var STATE_READY
 *      System is ready and awaiting further commands or actions.
 * @var STATE_DRIVING
 *      The ACU is actively controlling the vehicle or system in its driving mode.
 * @var STATE_EBS_ERROR
 *      An error has occurred in the Emergency Braking System (EBS).
 * @var STATE_EMERGENCY
 *      The system has entered an emergency state, requiring immediate attention.
 * @var STATE_FINISHED
 *      The mission or operation has completed, and the system is in a finished state.
 */
typedef enum
{
  STATE_INIT,
  STATE_MISSION_SELECT,
  STATE_JETSONWAITING,
  STATE_INITIAL_SEQUENCE,
  STATE_READY,
  STATE_DRIVING,
  STATE_EBS_ERROR,
  STATE_EMERGENCY,
  STATE_FINISHED,
  STATE_MANUAL,
} ACU_STATE_t;

/**
 * @enum AS_STATE_t
 * @brief Represents the different states of the Autonomous System (AS).
 * This enumeration defines the various operational states of the AS,
 * which can be used to manage the system's behavior during autonomous operations.
 * @var AS_STATE_OFF
 * ASSI OFF
 * @var AS_STATE_READY
 *    ASSI yellow
 *  @var AS_STATE_DRIVING
 *   ASSI Blinking yellow
 * @var AS_STATE_EMERGENCY
 *  ASSI blinking Blue
 * @var AS_STATE_FINISHED
 * ASSI Blue
 */
typedef enum
{
  AS_STATE_OFF = 1,       // 0
  AS_STATE_READY = 2,     // 1
  AS_STATE_DRIVING = 3,   // 2
  AS_STATE_EMERGENCY = 4, // 3
  AS_STATE_FINISHED = 5   // 4
} AS_STATE_t;

typedef enum
{
  MANUAL,       // 0
  ACCELERATION, // 1
  SKIDPAD,      // 2   // 3
  TRACKDRIVE,   // 4
  EBS_TEST,     // 5
  INSPECTION,   // 6
  AUTOCROSS     // 7
} current_mission_t;

typedef enum
{
  WDT_TOOGLE_CHECK,
  WDT_STP_TOOGLE_CHECK,
  PNEUMATIC_CHECK,
  PRESSURE_CHECK1,
  IGNITON,
  PRESSURE_CHECK_FRONT,
  PRESSURE_CHECK_REAR,
  PRESSURE_CHECK2,
  ERROR
} INITIAL_SEQUENCE_STATE_t;

extern unsigned long mission_LED_time;
extern bool mission_LED_state;
extern unsigned long init_delay_time;

// shared state variables (extern)
extern volatile ACU_STATE_t current_state;
extern volatile ACU_STATE_t previous_state;
extern volatile AS_STATE_t as_state;

extern INITIAL_SEQUENCE_STATE_t initial_sequence_state;
extern current_mission_t current_mission;
extern current_mission_t jetson_mission;

// pressure arrays
extern float EBS_TANK_PRESSURE_A_values[PRESSURE_READINGS];
extern float EBS_TANK_PRESSURE_B_values[PRESSURE_READINGS];

extern float TANK_PRESSURE_FRONT;
extern float TANK_PRESSURE_REAR;
extern float HYDRAULIC_PRESSURE_FRONT;
extern float HYDRAULIC_PRESSURE_REAR;

extern uint8_t adc_pointer;
extern volatile bool update_median_flag;

extern uint8_t ignition_flag;
extern uint8_t ignition_vcu;
extern uint8_t asms_flag;
extern uint8_t emergency_flag;
extern volatile uint8_t res_emergency;
extern volatile bool wdt_togle_enable;
extern unsigned long wdt_togle_counter;
extern unsigned long wdt_relay_timout;
extern unsigned long pressure_check_delay;
extern uint8_t ignition_enable;
extern volatile bool res_active;
extern unsigned long emergency_timestamp;
extern unsigned long ASSI_YELLOW_time;
extern unsigned long ASSI_BLUE_time;
extern unsigned long last_button_time_ms;
extern uint8_t last_ign_state;
extern uint8_t debounced_ign_state;

extern int speed;

// handbook / DV variables
extern uint8_t Speed_actual;
extern uint8_t Speed_target;
extern int8_t Steering_angle_actual;
extern int8_t Steering_angle_target;
extern uint8_t Brake_hydr_actual;
extern uint8_t Brake_hydr_target;
extern uint8_t Motor_moment_actual;
extern uint8_t Motor_moment_target;

extern float wheel_speed_fl;
extern float wheel_speed_fr;
extern float wheel_speed_rl;
extern float wheel_speed_rr;

extern uint8_t as_status;
extern uint8_t EBS_status;
extern uint8_t AMI_status;
extern bool Steering_state;
extern uint8_t ASB_redundancy;
extern uint8_t Lap_counter;
extern uint8_t cones_count_actual;
extern uint16_t cones_count;

extern uint8_t Brake_pressure_front;
extern int8_t Brake_pressure_rear;

extern int16_t dynamics_steering_angle;
extern int16_t rpm_vcu;

extern unsigned long RES_timeout;
extern unsigned long JETSON_timeout;
extern unsigned long VCU_timeout;
extern unsigned long MAXON_timeout;

extern volatile int jetson_ready;
extern volatile int IGN_manual;
extern volatile int sdc_signal;

extern volatile int IGN_manual;
