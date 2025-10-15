#pragma once
#include <Arduino.h>
#include "definitions.h"
#include "globals.h"

/* -------------------- STATE MACHINE DEFINITIONS -------------------- */
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
enum ACU_STATE_t {
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
};

// Make state names accessible externally
extern const char *state_names[];

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
enum AS_STATE_t {
  AS_STATE_OFF = 1,       // 0
  AS_STATE_READY = 2,     // 1
  AS_STATE_DRIVING = 3,   // 2
  AS_STATE_EMERGENCY = 4, // 3
  AS_STATE_FINISHED = 5   // 4
};

enum current_mission_t {
  MANUAL,       // 0
  ACCELERATION, // 1
  SKIDPAD,      // 2   // 3
  TRACKDRIVE,   // 4
  EBS_TEST,     // 5
  INSPECTION,   // 6
  AUTOCROSS     // 7
};

enum INITIAL_SEQUENCE_STATE_t {
  WDT_TOOGLE_CHECK,
  WDT_STP_TOOGLE_CHECK,
  PNEUMATIC_CHECK,
  PRESSURE_CHECK1,
  IGNITON,
  PRESSURE_CHECK_FRONT,
  PRESSURE_CHECK_REAR,
  PRESSURE_CHECK2,
  ERROR
};

void UpdateState(void);
void HandleState(void);
void initial_sequence();
void continuous_monitoring();
void print_state_transition(ACU_STATE_t from, ACU_STATE_t to);