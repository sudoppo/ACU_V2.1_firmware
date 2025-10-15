#pragma once
#include <Arduino.h>
#include "FlexCAN_T4.h"
#include "IntervalTimer.h"
#include "definitions.h"
#include "states.h"

// === State Machine Variables ===
extern volatile ACU_STATE_t current_state;
extern volatile ACU_STATE_t previous_state;
extern volatile AS_STATE_t as_state;
extern INITIAL_SEQUENCE_STATE_t initial_sequence_state;
extern current_mission_t current_mission;
extern current_mission_t jetson_mission;

// === Pressure Variables ===
extern float EBS_TANK_PRESSURE_A_values[];
extern float EBS_TANK_PRESSURE_B_values[];
extern float TANK_PRESSURE_FRONT;
extern float TANK_PRESSURE_REAR;
extern float HYDRAULIC_PRESSURE_FRONT;
extern float HYDRAULIC_PRESSURE_REAR;
extern uint8_t adc_pointer;
extern volatile bool update_median_flag;

// === Flags and Timing ===
extern uint8_t ignition_flag, ignition_vcu, ignition_enable, asms_flag, emergency_flag;
extern volatile uint8_t res_emergency;
extern volatile bool res_active;
extern unsigned long pressure_check_delay;
extern unsigned long emergency_timestamp;

// === CAN and Timers ===
extern FlexCAN_T4<CAN2, RX_SIZE_1024, TX_SIZE_1024> CAN;
extern IntervalTimer PRESSURE_TIMER, CAN_TIMER, HANDBOOK_MESSAGE_TIMER;

// === Handbook Variables === //



// === Other === //
extern bool mission_LED_state;
extern unsigned long mission_LED_time;