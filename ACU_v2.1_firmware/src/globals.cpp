#include <Arduino.h>

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