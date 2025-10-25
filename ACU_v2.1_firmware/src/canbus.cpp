#include <Arduino.h>
#include "canbus.h"
#include "globals.h"
#include "autonomous_temporary.h"

/**
 * @brief CAN receive interrupt callback.
 *
 * When a can message is received, this function is called to process the message.
 * It decodes the message based on its ID and updates the system state accordingly.
 * @note This function is called from the FlexCAN_T4 library's interrupt handler.
 * @param msg The received CAN message
 */
void canISR(const CAN_message_t &msg)
{
  uint16_t aux_brake_p = 0;
  digitalWrite(Debug_LED3, !digitalRead(Debug_LED3)); // Indicate reception of RES message
  switch (msg.id)
  {
  case 0x446:
    dynamics_steering_angle = msg.buf[1] << 8 | msg.buf[0]; // Update steering angle actual
    Steering_angle_actual = (dynamics_steering_angle / 10);
    Serial2.println("Steering angle actual: " + String(Steering_angle_actual) + " degrees in isr");
    Serial2.println("Dynamics steering angle: " + String(dynamics_steering_angle / 10) + " degrees in isr");
    break;

  case 0x546:
    aux_brake_p = (msg.buf[1] << 8 | msg.buf[0]);
    HYDRAULIC_PRESSURE_REAR = aux_brake_p / 10; // Update rear brake pressure
    Brake_pressure_rear = (u_int8_t)HYDRAULIC_PRESSURE_REAR;
    Serial2.println("Rear brake pressure: " + String(Brake_pressure_rear) + " bar in isr");
    break;
    // TODO: Read id 0x556
    //  divide by 10 and store in float

  case AUTONOMOUS_TEMPORARY_JETSON_MS_FRAME_ID:
    jetson_mission = (current_mission_t)msg.buf[0]; // Update mission response from Jetson
    break;

  case AUTONOMOUS_TEMPORARY_RES_FRAME_ID:
    RES_timeout = millis();
    if (msg.buf[0] == 0)
    {
      res_emergency = 1; // Set emergency response flag
    }
    else
    {
      res_emergency = 0; // Reset emergency response flag
    }

    if (!res_active)
    {
      res_active = true; // Set response active flag
    }
    break;

  case AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID:
    VCU_timeout = millis(); // Update VCU timeout
    if (ignition_flag == 1)
    {
      ignition_vcu = (msg.buf[0] == 9) ? 1 : 0; // Update ignition signal from VCU
    }
    else
    {
      ignition_vcu = 0; // Reset ignition flag if ignition enable is not set
    }

    HYDRAULIC_PRESSURE_FRONT = msg.buf[1]; // Convert to bar
    // Serial2.println("Hydraulic pressure front: " + String(HYDRAULIC_PRESSURE_FRONT) + " bar in isr");
    break;

  case 0x503:
    JETSON_timeout = millis(); // Update Jetson timeout
                               /* if (current_state != STATE_EMERGENCY || current_state != STATE_FINISHED)
                                {
                                  // Update autonomous system state
                                }*/

    if (msg.buf[0] == 5 || msg.buf[0] == 4)
    {
      if (msg.buf[0] == 5)
      {
        current_state = STATE_FINISHED; // Transition to FINISHED state
        Serial.println("Jetson responded with AS_STATE_FINISHED, transitioning to FINISHED state");
      }
      if (msg.buf[0] == AS_STATE_EMERGENCY)
      {
        current_state = STATE_EMERGENCY; // Transition to EMERGENCY state
        Serial.println("Jetson responded with AS_STATE_EMERGENCY, transitioning to EMERGENCY state");
      }
    }
    else
    {
      as_state = (AS_STATE_t)msg.buf[0];

      switch (as_state)
      {
      case AS_STATE_OFF:
        // If in JETSONWAITING state, transition to INITIAL_SEQUENCE when receiving AS_STATE_OFF
        if (current_state == STATE_JETSONWAITING)
        {

          current_state = STATE_INITIAL_SEQUENCE;
          initial_sequence_state = WDT_TOOGLE_CHECK; // Reset initial sequence state
          Serial.println("Jetson responded with AS_STATE_OFF, starting initial sequence");
        }
        jetson_ready = 0;
        // Don't transition to READY from other states - only after initial sequence completes
        break;

      case AS_STATE_READY:
        current_state = STATE_READY; // Transition to READY state
        jetson_ready = 1;
        break;

      case AS_STATE_DRIVING:
        current_state = STATE_DRIVING; // Transition to DRIVING state
        break;

      case AS_STATE_EMERGENCY:
        current_state = STATE_EMERGENCY; // Transition to EMERGENCY state
        break;

      case AS_STATE_FINISHED:
        current_state = STATE_FINISHED; // Transition to FINISHED state
        break;

      default:
        break;
      }
    }

    break;

  case 0x456: // Front wheels
    wheel_speed_fl = 0;
    wheel_speed_fr = 0;
    wheel_speed_fl = (uint16_t)msg.buf[0] | ((uint16_t)msg.buf[1] << 8);
    wheel_speed_fl = wheel_speed_fl * 0.1;
    wheel_speed_fr = (uint16_t)msg.buf[2] | ((uint16_t)msg.buf[3] << 8);
    wheel_speed_fr = wheel_speed_fr * 0.1;
    Serial2.println("Wheel speed front: FL = " + String(wheel_speed_fl) + " | FR = " + String(wheel_speed_fr));
    break;

  case 0x556: // Rear wheels
    wheel_speed_rl = 0;
    wheel_speed_rr = 0;
    wheel_speed_rl = (uint16_t)msg.buf[0] | ((uint16_t)msg.buf[1] << 8);
    wheel_speed_rl = wheel_speed_rl * 0.1;
    wheel_speed_rr = (uint16_t)msg.buf[2] | ((uint16_t)msg.buf[3] << 8);
    wheel_speed_rr = wheel_speed_rr * 0.1;
    // Serial.println("Wheel speed rear: RL = " + String(wheel_speed_rl) + " | RR = " + String(wheel_speed_rr));
    break;
  case 0x509:
    rpm_vcu = ((msg.buf[1] << 8) | msg.buf[0]);
    break;
  case 0x600:
    IGN_manual = msg.buf[0]; // Read manual ignition state
    sdc_signal = msg.buf[4];
  default:
    // Unknown message ID, ignore
    break;
  }
}

/**
 * @brief Send CAN messages based on system state
 */
void send_can_msg()
{

  uint8_t tx_buffer[8]; // Buffer for CAN message

  CAN_message_t tx_message;

  struct autonomous_temporary_acu_ign_t encoded_ign;
  encoded_ign.ebs_pressure_rear = (uint8_t)(TANK_PRESSURE_FRONT * 10); // Convert to 0.1 bar scale
  encoded_ign.ebs_pressure_front = (uint8_t)(TANK_PRESSURE_REAR * 10); // Convert to 0.1 bar scale
  encoded_ign.ign = ignition_flag;                                     // Set ignition flag
  encoded_ign.asms = asms_flag;                                        // Set ASMS flag
  encoded_ign.emergency = emergency_flag;                              // Set emergency flag

  autonomous_temporary_acu_ign_pack(tx_buffer, &encoded_ign, AUTONOMOUS_TEMPORARY_ACU_IGN_LENGTH);
  tx_message.id = AUTONOMOUS_TEMPORARY_ACU_IGN_FRAME_ID;                  // Set CAN ID
  tx_message.len = AUTONOMOUS_TEMPORARY_ACU_IGN_LENGTH;                   // Set message length
  memcpy(tx_message.buf, tx_buffer, AUTONOMOUS_TEMPORARY_ACU_IGN_LENGTH); // Copy data to CAN message buffer

  CAN.write(tx_message); // Send CAN message

  struct autonomous_temporary_acu_ms_t encoded_mission;
  encoded_mission.mission_select = (uint8_t)current_mission;
  autonomous_temporary_acu_ms_pack(tx_buffer, &encoded_mission, AUTONOMOUS_TEMPORARY_ACU_MS_LENGTH);
  tx_message.id = AUTONOMOUS_TEMPORARY_ACU_MS_FRAME_ID;                  // Set CAN ID
  tx_message.len = AUTONOMOUS_TEMPORARY_ACU_MS_LENGTH;                   // Set message length
  memcpy(tx_message.buf, tx_buffer, AUTONOMOUS_TEMPORARY_ACU_MS_LENGTH); // Copy data to CAN message buffer

  CAN.write(tx_message); // Send CAN message

  struct autonomous_temporary_rd_jetson_t RD_jetson_encode;
  if (current_state == STATE_JETSONWAITING)
  {
    RD_jetson_encode.rd = 1; // Set RD value for Jetson waiting state
  }
  if (current_state == STATE_READY)
  {
    RD_jetson_encode.rd = 2; // Set RD value based on current mission
  }
  else if (current_state == STATE_DRIVING)
  {
    RD_jetson_encode.rd = 3; // Set RD value based on current mission
  }
  else if (current_state == STATE_EMERGENCY)
  {
    RD_jetson_encode.rd = 4; // Set RD value based on current mission
  }
  else
  {
    RD_jetson_encode.rd = 1; // Set RD value based on current mission
  }
  autonomous_temporary_rd_jetson_pack(tx_buffer, &RD_jetson_encode, AUTONOMOUS_TEMPORARY_RD_JETSON_LENGTH);
  tx_message.id = 0x513;
  tx_message.len = AUTONOMOUS_TEMPORARY_RD_JETSON_LENGTH;                   //
  memcpy(tx_message.buf, tx_buffer, AUTONOMOUS_TEMPORARY_RD_JETSON_LENGTH); // Copy data to CAN message buffer

  CAN.write(tx_message); // Send CAN message

  // Send ACU state on ID 0x700
  CAN_message_t acu_state_msg;
  acu_state_msg.id = 0x700;
  acu_state_msg.len = 8;
  acu_state_msg.buf[0] = (uint8_t)current_state; // Send current ACU state in first byte
  acu_state_msg.buf[1] = (uint8_t)as_state;      // Send current autonomous system state in second byte
  CAN.write(acu_state_msg);                      // Send ACU state message
}

/**
 * @brief Encodes handbook variables for CAN bus transmission.
 * This function prepares the handbook variables related to driving dynamics
 */
void send_handbook_variables()
{

  uint8_t aux_buf = 0;

  // DV driving dynamics 1
  CAN_message_t msg;
  msg.buf[0] = Speed_actual;              // Scale speed actual to 0.5
  msg.buf[1] = Speed_target;              // Scale speed target to 0.5
  msg.buf[2] = Steering_angle_actual * 2; // Scale steering angle actual to 0.5
  msg.buf[3] = Steering_angle_target * 2; // Scale steering angle target to 0.5
  msg.buf[4] = Brake_hydr_actual;         // Scale brake hydraulic actual to 0.5
  msg.buf[5] = Brake_hydr_target;         // Scale brake hydraulic target to 0.5
  msg.buf[6] = Motor_moment_actual;       // Scale motor moment actual to 0.5
  msg.buf[7] = Motor_moment_target;       // Scale motor moment target to 0.5

  msg.id = 0x500; // Set CAN ID for driving dynamics 1
  msg.len = 8;    // Set message length to 8 bytes
  CAN.write(msg); // Send the CAN message

  // DV system status

  // byte 0
  CAN_message_t msg_dv;
  msg_dv.buf[0] = as_status && 0b00000111; // Autonomous system status
  aux_buf = EBS_status << 3;
  msg_dv.buf[0] = msg_dv.buf[0] || aux_buf; // Set EBS status in bits 3-4
  aux_buf = AMI_status << 5;                // set AMI status in bits 5-7
  msg_dv.buf[0] = msg_dv.buf[0] || aux_buf; // Set AMI status in bits 5-7

  // byte 1
  msg_dv.buf[1] = (Steering_state ? 1 : 0);
  aux_buf = ASB_redundancy << 1;            // Set ASB redundancy in bits 1-2
  msg_dv.buf[1] = msg_dv.buf[1] || aux_buf; // Set ASB redundancy in bits 1-2
  aux_buf = Lap_counter << 3;               // Set lap counter in bits 3-6
  msg_dv.buf[1] = msg_dv.buf[1] || aux_buf; // Set lap

  // byte 2
  msg_dv.buf[2] = cones_count_actual; // Set current cones count in byte 2
  // byte 3-4
  msg_dv.buf[3] = cones_count >> 8;
  msg_dv.buf[4] = (cones_count << 8) & 0xFF; // Set total cones count in bytes 3-4

  msg_dv.id = 0x502;
  msg_dv.len = 5;    // Set message length to 5 bytes
  CAN.write(msg_dv); // Send the CAN message

  // ASF signals

  CAN_message_t msg_asf;
  msg_asf.buf[0] = (uint8_t)(TANK_PRESSURE_FRONT * 10);
  msg_asf.buf[1] = (uint8_t)(TANK_PRESSURE_REAR * 10);
  msg_asf.buf[2] = Brake_pressure_front; // Convert to 0.1 bar scale
  msg_asf.buf[3] = Brake_pressure_rear;  // Convert to 0.1 bar scale

  msg_asf.len = 4;    // Set message length to 4 bytes
  msg_asf.id = 0x511; // Set CAN ID for ASF signals
  CAN.write(msg_asf); // Send the CAN message
}