#pragma once
#include "globals.h"

void canISR(const CAN_message_t &msg);
void send_can_msg();
void send_handbook_variables();