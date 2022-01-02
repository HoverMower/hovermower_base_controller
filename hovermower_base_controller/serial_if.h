// definition of serial interface


#ifndef SERIAL_IF_H
#define SERIAL_IF_H

#include "protocol.h"
#include "config.h"

SerialCommand  cmd_msg;
int msg_len = 0;
unsigned char prev_byte = 0;
uint16_t cmd_frame = 0;
unsigned char *p;
unsigned long last_time_command = 0;
#endif
