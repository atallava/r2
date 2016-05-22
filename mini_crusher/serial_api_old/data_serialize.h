/**********
 * data_serialize.h - Functions to write data to byte buffers
 * Jeff McMahill (jmcm@ri.cmu.edu)
 * Mike Licitra (mlicitra@cmu.edu)
 * National Robotics Engineering Center, Carnegie Mellon University
 * Copyright (c) 2012 Carnegie Mellon University
 * All rights reserved.
 **********/

#ifndef _DATA_SERIALIZE_H_
#define _DATA_SERIALIZE_H_

#include <stdint.h>

uint8_t read_uint8(uint8_t *buf);
int8_t read_int8(uint8_t *buf);
uint16_t read_uint16(uint8_t *buf);
int16_t read_int16(uint8_t *buf);
uint32_t read_uint32(uint8_t *buf);
uint32_t read_int32(uint8_t *buf);
float read_float(uint8_t *buf);

uint8_t write_uint8(uint8_t *buf, uint8_t d);
uint8_t write_int8(uint8_t *buf, int8_t d);
uint8_t write_uint16(uint8_t *buf, uint16_t d);
uint8_t write_int16(uint8_t *buf, int16_t d);
uint8_t write_uint32(uint8_t *buf, uint32_t d);
uint8_t write_int32(uint8_t *buf, int32_t d);
uint8_t write_float(uint8_t *buf, float d);

union float_t
{
	float f;
	uint32_t i;
};

#endif
