/**********
 * data_serialize.c - Functions to write data to byte buffers
 * Jeff McMahill (jmcm@ri.cmu.edu)
 * Mike Licitra (mlicitra@cmu.edu)
 * National Robotics Engineering Center, Carnegie Mellon University
 * Copyright (c) 2012 Carnegie Mellon University
 * All rights reserved.
 **********/

#include "data_serialize.h"


uint8_t read_uint8(uint8_t *buf)
{
	return buf[0];
}

int8_t read_int8(uint8_t *buf)
{
	return buf[0];
}

uint16_t read_uint16(uint8_t *buf)
{
	uint16_t x = 0;
	x = ((uint16_t)buf[0] << 8 ) | buf[1];
	return x;
}

int16_t read_int16(uint8_t *buf)
{
	int16_t x = 0;
	x = ((int16_t)buf[0] << 8 ) | buf[1];
	return x;
}

uint32_t read_uint32(uint8_t *buf)
{
	uint32_t x = 0;
	x = ((uint32_t)buf[0] << 24 ) | ((uint32_t)buf[1] << 16 ) | ((uint32_t)buf[2] << 8 ) | (buf[3] << 0 );
	return x;
}

uint32_t read_int32(uint8_t *buf)
{
	int32_t x = 0;
	x = ((int32_t)buf[0] << 24 ) | ((int32_t)buf[1] << 16 ) | ((int32_t)buf[2] << 8 ) | (buf[3] << 0 );
	return x;
}

float read_float(uint8_t *buf)
{
	union float_t f;
	f.i = read_uint32(buf);
	return f.f;
}

uint8_t write_uint8(uint8_t *buf, uint8_t d)
{
	buf[0] = d;
	return 1;
}

uint8_t write_int8(uint8_t *buf, int8_t d)
{
	buf[0] = d;
	return 1;
}

uint8_t write_uint16(uint8_t *buf, uint16_t d)
{
	buf[0] = d >> 8;
	buf[1] = d;
	return 2;
}

uint8_t write_int16(uint8_t *buf, int16_t d)
{
	buf[0] = d >> 8;
	buf[1] = d;
	return 2;
}

uint8_t write_uint32(uint8_t *buf, uint32_t d)
{
	buf[0] = d >> 24;
	buf[1] = d >> 16;
	buf[2] = d >> 8;
	buf[3] = d;
	return 4;
}

uint8_t write_int32(uint8_t *buf, int32_t d)
{
	buf[0] = d >> 24;
	buf[1] = d >> 16;
	buf[2] = d >> 8;
	buf[3] = d;
	return 4;
}


uint8_t write_float(uint8_t *buf, float d)
{
	union float_t f;
	f.f = d;
	return write_uint32(buf, f.i);
}

