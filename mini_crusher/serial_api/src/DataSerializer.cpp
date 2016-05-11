/**********
 * data_serialize.c - Functions to write data to byte buffers
 * Jeff McMahill (jmcm@ri.cmu.edu)
 * Mike Licitra (mlicitra@cmu.edu)
 * National Robotics Engineering Center, Carnegie Mellon University
 * Copyright (c) 2012 Carnegie Mellon University
 * All rights reserved.
 **********/

#include <serial_api/DataSerializer.h>

using namespace serial_api;

uint8_t DataSerializer::read_uint8(const uint8_t *buf)
{
	return buf[0];
}

int8_t DataSerializer::read_int8(const uint8_t *buf)
{
	return buf[0];
}

uint16_t DataSerializer::read_uint16(const uint8_t *buf)
{
	uint16_t x = 0;
	x = ((uint16_t)buf[0] << 8 ) | buf[1];
	return x;
}

int16_t DataSerializer::read_int16(const uint8_t *buf)
{
	int16_t x = 0;
	x = ((int16_t)buf[0] << 8 ) | buf[1];
	return x;
}

uint32_t DataSerializer::read_uint32(const uint8_t *buf)
{
	uint32_t x = 0;
	x = ((uint32_t)buf[0] << 24 ) | ((uint32_t)buf[1] << 16 ) | ((uint32_t)buf[2] << 8 ) | (buf[3] << 0 );
	return x;
}

uint32_t DataSerializer::read_int32(const uint8_t *buf)
{
	int32_t x = 0;
	x = ((int32_t)buf[0] << 24 ) | ((int32_t)buf[1] << 16 ) | ((int32_t)buf[2] << 8 ) | (buf[3] << 0 );
	return x;
}

float DataSerializer::read_float(const uint8_t *buf)
{
	union float_t f;
	f.i = read_uint32(buf);
	return f.f;
}

uint8_t DataSerializer::write_uint8(uint8_t *buf, const uint8_t data)
{
	buf[0] = data;
	return 1;
}

uint8_t DataSerializer::write_int8(uint8_t *buf, const int8_t data)
{
	buf[0] = data;
	return 1;
}

uint8_t DataSerializer::write_uint16(uint8_t *buf, const uint16_t data)
{
	buf[0] = data >> 8;
	buf[1] = data;
	return 2;
}

uint8_t DataSerializer::write_int16(uint8_t *buf, const int16_t data)
{
	buf[0] = data >> 8;
	buf[1] = data;
	return 2;
}

uint8_t DataSerializer::write_uint32(uint8_t *buf, const uint32_t data)
{
	buf[0] = data >> 24;
	buf[1] = data >> 16;
	buf[2] = data >> 8;
	buf[3] = data;
	return 4;
}

uint8_t DataSerializer::write_int32(uint8_t *buf, const int32_t data)
{
	buf[0] = data >> 24;
	buf[1] = data >> 16;
	buf[2] = data >> 8;
	buf[3] = data;
	return 4;
}


uint8_t DataSerializer::write_float(uint8_t *buf, const float data)
{
	union float_t f;
	f.f = data;
	return write_uint32(buf, f.i);
}

