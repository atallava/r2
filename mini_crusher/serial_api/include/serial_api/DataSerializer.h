/**********
 * data_serialize.h - Functions to write data to byte buffers
 * Jeff McMahill (jmcm@ri.cmu.edu)
 * Mike Licitra (mlicitra@cmu.edu)
 * National Robotics Engineering Center, Carnegie Mellon University
 * Copyright (c) 2012 Carnegie Mellon University
 * All rights reserved.
 **********/

#pragma once
#include <stdint.h>

namespace serial_api {
    class DataSerializer {
    public: 
	static uint8_t read_uint8(const uint8_t *buf);
	static int8_t read_int8(const uint8_t *buf);
	static uint16_t read_uint16(const uint8_t *buf);
	static int16_t read_int16(const uint8_t *buf);
	static uint32_t read_uint32(const uint8_t *buf);
	static uint32_t read_int32(const uint8_t *buf);
	static float read_float(const uint8_t *buf);

	static uint8_t write_uint8(uint8_t *buf, const uint8_t data);
	static uint8_t write_int8(uint8_t *buf, const int8_t data);
	static uint8_t write_uint16(uint8_t *buf, const uint16_t data);
	static uint8_t write_int16(uint8_t *buf, const int16_t data);
	static uint8_t write_uint32(uint8_t *buf, const uint32_t data);
	static uint8_t write_int32(uint8_t *buf, const int32_t data);
	static uint8_t write_float(uint8_t *buf, const float data);

	union float_t
	{
	    float f;
	    uint32_t i;
	};
    };
} // namespace serial_api

