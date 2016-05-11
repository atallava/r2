#pragma once
#include <inttypes.h>

namespace serial_api {
#define STATUS_PACKET_LEN 42
#define ENCODER_PACKET_LEN 30
#define CMD_PACKET_LEN 19

    class McPayloadCommunicator {
    public:
	enum payload_packets {PAYLOAD_CMD = 0, PAYLOAD_STATUS, PAYLOAD_ENCODER};

	typedef struct
	{
	    int16_t wheel_vel[6]; // velocity command in mm/s
	    uint8_t enable;
	    uint8_t timeout;
	    uint8_t new_packet;
	} cmd_t;

	typedef struct
	{
	    int32_t position[6]; // encoder positions for each wheel
	    uint8_t fresh;

	    uint8_t new_packet;

	} encoder_t;

	typedef struct
	{
	    int16_t sticks[6]; // encoder positions for each wheel
	    uint8_t fresh;

	    uint8_t new_packet;

	} radiodata_t;

	typedef struct
	{

	    uint8_t motor_temp[6]; // motor temperatures deg f
	    uint8_t drive_temp[6]; // motor temperatures def f
	    uint8_t angle_sensor[6];
	    int16_t current[6]; // in mA
	    int16_t battery_current;
	    int16_t battery_voltage; // battery voltage x100
	    int16_t time;

	    uint8_t fresh;
	    uint8_t new_packet;


	} status_t;

	uint8_t cmdPack(cmd_t *cmd, uint8_t *buf);
	uint8_t statusPack(status_t *cmd, uint8_t *buf);
	uint8_t encoderPack(encoder_t *cmd, uint8_t *buf);

	uint8_t cmdUnpack(cmd_t *cmd, uint8_t *buf);
	uint8_t statusUnpack(status_t *cmd, uint8_t *buf);
	uint8_t encoderUnpack(encoder_t *cmd, uint8_t *buf);

	void recvCmd(uint8_t c);
	void recvStatus(uint8_t c);
	void recvEncoder(uint8_t c);

	void printCmd(cmd_t *cmd);
	void printStatus(status_t *stat);
	void printEncoder(encoder_t *enc);

    private:
	uint8_t read_buffer_cmd[CMD_PACKET_LEN];
	uint8_t read_buffer_stat[CMD_PACKET_LEN];
	uint8_t read_buffer_enc[CMD_PACKET_LEN];
	cmd_t cmd;
	status_t stat;
	encoder_t enc;
    };
} // namespace serial_api
