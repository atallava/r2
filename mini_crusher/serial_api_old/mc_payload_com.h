#ifndef _MC_PAYLOAD_COM_H_
#define _MC_PAYLOAD_COM_H_

enum payload_packets {PAYLOAD_CMD = 0, PAYLOAD_STATUS, PAYLOAD_ENCODER};

typedef struct
{
	int16_t wheel_vel[6]; // velocity command in mm/s
	uint8_t enable;
	uint8_t timeout;
	uint8_t new_packet;
} mc_payload_cmd_t;



typedef struct
{
	int32_t position[6]; // encoder positions for each wheel
	uint8_t fresh;

	uint8_t new_packet;

} mc_payload_encoder_t;

typedef struct
{
	int16_t sticks[6]; // encoder positions for each wheel
	uint8_t fresh;

	uint8_t new_packet;

} mc_payload_radiodata_t;

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


} mc_payload_status_t;


uint8_t mc_payload_cmd_pack(mc_payload_cmd_t *cmd, uint8_t *buf);
uint8_t mc_payload_status_pack(mc_payload_status_t *cmd, uint8_t *buf);
uint8_t mc_payload_encoder_pack(mc_payload_encoder_t *cmd, uint8_t *buf);

uint8_t mc_payload_cmd_unpack(mc_payload_cmd_t *cmd, uint8_t *buf);
uint8_t mc_payload_status_unpack(mc_payload_status_t *cmd, uint8_t *buf);
uint8_t mc_payload_encoder_unpack(mc_payload_encoder_t *cmd, uint8_t *buf);

void mc_payload_print_status(mc_payload_status_t *stat);
void mc_payload_print_cmd(mc_payload_cmd_t *cmd);
void mc_payload_print_encoder(mc_payload_encoder_t *enc);


void mc_payload_recv_encoder(uint8_t c);
void mc_payload_recv_status(uint8_t c);
void mc_payload_recv_cmd(uint8_t c);

#endif

