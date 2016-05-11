#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "mc_payload_com.h"
#include "fcs.h"
#include "data_serialize.h"


#define STATUS_PACKET_LEN 42
#define ENCODER_PACKET_LEN 30
#define CMD_PACKET_LEN 19

extern mc_payload_cmd_t mc_cmd;
extern mc_payload_status_t mc_stat;
extern mc_payload_encoder_t mc_enc;



uint8_t mc_payload_cmd_pack(mc_payload_cmd_t *cmd, uint8_t *buf)
{
	uint8_t pos = 0;
	uint8_t x;

	buf[pos++] = 'M';
	buf[pos++] = 'C';
	buf[pos++] = PAYLOAD_CMD;
	buf[pos++] = CMD_PACKET_LEN;

	for(x = 0; x < 6; x++)
	{
		pos += write_int16(&buf[pos], cmd->wheel_vel[x]);
	}
	buf[pos++] = cmd->enable ? 1 : 0;
	return addfcs(buf, pos);
	// 19
}


uint8_t mc_payload_encoder_pack(mc_payload_encoder_t *enc, uint8_t *buf)
{
	uint8_t pos = 0;
	uint8_t x;

	buf[pos++] = 'M';
	buf[pos++] = 'C';
	buf[pos++] = PAYLOAD_ENCODER;
	buf[pos++] = ENCODER_PACKET_LEN;

	for(x = 0; x < 6; x++)
	{
		pos += write_int32(&buf[pos], enc->position[x]);
	}
	return addfcs(buf, pos);
	// 30
}

uint8_t mc_payload_status_pack(mc_payload_status_t *stat, uint8_t *buf)
{
	uint8_t pos = 0;
	uint8_t x;

	buf[pos++] = 'M';
	buf[pos++] = 'C';
	buf[pos++] = PAYLOAD_STATUS;
	buf[pos++] = STATUS_PACKET_LEN;


	for(x = 0; x < 6; x++)
	{
		pos += write_uint8(&buf[pos], stat->motor_temp[x]);
	}

	for(x = 0; x < 6; x++)
	{
		pos += write_uint8(&buf[pos], stat->drive_temp[x]);
	}
	for(x = 0; x < 6; x++)
	{
		pos += write_uint8(&buf[pos], stat->angle_sensor[x]);
	}

	for(x = 0; x < 6; x++)
	{
		pos += write_int16(&buf[pos], stat->current[x]);
	}

	pos += write_int16(&buf[pos], stat->battery_current);
	pos += write_int16(&buf[pos], stat->battery_voltage);

	pos += write_uint16(&buf[pos], stat->time);

	return addfcs(buf, pos);
	// 42
}

uint8_t mc_payload_cmd_unpack(mc_payload_cmd_t *cmd, uint8_t *buf)
{
	if(buf[0] != 'M' || buf[1] != 'C' || buf[2] != PAYLOAD_CMD)
	{
		printf("Wrong Packet\n\r");
		return 0;
	}
	if(!checkfcs(buf, CMD_PACKET_LEN))
	{
		printf("FCS Failed\n\r");
		return 0;
	}
	uint8_t pos = 0;
	uint8_t x;
	pos = 4;
	for(x = 0; x < 6; x++)
	{

		cmd->wheel_vel[x] = read_int16(&buf[pos]);
		pos += 2;
	}
	cmd->enable = buf[pos++] & 0x01;

	return 1;

}

uint8_t mc_payload_encoder_unpack(mc_payload_encoder_t *enc, uint8_t *buf)
{
	if(buf[0] != 'M' || buf[1] != 'C' || buf[2] != PAYLOAD_ENCODER)
	{
		printf("Wrong Packet\n\r");
		return 0;
	}
	if(!checkfcs(buf, ENCODER_PACKET_LEN))
	{
		printf("FCS Failed\n\r");
		return 0;
	}
	uint8_t pos = 0;
	uint8_t x;
	pos = 4;
	for(x = 0; x < 6; x++)
	{
		enc->position[x] = read_int32(&buf[pos]);
		pos += 4;
	}
	return 1;
}

uint8_t mc_payload_status_unpack(mc_payload_status_t *stat, uint8_t *buf)
{

	if(buf[0] != 'M' || buf[1] != 'C' || buf[2] != PAYLOAD_STATUS)
	{
		printf("Wrong Packet\n\r");
		return 0;
	}
	if(!checkfcs(buf, STATUS_PACKET_LEN))
	{
		printf("FCS Failed\n\r");
		return 0;
	}
	uint8_t pos = 0;
	uint8_t x;
	pos = 4;


	for(x = 0; x < 6; x++)
	{
		stat->motor_temp[x] = read_uint8(&buf[pos]);
		pos += 1;
	}
	for(x = 0; x < 6; x++)
	{
		stat->drive_temp[x] = read_uint8(&buf[pos]);
		pos += 1;
	}
	for(x = 0; x < 6; x++)
	{
		stat->angle_sensor[x] = read_uint8(&buf[pos]);
		pos += 1;
	}
	for(x = 0; x < 6; x++)
	{
		stat->current[x] = read_int16(&buf[pos]);
		pos += 2;
	}
	stat->battery_current = read_int16(&buf[pos]);
	pos += 2;
	stat->battery_voltage = read_int16(&buf[pos]);
	pos += 2;

	stat->time = read_int16(&buf[pos]);
	pos += 2;

	return 1;
}

void mc_payload_recv_encoder(uint8_t c)
{
	uint8_t x;
	static uint8_t mc_rx_buf1[ENCODER_PACKET_LEN];
	for(x = 0; x < ENCODER_PACKET_LEN-1; x++)
		mc_rx_buf1[x] = mc_rx_buf1[x + 1];
	mc_rx_buf1[ENCODER_PACKET_LEN-1] = c;
	if(mc_rx_buf1[0] == 'M' && mc_rx_buf1[1] == 'C' && mc_rx_buf1[2] == PAYLOAD_ENCODER)
	{
		if(checkfcs(&mc_rx_buf1[0], ENCODER_PACKET_LEN))
		{
			mc_payload_encoder_unpack(&mc_enc, &mc_rx_buf1[0]);
			mc_enc.new_packet = 1;
		}
	}
}

void mc_payload_recv_status(uint8_t c)
{
	uint8_t x;
	static uint8_t mc_rx_buf2[STATUS_PACKET_LEN];
	for(x = 0; x < STATUS_PACKET_LEN-1; x++)
		mc_rx_buf2[x] = mc_rx_buf2[x + 1];
	mc_rx_buf2[STATUS_PACKET_LEN-1] = c;
	if(mc_rx_buf2[0] == 'M' && mc_rx_buf2[1] == 'C' && mc_rx_buf2[2] == PAYLOAD_STATUS)
	{
		if(checkfcs(&mc_rx_buf2[0], STATUS_PACKET_LEN))
		{
			mc_payload_status_unpack(&mc_stat, &mc_rx_buf2[0]);
			mc_stat.new_packet = 1;
		}
	}
}

void mc_payload_recv_cmd(uint8_t c)
{
	uint8_t x;
	static uint8_t mc_rx_buf3[CMD_PACKET_LEN];
	for(x = 0; x < CMD_PACKET_LEN-1; x++)
		mc_rx_buf3[x] = mc_rx_buf3[x + 1];
	mc_rx_buf3[CMD_PACKET_LEN-1] = c;
	if(mc_rx_buf3[0] == 'M' && mc_rx_buf3[1] == 'C' && mc_rx_buf3[2] == PAYLOAD_CMD)
	{
		if(checkfcs(&mc_rx_buf3[0], CMD_PACKET_LEN))
		{
			mc_payload_cmd_unpack(&mc_cmd, &mc_rx_buf3[0]);
			mc_cmd.timeout = 25;
			mc_cmd.new_packet = 1;
		}
	}
}





void mc_payload_print_cmd(mc_payload_cmd_t *cmd)
{
  uint8_t x;
	printf("Payload Command\n\r    ");
	for(x = 0; x < 6; x++)
		printf("Speed %d: %d, ", x, cmd->wheel_vel[x]);
	printf("    Enable: %d\n\r", cmd->enable);
}

void mc_payload_print_encoder(mc_payload_encoder_t *enc)
{
  uint8_t x;
	printf("Payload Encoder Values\n\r    ");
	for(x = 0; x < 6; x++)
		printf("Pos %d: %ld, ", x, enc->position[x]);
	printf("\n\r");

}
void mc_payload_print_status(mc_payload_status_t *stat)
{
	uint8_t x;
	printf("Payload Status\n\r");

	printf("MT:       ");
	for(x = 0; x < 6; x++)
		printf("%d, ", stat->motor_temp[x]);

	printf("\n\rDT:     ");
	for(x = 0; x < 6; x++)
		printf("%d, ", stat->drive_temp[x]);

	printf("\n\rAngle   ");
	for(x = 0; x < 6; x++)
		printf("%d, ", stat->angle_sensor[x]);

	printf("\n\rI       ");
	for(x = 0; x < 6; x++)
		printf("%d, ", stat->current[x]);

	printf("\n\rTotal I: %d", stat->battery_current);
	printf("\n\rBatt  V: %d", stat->battery_voltage);
	printf("\n\rTime: %d\n\r", stat->time);
}
