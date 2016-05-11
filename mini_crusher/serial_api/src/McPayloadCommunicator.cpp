#include <stdio.h>
#include <math.h>

#include <serial_api/McPayloadCommunicator.h>
#include <serial_api/FrameCheckSequence.h>
#include <serial_api/DataSerializer.h>

using namespace serial_api;

uint8_t McPayloadCommunicator::cmdPack(cmd_t *cmd, uint8_t *buf)
{
    uint8_t pos = 0;
    uint8_t x;

    buf[pos++] = 'M';
    buf[pos++] = 'C';
    buf[pos++] = PAYLOAD_CMD;
    buf[pos++] = CMD_PACKET_LEN;

    for(x = 0; x < 6; x++)
	{
	    pos += DataSerializer::write_int16(&buf[pos], cmd->wheel_vel[x]);
	}
    buf[pos++] = cmd->enable ? 1 : 0;
    return FrameCheckSequence::addFcs(buf, pos);
    // 19
}

uint8_t McPayloadCommunicator::statusPack(status_t *stat, uint8_t *buf)
{
    uint8_t pos = 0;
    uint8_t x;

    buf[pos++] = 'M';
    buf[pos++] = 'C';
    buf[pos++] = PAYLOAD_STATUS;
    buf[pos++] = STATUS_PACKET_LEN;


    for(x = 0; x < 6; x++)
	{
	    pos += DataSerializer::write_uint8(&buf[pos], stat->motor_temp[x]);
	}

    for(x = 0; x < 6; x++)
	{
	    pos += DataSerializer::write_uint8(&buf[pos], stat->drive_temp[x]);
	}
    for(x = 0; x < 6; x++)
	{
	    pos += DataSerializer::write_uint8(&buf[pos], stat->angle_sensor[x]);
	}

    for(x = 0; x < 6; x++)
	{
	    pos += DataSerializer::write_int16(&buf[pos], stat->current[x]);
	}

    pos += DataSerializer::write_int16(&buf[pos], stat->battery_current);
    pos += DataSerializer::write_int16(&buf[pos], stat->battery_voltage);

    pos += DataSerializer::write_uint16(&buf[pos], stat->time);

    return FrameCheckSequence::addFcs(buf, pos);
    // 42
}

uint8_t McPayloadCommunicator::encoderPack(encoder_t *enc, uint8_t *buf)
{
    uint8_t pos = 0;
    uint8_t x;

    buf[pos++] = 'M';
    buf[pos++] = 'C';
    buf[pos++] = PAYLOAD_ENCODER;
    buf[pos++] = ENCODER_PACKET_LEN;

    for(x = 0; x < 6; x++)
	{
	    pos += DataSerializer::write_int32(&buf[pos], enc->position[x]);
	}
    return FrameCheckSequence::addFcs(buf, pos);
    // 30
}


uint8_t McPayloadCommunicator::cmdUnpack(cmd_t *cmd, uint8_t *buf)
{
    if(buf[0] != 'M' || buf[1] != 'C' || buf[2] != PAYLOAD_CMD)
	{
	    printf("Wrong Packet\n\r");
	    return 0;
	}
    if(!FrameCheckSequence::checkFcs(buf, CMD_PACKET_LEN))
	{
	    printf("FCS Failed\n\r");
	    return 0;
	}
    uint8_t pos = 0;
    uint8_t x;
    pos = 4;
    for(x = 0; x < 6; x++)
	{

	    cmd->wheel_vel[x] = DataSerializer::read_int16(&buf[pos]);
	    pos += 2;
	}
    cmd->enable = buf[pos++] & 0x01;

    return 1;

}

uint8_t McPayloadCommunicator::statusUnpack(status_t *stat, uint8_t *buf)
{

    if(buf[0] != 'M' || buf[1] != 'C' || buf[2] != PAYLOAD_STATUS)
	{
	    printf("Wrong Packet\n\r");
	    return 0;
	}
    if(!FrameCheckSequence::checkFcs(buf, STATUS_PACKET_LEN))
	{
	    printf("FCS Failed\n\r");
	    return 0;
	}
    uint8_t pos = 0;
    uint8_t x;
    pos = 4;


    for(x = 0; x < 6; x++)
	{
	    stat->motor_temp[x] = DataSerializer::read_uint8(&buf[pos]);
	    pos += 1;
	}
    for(x = 0; x < 6; x++)
	{
	    stat->drive_temp[x] = DataSerializer::read_uint8(&buf[pos]);
	    pos += 1;
	}
    for(x = 0; x < 6; x++)
	{
	    stat->angle_sensor[x] = DataSerializer::read_uint8(&buf[pos]);
	    pos += 1;
	}
    for(x = 0; x < 6; x++)
	{
	    stat->current[x] = DataSerializer::read_int16(&buf[pos]);
	    pos += 2;
	}
    stat->battery_current = DataSerializer::read_int16(&buf[pos]);
    pos += 2;
    stat->battery_voltage = DataSerializer::read_int16(&buf[pos]);
    pos += 2;

    stat->time = DataSerializer::read_int16(&buf[pos]);
    pos += 2;

    return 1;
}

uint8_t McPayloadCommunicator::encoderUnpack(encoder_t *enc, uint8_t *buf)
{
    if(buf[0] != 'M' || buf[1] != 'C' || buf[2] != PAYLOAD_ENCODER)
	{
	    printf("Wrong Packet\n\r");
	    return 0;
	}
    if(!FrameCheckSequence::checkFcs(buf, ENCODER_PACKET_LEN))
	{
	    printf("FCS Failed\n\r");
	    return 0;
	}
    uint8_t pos = 0;
    uint8_t x;
    pos = 4;
    for(x = 0; x < 6; x++)
	{
	    enc->position[x] = DataSerializer::read_int32(&buf[pos]);
	    pos += 4;
	}
    return 1;
}

void McPayloadCommunicator::recvCmd(uint8_t c)
{
    uint8_t x;
    for(x = 0; x < CMD_PACKET_LEN-1; x++)
	read_buffer_cmd[x] = read_buffer_cmd[x + 1];
    read_buffer_cmd[CMD_PACKET_LEN-1] = c;
    if(read_buffer_cmd[0] == 'M' && read_buffer_cmd[1] == 'C' && read_buffer_cmd[2] == PAYLOAD_CMD)
	{
	    if(FrameCheckSequence::checkFcs(&read_buffer_cmd[0], CMD_PACKET_LEN))
		{
		    cmdUnpack(&cmd, &read_buffer_cmd[0]);
		    cmd.timeout = 25;
		    cmd.new_packet = 1;
		}
	}
}

void McPayloadCommunicator::recvStatus(uint8_t c)
{
    uint8_t x;
    for(x = 0; x < STATUS_PACKET_LEN-1; x++)
	read_buffer_stat[x] = read_buffer_stat[x + 1];
    read_buffer_stat[STATUS_PACKET_LEN-1] = c;
    if(read_buffer_stat[0] == 'M' && read_buffer_stat[1] == 'C' && read_buffer_stat[2] == PAYLOAD_STATUS)
	{
	    if(FrameCheckSequence::checkFcs(&read_buffer_stat[0], STATUS_PACKET_LEN))
		{
		    statusUnpack(&stat, &read_buffer_stat[0]);
		    stat.new_packet = 1;
		}
	}
}

void McPayloadCommunicator::recvEncoder(uint8_t c)
{
    uint8_t x;
    for(x = 0; x < ENCODER_PACKET_LEN-1; x++)
	read_buffer_enc[x] = read_buffer_enc[x + 1];
    read_buffer_enc[ENCODER_PACKET_LEN-1] = c;
    if(read_buffer_enc[0] == 'M' && read_buffer_enc[1] == 'C' && read_buffer_enc[2] == PAYLOAD_ENCODER)
	{
	    if(FrameCheckSequence::checkFcs(&read_buffer_enc[0], ENCODER_PACKET_LEN))
		{
		    encoderUnpack(&enc, &read_buffer_enc[0]);
		    enc.new_packet = 1;
		}
	}
}

void McPayloadCommunicator::printCmd(cmd_t *cmd)
{
    uint8_t x;
    printf("Payload Command\n\r    ");
    for(x = 0; x < 6; x++)
	printf("Speed %d: %d, ", x, cmd->wheel_vel[x]);
    printf("    Enable: %d\n\r", cmd->enable);
}

void McPayloadCommunicator::printStatus(status_t *stat)
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

void McPayloadCommunicator::printEncoder(encoder_t *enc)
{
    uint8_t x;
    printf("Payload Encoder Values\n\r    ");
    for(x = 0; x < 6; x++)
	printf("Pos %d: %ld, ", x, enc->position[x]);
    printf("\n\r");

}
