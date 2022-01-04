/*
 * sbus.h
 *
 *  Created on: March 10, 2020
 *      Author: fouinux
 */

#ifndef SBUS_H_
#define SBUS_H_

#include <stdint.h>

enum sbusframe_state{
    FRAME_PENDING = 0,
    FRAME_COMPLETE = (1 << 0),
    FRAME_FAILSAFE = (1 << 1),
    FRAME_DROPPED = (1 << 2),
};

struct sbuschannels
{
	uint16_t Channel_0: 11;
	uint16_t Channel_1: 11;
	uint16_t Channel_2: 11;
	uint16_t Channel_3: 11;
	uint16_t Channel_4: 11;
	uint16_t Channel_5: 11;
	uint16_t Channel_6: 11;
	uint16_t Channel_7: 11;
	uint16_t Channel_8: 11;
	uint16_t Channel_9: 11;
	uint16_t Channel_10: 11;
	uint16_t Channel_11: 11;
	uint16_t Channel_12: 11;
	uint16_t Channel_13: 11;
	uint16_t Channel_14: 11;
	uint16_t Channel_15: 11;
	uint8_t Channel_16: 1;
	uint8_t Channel_17: 1;
	uint8_t FrameLost: 1;
	uint8_t FailSafe: 1;
	uint8_t Zero: 4;
}__attribute__((packed));

struct sbusframe
{
	uint8_t Header;
	struct sbuschannels Channels;
	uint8_t Footer;
}__attribute__((packed));

uint32_t SBUS_Init(void);
uint32_t SBUS_AddByte(uint8_t Byte);
uint32_t SBUS_GetChannel(struct sbuschannels *pChannels);
int32_t SBUS_NormalizeChannel(uint16_t ChannelValue, int32_t Min, int32_t Max);
void SBUS_TimeoutCallback(void);

#endif /* SBUS_H_ */
