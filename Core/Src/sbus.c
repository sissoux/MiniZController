/*
 * sbus.c
 *
 *  Created on: March 10, 2020
 *      Author: fouinux
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <sbus.h>


#define SBUS_HEADER 0x0F
#define SBUS_FOOTER 0x00

#define SBUS_FRAME_SIZE sizeof(struct sbusframe)

uint8_t gByteBuffer[SBUS_FRAME_SIZE];
uint8_t gByteBufferIndex;

uint32_t SBUS_Init(void)
{
	// Reset byte buffer
	memset(&gByteBuffer[0], 0, SBUS_FRAME_SIZE);
	gByteBufferIndex = 0;

	return 0;
}


uint32_t SBUS_AddByte(uint8_t Byte)
{
	if (gByteBufferIndex < SBUS_FRAME_SIZE)
		gByteBuffer[gByteBufferIndex++] = Byte;

	return gByteBufferIndex;
}


uint32_t SBUS_GetChannel(struct sbuschannels *pChannels)
{
	if (NULL == pChannels)
		return FRAME_PENDING;

	// Enough byte received
	if (gByteBufferIndex == SBUS_FRAME_SIZE)
	{
		// Frame with Header and Footer?
		struct sbusframe *pFrame = (struct sbusframe *) gByteBuffer;

		if (pFrame->Header == SBUS_HEADER && pFrame->Footer == SBUS_FOOTER)
		{
			uint32_t ret = FRAME_COMPLETE;

			// Return the frame
			memcpy(&pChannels[0], &gByteBuffer[1], sizeof(struct sbuschannels));

			// Check for frame errors
			if (pChannels->FrameLost)
				ret |= FRAME_DROPPED;

			if (pChannels->FailSafe)
				ret |= FRAME_FAILSAFE;

//			// Debug SBUS
//			char Str[128];
//			sprintf(Str, "%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%03X:%01X:%01X:%01X:%01X\r\n",
//					pChannels->Channel_0,
//					pChannels->Channel_1,
//					pChannels->Channel_2,
//					pChannels->Channel_3,
//					pChannels->Channel_4,
//					pChannels->Channel_5,
//					pChannels->Channel_6,
//					pChannels->Channel_7,
//					pChannels->Channel_8,
//					pChannels->Channel_9,
//					pChannels->Channel_10,
//					pChannels->Channel_11,
//					pChannels->Channel_12,
//					pChannels->Channel_13,
//					pChannels->Channel_14,
//					pChannels->Channel_15,
//					pChannels->Channel_16,
//					pChannels->Channel_17,
//					pChannels->FailSafe,
//					pChannels->FrameLost);
//			CDC_Transmit_FS((uint8_t *)Str, strlen(Str));

			// Reset byte buffer
			gByteBufferIndex = 0;

			return ret;
		}
	}

	return FRAME_PENDING;
}

#define SBUS_CHANNEL_VALUE_MIN	172
#define SBUS_CHANNEL_VALUE_MAX	1811

int32_t SBUS_NormalizeChannel(uint16_t ChannelValue, int32_t Min, int32_t Max)
{
	if (ChannelValue <SBUS_CHANNEL_VALUE_MIN) ChannelValue =SBUS_CHANNEL_VALUE_MIN;
	if (ChannelValue >SBUS_CHANNEL_VALUE_MAX) ChannelValue =SBUS_CHANNEL_VALUE_MAX;

	int32_t ScaleA = Max - Min;
	int32_t ScaleB = (Min * SBUS_CHANNEL_VALUE_MAX) - (Max * SBUS_CHANNEL_VALUE_MIN);
	int32_t ScaleD = SBUS_CHANNEL_VALUE_MAX - SBUS_CHANNEL_VALUE_MIN;

	return ((int32_t) ChannelValue * ScaleA + ScaleB) / ScaleD;
}

void SBUS_TimeoutCallback(void)
{
	// Reset byte buffer
	gByteBufferIndex = 0;
}
