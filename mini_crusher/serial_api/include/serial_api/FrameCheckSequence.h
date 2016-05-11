#pragma once
#include <inttypes.h>

namespace serial_api {
#define INITFCS16    0xffff  /* Initial FCS value */
#define GOODFCS16    0xf0b8  /* Good final FCS value */

    class FrameCheckSequence {
    public:
	static unsigned short fcs16(unsigned short fcs, uint8_t* cp, int len);
	static int addFcs(uint8_t* dat, int len);
	static uint8_t checkFcs(uint8_t* dat, int len);
	static unsigned short compFcs(uint8_t* dat, int len);

	static unsigned short fcstab[256];

    };
} // namespace serial_api


