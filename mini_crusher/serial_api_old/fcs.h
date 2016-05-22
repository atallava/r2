#ifndef _FCS_H_
#define _FCS_H_

unsigned short fcs16(unsigned short fcs, uint8_t* cp, int len);
int addfcs(uint8_t* dat, int len);
uint8_t checkfcs(uint8_t* dat, int len);
unsigned short compfcs(uint8_t* dat, int len);

#endif

