#ifndef CRC16_H
#define CRC16_H

extern unsigned short frame_crc;

extern unsigned short calc_crc(unsigned char *buf,int count); 
extern void init_crc(void);

#endif
