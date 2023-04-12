#ifndef _DS1307_H
#define _DS1307_H

void rtc_write(unsigned char adress, unsigned char data);

void rtc_read(unsigned char adress);

unsigned char bcd_dec(unsigned char x);

void doc_gio(void);

void doc_ngay(void);

#endif
