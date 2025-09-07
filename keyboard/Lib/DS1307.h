#ifndef __DS1307_H
#define __DS1307_H

#include "stm32f1xx.h"

#define RTC_ADD (0x68<<1)

typedef struct{
	int8_t sec;
	int8_t min;
	int8_t hour;
	int8_t dow;
	int8_t date;
	int8_t month;
	int8_t year;
}DS1307_Typedef;

void DS1307_Init(I2C_HandleTypeDef *ds1307_i2c);
uint8_t Dec_to_BCD(uint8_t num);
uint8_t BCD_to_Dec(uint8_t num);
void DS1307_write(DS1307_Typedef *data);
void DS1307_read(DS1307_Typedef *data);
uint8_t DS1307_get_day_of_week(DS1307_Typedef *data);
uint8_t DS1307_check_leap_year(uint8_t year);
uint8_t DS1307_get_day_of_month(uint8_t month, uint8_t year);
#endif

