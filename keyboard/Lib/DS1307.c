#include "DS1307.h"

I2C_HandleTypeDef *hi2c;

void DS1307_Init(I2C_HandleTypeDef *ds1307_i2c)
{
	hi2c = ds1307_i2c;
}

uint8_t Dec_to_BCD(uint8_t num)
{
	return (num/10)<<4|(num%10);
}

uint8_t BCD_to_Dec(uint8_t num)
{
	return (num>>4)*10+(num&0x0F);
}

void DS1307_write(DS1307_Typedef *data)
{
	uint8_t buff_data[8];
	buff_data[0] = 0x00; //start reg address
	buff_data[1] = Dec_to_BCD(data->sec);
	buff_data[2] = Dec_to_BCD(data->min);
	buff_data[3] = Dec_to_BCD(data->hour);
	buff_data[4] = Dec_to_BCD(data->dow);
	buff_data[5] = Dec_to_BCD(data->date);
	buff_data[6] = Dec_to_BCD(data->month);
	buff_data[7] = Dec_to_BCD(data->year);
	HAL_I2C_Master_Transmit(hi2c, RTC_ADD, buff_data, 8, 100);
}

void DS1307_read(DS1307_Typedef *data)
{
	uint8_t buff_data[7];
	uint8_t reg_address = 0;
	HAL_I2C_Master_Transmit(hi2c, RTC_ADD, &reg_address, 1, 100);
	HAL_I2C_Master_Receive(hi2c, RTC_ADD, buff_data, 7, 100);
	data->sec = BCD_to_Dec(buff_data[0]);
	data->min = BCD_to_Dec(buff_data[1]);
	data->hour = BCD_to_Dec(buff_data[2]);
	data->dow = BCD_to_Dec(buff_data[3]);
	data->date = BCD_to_Dec(buff_data[4]);
	data->month = BCD_to_Dec(buff_data[5]);
	data->year = BCD_to_Dec(buff_data[6]);
}

uint8_t DS1307_get_day_of_week(DS1307_Typedef *data)
{
	uint16_t date = data->date;
	uint16_t month = data->month;
	uint16_t year = 2000 + data->year;
	uint16_t dow = (date += month < 3 ? year-- : year-2, 23*month/9 + date + 4 + year/4 - year/100 + year/400)%7;
	return dow;
}

uint8_t DS1307_check_leap_year(uint8_t year)
{
	if((year % 4) == 0 & (year % 100) != 0 || (year % 400) == 0)
	{
		return 1;
	}
	return 0;
}


uint8_t DS1307_get_day_of_month(uint8_t month, uint8_t year)
{
	switch(month)
	{
		case 1: case 3: case 5: case 7: case 8: case 10: case 12:
			return 31;
			break;
		case 4: case 6: case 9: case 11:
			return 30;
			break;
		case 2:
			if(DS1307_check_leap_year(year))
			{
				return 29;
			}
	}
	return 28;
}




