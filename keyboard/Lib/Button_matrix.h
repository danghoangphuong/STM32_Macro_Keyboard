#ifndef __BUTTON_MATRIX_H
#define __BUTTON_MATRIX_H

#include "stm32f1xx.h"
#include "stdio.h"
#include "stdarg.h"

#define NUM_ROW 4	// modify this
#define NUM_COL 4	// modify this
#define SPAM_AFTER 600 // if hold button, after 600ms spam key
#define SPAM_RATE 100 // after 600ms when in spam mode => spam key every 100ms

typedef struct
{
	GPIO_TypeDef *port;
	uint16_t pin;
}Matrix_Pin_Typedef;

typedef struct
{
	uint8_t last_key;
	uint8_t num_row;
	uint8_t num_col;
	Matrix_Pin_Typedef *row_pins;
	Matrix_Pin_Typedef *col_pins;
}Matrix_Typedef;

typedef enum
{
	KEY_PRESS,
	KEY_RELEASE,
	KEY_SPAM
}KeyEvent_t;

void Matrix_handle(Matrix_Typedef *Matrix_X);
#endif

