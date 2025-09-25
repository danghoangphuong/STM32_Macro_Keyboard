#include "Button_matrix.h"

uint8_t history[NUM_ROW][NUM_COL] = {0}; //save bit history and perform shift register
uint8_t stable_state[NUM_ROW][NUM_COL] = {0}; 
uint32_t key_x_press_time[NUM_ROW][NUM_COL] = {0}; // save time when key[r][c] start press
uint32_t key_x_spam_time[NUM_ROW][NUM_COL] = {0};
uint8_t key_x_spam_active[NUM_ROW][NUM_COL] = {0};	// spam flag

__weak void Matrix_key_press_Callback(uint8_t key, KeyEvent_t key_event) // Callback
{
	
}

void Matrix_handle(Matrix_Typedef *Matrix_X)
{
    for (uint8_t i = 0; i < NUM_ROW; i++)
    {
        for (uint8_t r = 0; r < NUM_ROW; r++) 
		{
            HAL_GPIO_WritePin(Matrix_X->row_pins[r].port, Matrix_X->row_pins[r].pin, (r == i) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }
        for (uint8_t c = 0; c < NUM_COL; c++)
        {
            uint8_t sample = HAL_GPIO_ReadPin(Matrix_X->col_pins[c].port, Matrix_X->col_pins[c].pin) ? 1 : 0;
            history[i][c] = (history[i][c] << 1) | sample;
			uint32_t now = HAL_GetTick();
			//===================Single press==============================
            if ((history[i][c] & 0xFF) == 0x00)
            {
				if (!stable_state[i][c] && (now - key_x_press_time[i][c] > 5)) // trigger when stable_state = 0
				{
					stable_state[i][c] = 1;
					key_x_press_time[i][c] = now;
					key_x_spam_active[i][c] = 0;
					key_x_spam_time[i][c] = now;
					Matrix_X->last_key = i * NUM_COL + c + 1;
					Matrix_key_press_Callback(Matrix_X->last_key, KEY_PRESS);
				}  
            }
            else if ((history[i][c] & 0xFF) == 0xFF)
            {
				if (stable_state[i][c] && (now - key_x_press_time[i][c] > 5))
				{
					stable_state[i][c] = 0;
					key_x_spam_active[i][c] = 0;
					key_x_spam_time[i][c] = 0;
					Matrix_X->last_key = i * NUM_COL + c + 1;
					Matrix_key_press_Callback(Matrix_X->last_key, KEY_RELEASE);
				}		
            }
			// ======================Spaming==============================
//			if(stable_state[i][c] == 1)
//			{
//				if(key_x_spam_active[i][c] == 0)
//				{
//					if(now - key_x_spam_time[i][c] > SPAM_AFTER)
//					{
//						key_x_spam_active[i][c] = 1;
//						key_x_spam_time[i][c] = now;
//						Matrix_X->last_key = i * NUM_COL + c + 1;
//						Matrix_key_press_Callback(Matrix_X->last_key, KEY_SPAM);
//					}
//				}
//				else
//				{
//					if(now - key_x_spam_time[i][c] > SPAM_RATE)
//					{
//						key_x_spam_time[i][c] = now;
//						Matrix_X->last_key = i * NUM_COL + c + 1;
//						Matrix_key_press_Callback(Matrix_X->last_key, KEY_SPAM);
//					}
//				}
//			}
        }
    }
}



// example of initializing matrix
//Matrix_Pin_Typedef rowS[E] = {
//	{GPIOx, GPIO_PIN_a}, // ROW 1 - 4
//	{GPIOy, GPIO_PIN_b},
//	{GPIOz, GPIO_PIN_c},
//	{GPIOt, GPIO_PIN_d},
//};

//Matrix_Pin_Typedef colS[F] = {
//	{GPIOf, GPIO_PIN_n}, // col 1 - 4
//	{GPIOg, GPIO_PIN_m},
//	{GPIOh, GPIO_PIN_o},
//	{GPIOk, GPIO_PIN_p},
//};


//Matrix_Typedef mMatrix = {
//	.num_row = E, 
//	.num_col = F,
//	.row_pins = rowS,
//	.col_pins = colS
//};
