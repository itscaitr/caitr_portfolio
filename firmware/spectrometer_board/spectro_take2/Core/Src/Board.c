/*
 * Board.c
 *
 *  Created on: Nov 16, 2025
 *      Author: basco
 */

#include "Board.h"

HAL_StatusTypeDef Board_Init(FDCAN_HandleTypeDef *canHandle1)
{
	//FDCAN_FilterTypeDef f1 = {0};
	//FDCAN_Init_Filter(canHandle1, f1, 0x01);
	FDCAN_Init(canHandle1);
}

