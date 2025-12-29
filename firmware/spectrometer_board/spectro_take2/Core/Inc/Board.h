/*
 * Board.h
 *
 *  Created on: Nov 16, 2025
 *      Author: basco
 */

#ifndef INC_BOARD_H_
#define INC_BOARD_H_

#include "stm32g4xx_hal.h"

HAL_StatusTypeDef Board_Init(FDCAN_HandleTypeDef *canHandle1);


#endif /* INC_BOARD_H_ */
