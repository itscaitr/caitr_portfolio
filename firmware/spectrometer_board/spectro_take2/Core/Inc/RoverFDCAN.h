/*
 * RoverFDCAN.h
 *
 *  Created on: Nov 16, 2025
 *      Author: basco
 */

#include "stm32g4xx_hal.h"

#ifndef INC_ROVERFDCAN_H_
#define INC_ROVERFDCAN_H_

typedef struct {
	FDCAN_RxHeaderTypeDef hdr;
	uint8_t data[64];
} FDCAN_Frame;

extern volatile uint8_t FDCAN_RxFlag;
extern FDCAN_Frame FDCAN_RxFrame;

HAL_StatusTypeDef FDCAN_Init(FDCAN_HandleTypeDef *canHandle);

HAL_StatusTypeDef FDCAN_Init_Filter(FDCAN_HandleTypeDef *canHandle, FDCAN_FilterTypeDef *filter, uint16_t id);


HAL_StatusTypeDef FDCAN_Transmit(FDCAN_HandleTypeDef *hfdcan, uint16_t std_id, const uint8_t *data, uint8_t len);

#endif /* INC_ROVERFDCAN_H_ */
