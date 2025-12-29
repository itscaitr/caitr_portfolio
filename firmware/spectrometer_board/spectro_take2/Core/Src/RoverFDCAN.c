/*
 * RoverFDCAN.c
 *
 *  Created on: Nov 16, 2025
 *      Author: basco
 */


#include "RoverFDCAN.h"

volatile uint8_t FDCAN_RxFlag;
FDCAN_Frame FDCAN_RxFrame;

HAL_StatusTypeDef FDCAN_Init(FDCAN_HandleTypeDef *canHandle)
{
	HAL_FDCAN_Start(canHandle);
	HAL_FDCAN_ActivateNotification(
	        canHandle,
	        FDCAN_IT_RX_FIFO0_NEW_MESSAGE    |  // RX FIFO0 has a new frame
	        FDCAN_IT_BUS_OFF                 |  // bus-off
	        FDCAN_IT_ERROR_WARNING,             // last error code
	        0U                                  // BufferIndex (only needed for TX interrupts)
	    );
}

HAL_StatusTypeDef FDCAN_Init_Filter(FDCAN_HandleTypeDef *canHandle, FDCAN_FilterTypeDef *filter, uint16_t id)
{
	filter->IdType			= FDCAN_STANDARD_ID;
	filter->FilterType		= FDCAN_FILTER_MASK;
	filter->FilterConfig	= FDCAN_FILTER_TO_RXFIFO0;
	filter->FilterIndex		= 0;
	filter->FilterID1		= id;
	filter->FilterID2		= 0x7FFU;

	return HAL_FDCAN_ConfigFilter(canHandle, filter);
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *canHandle, uint32_t RxFifo0ITs)
{
	if (canHandle->Instance == FDCAN1)
	{
		if (HAL_FDCAN_GetRxMessage(canHandle, FDCAN_RX_FIFO0, &FDCAN_RxFrame.hdr, FDCAN_RxFrame.data) == HAL_OK)
		{
			FDCAN_RxFlag = 1;
		}
	}
}

HAL_StatusTypeDef FDCAN_Transmit(FDCAN_HandleTypeDef *hfdcan, uint16_t std_id, const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef tx = {0};

    tx.Identifier           = std_id;
    tx.IdType               = FDCAN_STANDARD_ID;
    tx.TxFrameType          = FDCAN_DATA_FRAME;
    tx.ErrorStateIndicator  = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch        = FDCAN_BRS_OFF;
    tx.FDFormat             = FDCAN_FD_CAN;
    tx.TxEventFifoControl   = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker        = 0U;

    switch (len) {
        case 0: tx.DataLength = FDCAN_DLC_BYTES_0; break;
        case 1: tx.DataLength = FDCAN_DLC_BYTES_1; break;
        case 2: tx.DataLength = FDCAN_DLC_BYTES_2; break;
        case 3: tx.DataLength = FDCAN_DLC_BYTES_3; break;
        case 4: tx.DataLength = FDCAN_DLC_BYTES_4; break;
        case 5: tx.DataLength = FDCAN_DLC_BYTES_5; break;
        case 6: tx.DataLength = FDCAN_DLC_BYTES_6; break;
        case 7: tx.DataLength = FDCAN_DLC_BYTES_7; break;
        case 8: tx.DataLength = FDCAN_DLC_BYTES_8; break;
    }

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx, (uint8_t*)data);
}
