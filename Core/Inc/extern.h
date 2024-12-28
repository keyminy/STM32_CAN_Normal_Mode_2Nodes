/*
 * extern.h
 *
 *  Created on: Dec 20, 2024
 *      Author: me
 */

#ifndef INC_EXTERN_H_
#define INC_EXTERN_H_
#include "def.h"

extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;
extern uint32_t uart2RxIndex1;
extern uint32_t uart2RxIndex2;
extern uint8_t uart2RxBuf[UART2_BUF_MAX];



#endif /* INC_EXTERN_H_ */
