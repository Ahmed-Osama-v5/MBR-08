/*
 * modbus_timer.h
 *
 *  Created on: 3 Jun 2026
 *      Author: Osama
 */

#ifndef INC_MODBUS_TIMER_H_
#define INC_MODBUS_TIMER_H_


#include <stdint.h>

/* Supported baud rates via 2-pin GPIO encoding */
typedef enum
{
    MODBUS_BAUD_1200   = 1200U,
    MODBUS_BAUD_9600   = 9600U,
    MODBUS_BAUD_19200  = 19200U,
    MODBUS_BAUD_115200 = 115200U,
} ModbusBaud_t;

void     ModbusTimer_Init(ModbusBaud_t baud);
void     ModbusTimer_Restart(void);   /* call from USART RXNE IRQ */
void     ModbusTimer_Stop(void);

/* Weak callback — override in application */
void     ModbusTimer_FrameDoneCallback(void);
uint32_t Modbus_GetArrValue(void);


#endif /* INC_MODBUS_TIMER_H_ */
