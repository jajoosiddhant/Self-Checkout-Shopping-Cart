/*
 * @file leuart.h
 * @brief Header file for leuart.c.
 *
 * @author: Siddhant Jajoo.
 * @date 11/05/2019
 * @copyright Copyright (c) 2019
 *
 */
#ifndef INC_LEUART_H_
#define INC_LEUART_H_

#include <stdbool.h>
#include "em_leuart.h"


#define LEUART_BUFFER_MAXSIZE					(512)
#define LEUART_BUFFER_INTERRUPT_SIZE			(10)
#define LEUART_INTERRUPT_TIMER					(1)


/* Variable Declaration */
struct leuart_circbuff
{
	/* A circular buffer of size 512 bytes to store the contents of the data received from the UART register*/
	char buffer[LEUART_BUFFER_MAXSIZE];

	/*The write index for the circular buffer*/
	uint32_t write_index;

	/*The read index for the circular buffer*/
	uint32_t read_index;

	/*The current number of elements is stored here*/
	volatile uint32_t buffer_count;

	/* The buffer count value in order to trigger the gecko_evt_system_external_signal_id event
	   The event would be triggered when buffer_interrupt_count == BUFFER_INTERRUPT_SIZE */
	volatile uint32_t buffer_interrupt_count;

	/*This variable is set true after every BUFFER_INTERRUPT_SIZE bytes are received.
	  If this flag is true the control would be transferred to gecko_evt_system_external_signal_id event */
	volatile bool buffer_interrupt_flag;
};


struct leuart_circbuff leuart_circbuff;					/* Only one instance of leuart buffer since
															there is only one leuart peripheral available i.e LEUART0*/


/* Function declarations */
void leuart_init(void);
void leuart_send(LEUART_TypeDef *leuart, uint8_t data);
char leuart_rcv(LEUART_TypeDef *leuart);
uint32_t leuart_circbuff_index_increment(uint32_t index);
void leuart_buffer_push(void);
char leuart_buffer_pop(void);
bool leuart_buffer_empty_status(void);
void leuart_loopback_test_blocking(void);
void leuart_loopback_test_non_blocking(void);



#endif /* INC_LEUART_H_ */
