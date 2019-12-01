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


#define BUFFER_MAXSIZE					(512)
#define BUFFER_INTERRUPT_SIZE			(10)


//Variable Declaration
struct leuart_circbuff
{
	/* A circular buffer of size 512 bytes to store the contents of the data received from the UART register*/
	char buffer[BUFFER_MAXSIZE];

	/*The write index for the circular buffer*/
	uint16_t write_index;

	/*The read index for the circular buffer*/
	uint16_t read_index;

	/* The buffer count value in order to trigger the gecko_evt_system_external_signal_id event
	   The event would be triggered when buffer_interrupt_count == BUFFER_INTERRUPT_SIZE */
	uint32_t buffer_interrupt_count;

	/*This variable is set true after every BUFFER_INTERRUPT_SIZE bytes are received.
	  If this flag is true the control would be transferred to gecko_evt_system_external_signal_id event */
	bool buffer_interrupt_flag;
};


struct leuart_circbuff leuart_circbuff;


//Function declarations
void leuart_init(void);
void leuart_send(LEUART_TypeDef *leuart, uint8_t data);
char leuart_rcv(LEUART_TypeDef *leuart);
void leuart_loopback_test_blocking(void);
void leuart_loopback_test_non_blocking(void);


/**
 * @brief This function increments the read or write index of the circular buffer based on the
 * maximum size of the circular buffer.
 * @note This function might lead to overwriting of the previous data on index rollover.
 * Developer must make sure to read the old data as soon as possible.
 * @param The read or write index value of the circular buffer
 * @return void
 */
inline uint16_t leuart_circbuff_index_increment(uint16_t index)		//Can make inline
{
	if(index == BUFFER_MAXSIZE - 1)
	{
		index = 0;
	}
	else
	{
		index++;
	}

	return index;
}



#endif /* INC_LEUART_H_ */
