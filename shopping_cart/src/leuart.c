/*
 * @file leuart.c
 * @brief This file consists of functions related to LEUART peripheral.
 *
 * @author: Siddhant Jajoo.
 * @date 11/05/2019
 * @copyright Copyright (c) 2019
 *
 */

#include <stdio.h>
#include "em_core.h"
#include "em_cmu.h"
#include "native_gecko.h"
#include "inc/leuart.h"
#include "inc/connection_param.h"
#include "inc/external_events.h"




/**
 * @brief This function increments the buffer_interrupt_count variable based on the BUFFER_INTERRUPT_SIZE value.
 * @param The buffer_interrupt_count value stored in the leuart_circbuff structure
 * @return void

static uint32_t leuart_circbuff_interrupt_count_test_increment(uint32_t index)		//Can make inline
{
	if(index == BUFFER_INTERRUPT_SIZE)
	{
		index = 0;
	}
	else
	{
		index++;
	}

	return index;
}
 */

/**
 * @brief Interrupt handler for LEUART
 * @param void
 * @return void
 */
void LEUART0_IRQHandler(void)
{
	//Disable All Interrupts
	CORE_AtomicDisableIrq();

	//Acknowledge and Clear the Interrupt
	uint32_t flags = LEUART_IntGet(LEUART0);
	LEUART_IntClear(LEUART0, LEUART_IF_TXC);

	// RX portion of the interrupt handler
	if (flags & LEUART_IF_RXDATAV)
	{

		//TODO: Need to implement a Software Timer as well here for triggering external event ID.
		if(leuart_circbuff.buffer_interrupt_count == BUFFER_INTERRUPT_SIZE)
		{
			//Update the External Event after every BUFFER_INTERRUPT_SIZE(define in leuart.h) bytes of receiving data
			external_event |= LEUART_EVENT;
			gecko_external_signal(external_event);
			leuart_circbuff.buffer_interrupt_count = 0;
		}
		else
		{
			leuart_circbuff.buffer_interrupt_count++;
		}

		while (LEUART0->STATUS & LEUART_STATUS_RXDATAV)
		{
			// While there is still incoming data
			leuart_buffer_push();
		}
	}

	//Enable All Interrupts
	CORE_AtomicEnableIrq();
}


/**
 * @brief Function to initialize the GPIO pins required for LEUART.
 * @param void
 * @return void
 */
static void leuart_gpio_init(void)
{
	// GPIO clock
	CMU_ClockEnable(cmuClock_GPIO, true);

	// Initialize LEUART0 TX and RX pins
	GPIO_PinModeSet(gpioPortD, 10, gpioModePushPull, 1); 	// TX (Pin Number 7)
	GPIO_PinModeSet(gpioPortD, 11, gpioModeInputPull, 1);   // RX (Pin Number 9)
}


/**
 * @brief Initialization function for LEUART0. This function enables the required clock and GPIO peripherals.
 * @param void
 * @return void
 */
void leuart_init(void)
{
	//Enabling GPIO required for LEUART.
	leuart_gpio_init();

	// Enable clocks for LEUART0
	CMU_ClockEnable(cmuClock_LEUART0, true);
	CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_1); // Don't prescale LEUART clock

	// Initialize the LEUART0 module
	LEUART_Init_TypeDef init = LEUART_INIT_DEFAULT;
	LEUART_Init(LEUART0, &init);

	// Enable LEUART0 RX/TX pins
	LEUART0->ROUTEPEN |= LEUART_ROUTEPEN_RXPEN | LEUART_ROUTEPEN_TXPEN;

	LEUART0->ROUTELOC0 |= (LEUART0->ROUTELOC0 & (~_LEUART_ROUTELOC0_TXLOC_MASK)) | LEUART_ROUTELOC0_TXLOC_LOC18;
	LEUART0->ROUTELOC0 |= (LEUART0->ROUTELOC0 & (~_LEUART_ROUTELOC0_RXLOC_MASK)) | LEUART_ROUTELOC0_RXLOC_LOC18;

	// Enable LEUART0 RX/TX interrupts
	LEUART_IntEnable(LEUART0, LEUART_IEN_RXDATAV);
	//LEUART_IntEnable(LEUART0, LEUART_IEN_TXC);
	NVIC_EnableIRQ(LEUART0_IRQn);
}


/**
 * @brief Function to send data using LEUART peripheral.
 * @param LEUART_TypeDef* The LEUART peripheral being used. eg: LEUART0.
 * @param uint8_data 8 bit data to be sent.
 * @return void
 */
void leuart_send(LEUART_TypeDef *leuart, uint8_t data)
{
	uint8_t tx_data = data;
	LEUART_Tx(leuart, tx_data);
}


/**
 * @brief Function to receive data using LEUART Peripheral.
 * @param LEUART_TypeDef* The LEUART peripheral being used. eg: LEUART0.
 * @return void
 */
char leuart_rcv(LEUART_TypeDef *leuart)
{
	return LEUART_Rx(leuart);
}


/**
 * @brief This function pushes the data received over UART into the Circular Buffer. i.e leuart_circbuff
 * @note This function must be enclosed in CORE_AtomicDisableIrq() and CORE_AtomicEnableIrq();
 * @param void
 * @return void
 */
void leuart_buffer_push(void)
{
	if(leuart_circbuff.write_index < BUFFER_MAXSIZE)
	{
		//Save UART register data into the circular buffer
		leuart_circbuff.buffer[leuart_circbuff.write_index] = leuart_rcv(LEUART0);

		//Increment the write index value
		leuart_circbuff.write_index = leuart_circbuff_index_increment(leuart_circbuff.write_index);

		//Incrementing the buffer count variable if the buffer is not full or else keeping it the same i.e the maximum value here according to the if loop.
		if(leuart_circbuff.buffer_count < BUFFER_MAXSIZE)
		{
			leuart_circbuff.buffer_count++;
		}
	}
}



/**
 * @brief This function pops and returns the data from the Circular Buffer. i.e leuart_circbuff
 * @param void
 * @return Data from the circular buffer using the read_index. -1 signifies no valid data present
 */
char leuart_buffer_pop(void)
{
	if((leuart_circbuff.read_index < BUFFER_MAXSIZE) && (leuart_circbuff.buffer_count > 0))
	{
		uint32_t temp_read_index = leuart_circbuff.read_index;

		//Increment the write index value
		leuart_circbuff.read_index = leuart_circbuff_index_increment(leuart_circbuff.read_index);

		//Decrementing the buffer count variable if the buffer is not empty.
		CORE_AtomicDisableIrq();
		leuart_circbuff.buffer_count--;
		CORE_AtomicEnableIrq();

		return leuart_circbuff.buffer[temp_read_index];
	}

	//TODO: Check return value here with datatype of the function
	return -1;
}


bool leuart_buffer_empty_status(void)
{
	if(leuart_circbuff.buffer_count == 0)
	{
		return true;
	}

	return false;
}



/**
 * @brief Function to test LEUART0 peripheral in loopback using blocking mode.
 * @note This function should be called only after initializing LEUART0 peripheral using leuart_init().
 * @param void
 * @return void
 */
void leuart_loopback_test_blocking(void)
{
	const uint8_t cmd[9] = {0x7E, 0x00, 0x07, 0x01, 0x00, 0x2A, 0x02, 0xD8, 0x0F};

	//Disable Interrupts over here in order to support blocking
	if ((LEUART0->IEN & LEUART_IEN_RXDATAV) || (LEUART0->IEN & LEUART_IEN_TXC))
	{
		LEUART_IntDisable(LEUART0, LEUART_IEN_RXDATAV | LEUART_IEN_TXC);
	}

	leuart_send(LEUART0, cmd[0]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

	leuart_send(LEUART0, cmd[1]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

	leuart_send(LEUART0, cmd[2]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

	leuart_send(LEUART0, cmd[3]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

	leuart_send(LEUART0, cmd[4]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

	leuart_send(LEUART0, cmd[5]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

	leuart_send(LEUART0, cmd[6]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

	leuart_send(LEUART0, cmd[7]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

	leuart_send(LEUART0, cmd[8]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

}


/**
 * @brief Function to test LEUART0 peripheral in loopback using interrupt mode.
 * @note This function should be called only after initializing LEUART0 peripheral using leuart_init().
 * @param void
 * @return void
 */
void leuart_loopback_test_non_blocking(void)
{
	const uint8_t cmd[9] = {0x7E, 0x00, 0x07, 0x01, 0x00, 0x2A, 0x02, 0xD8, 0x0F};


	leuart_send(LEUART0, cmd[0]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

	leuart_send(LEUART0, cmd[1]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

	leuart_send(LEUART0, cmd[2]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

	leuart_send(LEUART0, cmd[3]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

	leuart_send(LEUART0, cmd[4]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

	leuart_send(LEUART0, cmd[5]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

	leuart_send(LEUART0, cmd[6]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

	leuart_send(LEUART0, cmd[7]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

	leuart_send(LEUART0, cmd[8]);
	printf("Data Sent\n");
	printf("DATA: %x\n", leuart_rcv(LEUART0));

}

