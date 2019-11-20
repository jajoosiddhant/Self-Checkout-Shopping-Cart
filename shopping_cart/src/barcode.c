/*
 * @file barcode.c
 * @brief This file consists of functions related to Barcode Sensor.
 *
 * @author: Siddhant Jajoo.
 * @date 11/16/2019
 * @copyright Copyright (c) 2019
 *
 */


#include <stdio.h>
#include "inc/barcode.h"
#include "inc/leuart.h"



void barcode_test_blocking(void)
{
	const uint8_t cmd[9] = {0x7E, 0x00, 0x07, 0x01, 0x00, 0x2A, 0x02, 0xD8, 0x0F};


	//Disable Interrupts over here in order to support blocking
	if ((LEUART0->IEN & LEUART_IEN_RXDATAV) || (LEUART0->IEN & LEUART_IEN_TXC))
	{
		LEUART_IntEnable(LEUART0, LEUART_IEN_RXDATAV | LEUART_IEN_TXC);
		NVIC_DisableIRQ(LEUART0_IRQn);
	}


	leuart_send(LEUART0, cmd[0]);
	printf("Data Sent\n");
	leuart_send(LEUART0, cmd[1]);
	printf("Data Sent\n");
	leuart_send(LEUART0, cmd[2]);
	printf("Data Sent\n");
	leuart_send(LEUART0, cmd[3]);
	printf("Data Sent\n");
	leuart_send(LEUART0, cmd[4]);
	printf("Data Sent\n");
	leuart_send(LEUART0, cmd[5]);
	printf("Data Sent\n");
	leuart_send(LEUART0, cmd[6]);
	printf("Data Sent\n");
	leuart_send(LEUART0, cmd[7]);
	printf("Data Sent\n");
	leuart_send(LEUART0, cmd[8]);
	printf("Data Sent\n");


	printf("DATA: %x\n", leuart_rcv(LEUART0));
	printf("DATA: %x\n", leuart_rcv(LEUART0));
	printf("DATA: %x\n", leuart_rcv(LEUART0));
	printf("DATA: %x\n", leuart_rcv(LEUART0));
	printf("DATA: %x\n", leuart_rcv(LEUART0));
	printf("DATA: %x\n", leuart_rcv(LEUART0));
	printf("DATA: %x\n", leuart_rcv(LEUART0));
	printf("DATA: %x\n", leuart_rcv(LEUART0));
	printf("DATA: %x\n", leuart_rcv(LEUART0));

}

