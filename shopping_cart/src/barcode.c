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
#include <stdlib.h>
#include "inc/barcode.h"
#include "inc/leuart.h"



void barcode_packet_create(struct barcode_packet* barcode_packet)
{

	if(barcode_packet->payload == NULL && barcode_packet->payload_cost == NULL
										&& leuart_circbuff.read_index < BUFFER_MAXSIZE)
	{
		int i;

		barcode_packet->preamble = leuart_buffer_pop();

		barcode_packet->payload_size = ((uint16_t)leuart_buffer_pop()) |
				(uint16_t)(leuart_buffer_pop() << 8);

		barcode_packet->cost_size = leuart_buffer_pop();

		barcode_packet->payload = malloc(sizeof(char) * barcode_packet->payload_size);
		if(barcode_packet->payload == NULL)
		{
			printf("ERROR: Cannot Malloc Payload data in barcode_packet_create() function./n");
		}
		//Increment Read Pointer here
		for(i = 0; i < barcode_packet->payload_size; i++)
		{
			leuart_buffer_pop();
		}

		barcode_packet->payload_cost = malloc(sizeof(char) * barcode_packet->cost_size);
		if(barcode_packet->payload_cost == NULL)
		{
			printf("ERROR: Cannot Malloc Payload Cost data in barcode_packet_create() function./n");
		}
		//Increment Read Pointer here
		for(i = 0; i < barcode_packet->cost_size; i++)
		{
			leuart_buffer_pop();
		}

	}
}

//Output should be 2,0,0,2,39,1,SS,SS where SS is checksum value and varies as per the data packet.
void barcode_test_blocking(void)
{
	const uint8_t cmd[9] = {0x7E, 0x00, 0x07, 0x01, 0x00, 0x2A, 0x02, 0xD8, 0x0F};


	//Disable Interrupts over here in order to support blocking
	if ((LEUART0->IEN & LEUART_IEN_RXDATAV) || (LEUART0->IEN & LEUART_IEN_TXC))
	{
		LEUART_IntDisable(LEUART0, LEUART_IEN_RXDATAV | LEUART_IEN_TXC);
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

}


void barcode_test_blocking_scanning(void)
{

	//Disable Interrupts over here in order to support blocking
	if ((LEUART0->IEN & LEUART_IEN_RXDATAV) || (LEUART0->IEN & LEUART_IEN_TXC))
	{
		LEUART_IntDisable(LEUART0, LEUART_IEN_RXDATAV | LEUART_IEN_TXC);
	}


	printf("Testing Barcode UART by scanning the barcode.\n");

	printf("DATA: %x\n", leuart_rcv(LEUART0));
	printf("DATA: %x\n", leuart_rcv(LEUART0));
	printf("DATA: %x\n", leuart_rcv(LEUART0));
	printf("DATA: %x\n", leuart_rcv(LEUART0));
	printf("DATA: %x\n", leuart_rcv(LEUART0));
	printf("DATA: %x\n", leuart_rcv(LEUART0));
	printf("DATA: %x\n", leuart_rcv(LEUART0));
	printf("DATA: %x\n", leuart_rcv(LEUART0));

}

