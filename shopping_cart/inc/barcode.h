/*
 * @file barcode.h
 * @brief Header file for barcode.c.
 *
 * @author: Siddhant Jajoo.
 * @date 11/16/2019
 * @copyright Copyright (c) 2019
 *
 */

#ifndef INC_BARCODE_H_
#define INC_BARCODE_H_


#define BARCODE_PREAMBLE		(0xAB)


//Variable Declarations
struct barcode_packet
{
	uint8_t preamble;
	uint16_t payload_size;
	uint8_t cost_size;
	char* payload;
	char* payload_cost;
};


struct barcode_packet barcode_packet;


//Function Declarations
void barcode_test_blocking(void);
void barcode_test_blocking_scanning(void);
void barcode_packet_create(struct barcode_packet* barcode_packet);

#endif /* INC_BARCODE_H_ */
