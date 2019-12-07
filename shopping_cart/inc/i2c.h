/*
 * @file i2c.h
 * @brief Header file for i2c.c
 *
 * @author: Satya Mehta.
 * @date 11/16/2019
 * @copyright Copyright (c) 2019
 *
 */
#include <stdint.h>
#include "em_i2c.h"
#include "src/gpio.h"
#include "em_core.h"


#define NXP_NTAG_R (0x05) //NXP NTAG NFC Read Command
#define NXP_NTAG_W (0x04) //NXP NTAG NFC Write Command

/*function declarations*/

void i2cinit(void);
void i2c_write_poll(uint8_t,uint8_t *);
void i2cdisable(void);
uint8_t* i2c_read_poll(uint8_t);
uint8_t read_session_poll(uint8_t);

/*Global Variable*/
uint8_t interrupt_flag_ack;
uint8_t interrupt_flag_rxdata;
uint8_t read[16];
uint8_t write[16];
