/*
 * @file i2c.c
 * @brief This file consists of functions related to I2C0 peripheral.
 *
 * @author: Satya Mehta
 * @date 11/05/2019
 * @copyright Copyright (c) 2019
 *
 */

#include "src/i2c.h"


/**
 * @brief This function is used to initialize I2C0 peripheral.
 * PortC 10 SCL, PortC 11 SDA is used.
 */
void i2cinit()
{

	GPIO_PinModeSet(gpioPortC, 10, gpioModeWiredAnd, 1); // SCL Before
	GPIO_PinModeSet(gpioPortC, 11, gpioModeWiredAnd, 1);//SDA
	GPIO_PinModeSet(gpioPortD, 15, gpioModePushPull, 1);
	GPIO_PinOutSet(gpioPortD, 15);

	for (int k=0;k<200000;k++); //Additional delay before ROUTE

	I2C0 -> ROUTEPEN = I2C_ROUTEPEN_SCLPEN | I2C_ROUTEPEN_SDAPEN;
	I2C0->ROUTELOC0 |= (I2C0->ROUTELOC0 & (~_I2C_ROUTELOC0_SCLLOC_MASK))| I2C_ROUTELOC0_SCLLOC_LOC14;
	I2C0->ROUTELOC0 |= (I2C0->ROUTELOC0 & (~_I2C_ROUTELOC0_SDALOC_MASK))| I2C_ROUTELOC0_SDALOC_LOC16;

	const I2C_Init_TypeDef i2cinitialization = I2C_INIT_DEFAULT; //Default initialization

	I2C_Init(I2C0, &i2cinitialization);
	delay();
	I2C_Enable(I2C0,true);
	delay();

	/*Toggle SCL for 9 times as specified in datasheet*/
	for (int i=0; i<9; i++)
	{
		GPIO_PinOutClear(gpioPortC,10);
		GPIO_PinOutSet(gpioPortC,10);

	}
	for (int k=0;k<200000;k++);
	if(I2C0->STATE & I2C_STATE_BUSY)
	{
		I2C0->CMD = I2C_CMD_ABORT;
		//NVIC_EnableIRQ(I2C0_IRQn);
	}
	NVIC_EnableIRQ(I2C0_IRQn);
	I2C_IntEnable(I2C0, I2C_IFC_NACK);
}


/**
 * @brief This function is a polling write driver for NXP NTAG I2C NFC Device.
 * 16 Bytes are written in a single I2C write transfer.
 * @param add - Address of the starting register you need to write data into. Following 15 bytes will also be written.
 * 		  data - Starting address of the data which is to be written into NFC.
 * @return None.
 */
void i2c_write_poll(uint8_t add, uint8_t *data)
{
	I2C0->CMD = I2C_CMD_START;  //send start bit
	I2C0->TXDATA = 0x04; //NXP NTAG Address.


	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->TXDATA = add;
	uint8_t index = 0;

	/*Writing 16 Bytes */
	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;
	I2C0->TXDATA = data[0];

	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;
	I2C0->TXDATA = data[1];

	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;
	I2C0->TXDATA = data[2];

	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->TXDATA = data[3];
	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->TXDATA = data[4];
	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->TXDATA = data[5];
	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->TXDATA = data[6];
	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->TXDATA = data[7];
	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->TXDATA = data[8];
	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->TXDATA = data[9];
	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->TXDATA = data[10];
	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->TXDATA = data[11];
	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->TXDATA = data[12];
	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->TXDATA = data[13];
	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->TXDATA = data[14];
 	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->TXDATA = data[15];
	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->CMD = I2C_CMD_STOP;
	while(I2C0->CMD & I2C_STATUS_PSTOP);
}

/**
 * @brief This function is used to read the session register inside the NFC using the interrupts
 * @param session_register - Address of the session register which is read.
 * @return uint8_t - Returns data read from the session register.
 */
uint8_t read_session(uint8_t session_register)
{
	I2C0->CMD = I2C_CMD_START;  //send start bit
	I2C0->TXDATA = 0x04; //slave address.
	delay();
	if(interrupt_flag_ack && ((I2C0->IF & I2C_IF_ACK) == 0))
	{
		interrupt_flag_ack = 0;
		I2C0->TXDATA = 0xFE;
		delay();
	}
	if(interrupt_flag_ack && ((I2C0->IF & I2C_IF_ACK) == 0))
	{
		interrupt_flag_ack = 0;
		I2C0->TXDATA = session_register;
	}
	if(interrupt_flag_ack && ((I2C0->IF & I2C_IF_ACK) == 0))
	{
		interrupt_flag_ack = 0;
		I2C0->CMD = I2C_CMD_STOP;
	}
	delay(); //Required for EEPROM Writing

	for(int i = 0; i < 10; i++);
	I2C0->CMD = I2C_CMD_START;
	I2C0->TXDATA = NXP_NTAG_R;
	uint8_t data = 0;
	if(interrupt_flag_ack && ((I2C0->IF & I2C_IF_ACK) == 0))
	{
		interrupt_flag_ack = 0;
	}
	while(I2C0->IF & I2C_IF_RXDATAV)
	{
		data = I2C0->RXDATA;
		delay();
	}

	I2C0->CMD = I2C_CMD_ACK;
	I2C0->CMD = I2C_CMD_STOP;
	delay();
	return data;

}

/**
 * @brief This function is used to read the session register inside the NFC using polling method
 * @param session_register - Address of the session register which is read.
 * @return uint8_t - Returns data read from the session register.
 */
uint8_t read_session_poll(uint8_t session_register)
{
	 //slave address.
	I2C0->CMD = I2C_CMD_START;  //send start bit
	I2C0->TXDATA = 0x04;

	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;
	//if(interrupt_flag_ack)

	I2C0->TXDATA = 0xFE;

	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;
	//if(interrupt_flag_ack)

	I2C0->TXDATA = session_register;
	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->CMD = I2C_CMD_STOP;
	delay(); //Required for EEPROM Writing


	I2C0->CMD = I2C_CMD_START;
	I2C0->TXDATA = NXP_NTAG_R;
	uint8_t data;

	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	while(I2C0->IF & I2C_IF_RXDATAV)
	{
		data = I2C0->RXDATA;
	}

	I2C0->CMD = I2C_CMD_ACK;
	I2C0->CMD = I2C_CMD_STOP;
	delay();
	return data;
}

/**
 * @brief This function is a polling read driver for NXP NTAG I2C NFC Device.
 * 16 Bytes are read in a single I2C read transfer.
 * @param register_address - Address of the starting register you want to read. Following 15 bytes will be also read.
 * @return None.
 */
uint8_t* i2c_read_poll(uint8_t register_address)
{
	I2C0->CMD = I2C_CMD_START;  //send start bit
	I2C0->TXDATA = 0x04; //slave address.

	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->TXDATA = register_address;
	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	I2C0->CMD = I2C_CMD_STOP;
	delay();

	I2C0->CMD = I2C_CMD_START;
	I2C0->TXDATA = NXP_NTAG_R;

	while((I2C0->IF & I2C_IF_ACK) == 0);
	I2C0->IFC |= I2C_IFC_ACK;

	//uint8_t index = 0;

	while((I2C0->IF & I2C_IF_RXDATAV) == 0);
	read[0] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_ACK;

	while((I2C0->IF & I2C_IF_RXDATAV) == 0);
	read[1] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_ACK;

	while((I2C0->IF & I2C_IF_RXDATAV) == 0);
	read[2] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_ACK;

	while((I2C0->IF & I2C_IF_RXDATAV) == 0);
	read[3] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_ACK;

	while((I2C0->IF & I2C_IF_RXDATAV) == 0);
	read[4] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_ACK;

	while((I2C0->IF & I2C_IF_RXDATAV) == 0);
	read[5] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_ACK;

	while((I2C0->IF & I2C_IF_RXDATAV) == 0);
	read[6] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_ACK;

	while((I2C0->IF & I2C_IF_RXDATAV) == 0);
	read[7] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_ACK;

	while((I2C0->IF & I2C_IF_RXDATAV) == 0);
	read[8] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_ACK;

	while((I2C0->IF & I2C_IF_RXDATAV) == 0);
	read[9] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_ACK;

	while((I2C0->IF & I2C_IF_RXDATAV) == 0);
	read[10] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_ACK;

	while((I2C0->IF & I2C_IF_RXDATAV) == 0);
	read[11] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_ACK;

	while((I2C0->IF & I2C_IF_RXDATAV) == 0);
	read[12] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_ACK;

	while((I2C0->IF & I2C_IF_RXDATAV) == 0);
	read[13] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_ACK;

	while((I2C0->IF & I2C_IF_RXDATAV) == 0);
	read[14] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_ACK;

	while((I2C0->IF & I2C_IF_RXDATAV) == 0);
	read[15] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_ACK;

	I2C0->CMD = I2C_CMD_STOP;
	delay();
	while(I2C0->CMD & I2C_STATUS_PSTOP);
	return read;

}


/**
 * @brief This function is used to disable the I2C peripheral.
 * @param None
 *
 */
void i2cdisable()
{
	I2C0 -> ROUTEPEN &= ~I2C_ROUTEPEN_SCLPEN;
	I2C0 -> ROUTEPEN &=~ I2C_ROUTEPEN_SDAPEN;
	I2C_Enable(I2C0, false);
	GPIO_PinOutClear(gpioPortC, 10); // SCL Before
	GPIO_PinOutClear(gpioPortC, 11);//SDA
	GPIO_PinOutClear(gpioPortD, 15);
}

/**
 * @brief- IRQ Handler for I2C0 Peripheral
 */
void I2C0_IRQHandler()
{
	CORE_ATOMIC_IRQ_DISABLE();
	if(I2C0->IF & I2C_IF_ACK)
	{
		I2C0->IFC |= I2C_IFC_ACK;
		interrupt_flag_ack = 1;
	}
	if(I2C0->IF & I2C_IF_NACK)
	{
		printf("NACK Received\n");
		I2C0->IFC |= I2C_IFC_NACK;
	}
	CORE_ATOMIC_IRQ_ENABLE();
}
/**
 * @brief Delay function used to add additional delays required in I2C Initialization.
 * @param None
 *
 */
void delay(void)
{
	for(int i=0; i<1000000; i++);
}
