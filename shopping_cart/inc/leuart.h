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



//Function declarations
void gpio_init(void);
void leuart_init(void);
void send(LEUART_TypeDef *leuart, uint8_t data);
char rcv(LEUART_TypeDef *leuart);
void loopback_test_blocking(void);
void barcode_test_blocking(void);


#endif /* INC_LEUART_H_ */
