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

#include "em_leuart.h"


//Function declarations
void leuart_init(void);
void leuart_send(LEUART_TypeDef *leuart, uint8_t data);
char leuart_rcv(LEUART_TypeDef *leuart);
void leuart_loopback_test_blocking(void);



#endif /* INC_LEUART_H_ */
