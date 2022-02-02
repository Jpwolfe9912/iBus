/*
 * drv_crsf.h
 *
 *  Created on: Jan 14, 2022
 *      Author: jeremywolfe
 */

#ifndef INC_DRV_USART_H_
#define INC_DRV_USART_H_

/* Define the Size Here */
#define RXBUF_SIZE 					32


/*		Function Prototypes	*/
void usart_init(void);

/*		Global Variables	*/
extern uint8_t rxBuf[RXBUF_SIZE];
extern lwrb_t rxRingBuf;
extern uint8_t rxRingBufData[RXBUF_SIZE];

#endif /* INC_DRV_USART_H_ */



