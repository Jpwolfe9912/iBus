/*
 * drv_crsf.h
 *
 *  Created on: Jan 14, 2022
 *      Author: jeremywolfe
 */

#ifndef INC_DRV_IBUS_H_
#define INC_DRV_IBUS_H_

#define IBUS_FRAME_SIZE_MAX			32

/* Define the Size Here */
#define PAYLOAD_SIZE				IBUS_FRAME_SIZE_MAX - 4U
#define RC_CHANNELS					14

#define ARRAY_LEN(x)				(sizeof(x) / sizeof((x)[0]))

void usart_rx_check(uint32_t bytesLeft);
void usart_process_data(const void* data, size_t len);
void ibus_process(void);
bool ibus_process_frame(void);

/*		Global Variables	*/
extern uint8_t rxBuf[RXBUF_SIZE];

extern lwrb_t rxRingBuf;
extern uint8_t rxRingBufData[RXBUF_SIZE];

extern uint8_t ibusPayload[PAYLOAD_SIZE];
extern uint16_t ibusRcChannels[RC_CHANNELS];

enum {
	IBUS_STATE_LENGTH,
	IBUS_STATE_TYPE,
	IBUS_STATE_PAYLOAD,
	IBUS_STATE_CRC
};

#endif /* INC_DRV_IBUS_H_ */
