/*
 * crsf.c
 *
 *  Created on: Jan 14, 2022
 *      Author: jeremywolfe
 */
#include "board.h"

/*		Static Function Prototypes	*/
static bool usart_read(uint8_t *pData, uint8_t size);

/*		Global Variables	*/
uint8_t rxBuf[RXBUF_SIZE];			// dma data
lwrb_t rxRingBuf;					// ring buffer instance
uint8_t rxRingBufData[RXBUF_SIZE];	// ring buffer data

#ifdef USE_USART1

void usart_init(void){
	/* ---PIN INFO---
	 * USART1_RX
	 * 		PA10
	 * 		AF7
	 * 		DMA 2
	 * 		Stream 2
	 * 		Channel 4
	 * USART1_TX
	 * 		PA9
	 * 		AF7
	 * 		DMA 2
	 * 		Stream 7
	 * 		Channel 4
	 * */

	/////////////////GPIO INIT///////////////////
	// enable clock for GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	// set mode, speed, type, pull, AF
	GPIOA->MODER 	&= ~GPIO_MODER_MODER9;
	GPIOA->MODER 	|= GPIO_MODER_MODER9_1;				// AF mode
	GPIOA->OSPEEDR	|= GPIO_OSPEEDR_OSPEEDR9;			// high speed
	GPIOA->OTYPER	&= ~GPIO_OTYPER_OT9;
	GPIOA->PUPDR	&= ~GPIO_PUPDR_PUPDR9;
	GPIOA->AFR[1]	&= ~GPIO_AFRH_AFRH1;
	GPIOA->AFR[1] 	|= (0x7 << (4U * 1U));				// AF 7

	GPIOA->MODER 	&= ~GPIO_MODER_MODER10;
	GPIOA->MODER 	|= GPIO_MODER_MODER10_1;
	GPIOA->OSPEEDR	|= GPIO_OSPEEDR_OSPEEDR10;
	GPIOA->OTYPER	&= ~GPIO_OTYPER_OT10;
	GPIOA->PUPDR	&= ~GPIO_PUPDR_PUPDR10;
	GPIOA->AFR[1]	&= ~GPIO_AFRH_AFRH2;
	GPIOA->AFR[1] 	|= (0x7 << (4U * 2U));

	// DMA IRQ Init
	NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(DMA2_Stream2_IRQn);

	NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(USART1_IRQn);

	/////////////////USART INIT///////////////////
	RCC->APB2ENR	|= RCC_APB2ENR_USART1EN;

	USART1->CR1		&= ~USART_CR1_UE;		// disable usart
	USART1->BRR		= 0x101;				// 115200 BR
	USART1->CR1		&= ~USART_CR1_M;		// 8 bit transfer
	USART1->CR2		&= ~USART_CR2_STOP;
	USART1->CR1		&= ~USART_CR1_PCE;
	USART1->CR1		|= USART_CR1_RE	|		// enable rx, tx
			USART_CR1_TE;
	USART1->CR3		&= ~(USART_CR3_CTSE |
			USART_CR3_RTSE);
	USART1->CR1		&= ~USART_CR1_OVER8;

	/////////////////DMA INIT///////////////////
	RCC->AHB1RSTR	|= RCC_AHB1RSTR_DMA2RST;
	RCC->AHB1RSTR	&= ~RCC_AHB1RSTR_DMA2RST;

	// disable DMA 1 stream 1
	DMA2_Stream2->CR 	&= ~DMA_SxCR_EN;
	while(DMA2_Stream2->CR & DMA_SxCR_EN){}
	DMA2_Stream2->CR	= 0;
	DMA2_Stream2->NDTR	= 0;
	DMA2_Stream2->PAR	= 0;
	DMA2_Stream2->M0AR	= 0;
	DMA2_Stream2->M1AR	= 0;
	DMA2_Stream2->FCR	= 0x00000021U;
	DMA2_Stream2->CR	&= ~DMA_SxCR_CHSEL;
	DMA2->LIFCR			|= (0x3F << 16U); //0x00000F40U;

	RCC->AHB1ENR		|= RCC_AHB1ENR_DMA2EN;

	// stream 1 ch 4 DMA settings
	DMA2_Stream2->CR	|= (0x4 << 25U);			// channel 4
	DMA2_Stream2->M0AR 	= (uint32_t)rxBuf;			// set mem address
	DMA2_Stream2->CR 	&= ~DMA_SxCR_DIR;			// per to mem
	DMA2_Stream2->FCR	&= ~DMA_SxFCR_DMDIS;		// fifo dis
	DMA2_Stream2->CR 	&= ~DMA_SxCR_MBURST;
	DMA2_Stream2->CR 	&= ~DMA_SxCR_PBURST;
	DMA2_Stream2->PAR 	= (uint32_t)(&(USART1->RDR));// set per address
	DMA2_Stream2->NDTR	= RXBUF_SIZE;		// 64 bytes
	DMA2_Stream2->CR 	&= ~DMA_SxCR_PINC;			// don't inc per
	DMA2_Stream2->CR 	|= DMA_SxCR_MINC;			// increment mem
	DMA2_Stream2->CR 	&= ~DMA_SxCR_MSIZE;			// 8 bit size
	DMA2_Stream2->CR 	&= ~DMA_SxCR_PSIZE;			// 8 bit size
	DMA2_Stream2->CR 	|= DMA_SxCR_CIRC;			// circ mode en
	DMA2_Stream2->CR 	&= ~DMA_SxCR_PL_0;			// medium priority

	USART1->CR1			|= USART_CR1_UE;			// enable usart

	lwrb_init(&rxRingBuf, rxRingBufData, sizeof(rxRingBufData));

	usart_read(rxBuf, RXBUF_SIZE);


}

static bool usart_read(uint8_t *pData, uint8_t size){
	if(!(USART1->ISR & USART_ISR_BUSY)){		// wait for UART to be ready
		DMA2_Stream2->CR	&= ~DMA_SxCR_EN;	// disable DMA
		while(DMA2_Stream2->CR & DMA_SxCR_EN);
		DMA2_Stream2->CR	|= (0x4 << 25U);	// set DMA channel
		DMA2_Stream2->NDTR	= size;				// set transfer size
		DMA2_Stream2->M0AR	= (uint32_t)pData;	// set memory address

		DMA2->LIFCR			|= (0x3F << 16U);	// clear flags

		DMA2_Stream2->CR 	|= DMA_SxCR_TCIE;	// set transfer complete interrupts
		DMA2_Stream2->CR	|= DMA_SxCR_HTIE;

		DMA2_Stream2->CR	|= DMA_SxCR_EN;		// enable DMA

		USART1->CR3			|= USART_CR3_DMAR;	// enable DMA for UART

		USART1->ICR			|= USART_ICR_IDLECF;// clear idle interrupt flag
		USART1->CR1			|= USART_CR1_IDLEIE;// enable idle line interrupts

		return true;
	}
	else return false;
}


/* Interrupt handlers here */

/**
 * \brief           DMA2 stream1 interrupt handler for USART1 RX
 */
void DMA2_Stream2_IRQHandler(void) {
	/* Check half-transfer complete interrupt */
	if(DMA2->LISR & DMA_LISR_TCIF2){
		DMA2->LIFCR		|= DMA_LIFCR_CTCIF2;	/* Clear half-transfer complete flag */
		usart_rx_check(DMA2_Stream2->NDTR);                       /* Check for data to process */
	}

	/* Check transfer-complete interrupt */
	if(DMA2->LISR & DMA_LISR_HTIF2){
		DMA2->LIFCR		|= DMA_LIFCR_CHTIF2;	/* Clear half-transfer complete flag */
		usart_rx_check(DMA2_Stream2->NDTR);                       /* Check for data to process */
	}

	/* Implement other events when needed */
}

/**
 * \brief           USART1 global interrupt handler
 */
void USART1_IRQHandler(void) {
	/* Check for IDLE line interrupt */
	if (USART1->ISR & USART_ISR_IDLE) {
		USART1->ICR		|= USART_ICR_IDLECF;	/* Clear IDLE line flag */
		usart_rx_check(DMA2_Stream2->NDTR);                       /* Check for data to process */
	}

	/* Implement other events when needed */
}


#endif

#ifdef USE_USART3

#ifdef USE_USART1
#undef USE_USART1
#endif

void usart_init(void){
	/* ---PIN INFO---
	 * USART3_RX
	 * 		PD9
	 * 		AF7
	 * 		DMA 1
	 * 		Stream 1
	 * 		Channel 4
	 * USART3_RX
	 * 		PD8
	 * 		AF7
	 * 		DMA 1
	 * 		Stream 3
	 * 		Channel 4
	 * */

	/////////////////GPIO INIT///////////////////
	// enable clock for GPIOD
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	// set mode, speed, type, pull, AF
	GPIOD->MODER 	&= ~GPIO_MODER_MODER8;
	GPIOD->MODER 	|= GPIO_MODER_MODER8_1;				// AF mode
	GPIOD->OSPEEDR	|= GPIO_OSPEEDR_OSPEEDR8;			// high speed
	GPIOD->OTYPER	&= ~GPIO_OTYPER_OT8;
	GPIOD->PUPDR	&= ~GPIO_PUPDR_PUPDR8;
	GPIOD->AFR[1]	&= ~GPIO_AFRH_AFRH0;
	GPIOD->AFR[1] 	|= (0x7 << (4U * 0U));				// AF 7

	GPIOD->MODER 	&= ~GPIO_MODER_MODER9;
	GPIOD->MODER 	|= GPIO_MODER_MODER9_1;
	GPIOD->OSPEEDR	|= GPIO_OSPEEDR_OSPEEDR9;
	GPIOD->OTYPER	&= ~GPIO_OTYPER_OT9;
	GPIOD->PUPDR	&= ~GPIO_PUPDR_PUPDR9;
	GPIOD->AFR[1]	&= ~GPIO_AFRH_AFRH1;
	GPIOD->AFR[1] 	|= (0x7 << (4U * 1U));

	// DMA IRQ Init
	NVIC_SetPriority(DMA1_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(DMA1_Stream1_IRQn);

	NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(USART3_IRQn);

	/////////////////USART INIT///////////////////
	RCC->APB1ENR	|= RCC_APB1ENR_USART3EN;

	USART3->CR1		&= ~USART_CR1_UE;		// disable usart
	USART3->BRR		= 0x1D5;				// 420000 BR
	USART3->CR1		&= ~USART_CR1_M;		// 8 bit transfer
	USART3->CR2		&= ~USART_CR2_STOP;
	USART3->CR1		&= ~USART_CR1_PCE;
	USART3->CR1		|= USART_CR1_RE	|		// enable rx, tx
					   USART_CR1_TE;
	USART3->CR3		&= ~(USART_CR3_CTSE |
						 USART_CR3_RTSE);
	USART3->CR1		&= ~USART_CR1_OVER8;

	/////////////////DMA INIT///////////////////
	RCC->AHB1RSTR	|= RCC_AHB1RSTR_DMA1RST;
	RCC->AHB1RSTR	&= ~RCC_AHB1RSTR_DMA1RST;

	// disable DMA 1 stream 1
	DMA1_Stream1->CR 	&= ~DMA_SxCR_EN;
	while(DMA1_Stream1->CR & DMA_SxCR_EN){}
	DMA1_Stream1->CR	= 0;
	DMA1_Stream1->NDTR	= 0;
	DMA1_Stream1->PAR	= 0;
	DMA1_Stream1->M0AR	= 0;
	DMA1_Stream1->M1AR	= 0;
	DMA1_Stream1->FCR	= 0x00000021U;
	DMA1_Stream1->CR	&= ~DMA_SxCR_CHSEL;
	DMA1->LIFCR			|= (0x3F << 6U); //0x00000F40U;

	RCC->AHB1ENR		|= RCC_AHB1ENR_DMA1EN;

	// stream 1 ch 4 DMA settings
	DMA1_Stream1->CR	|= (0x4 << 25U);			// channel 4
	DMA1_Stream1->M0AR 	= (uint32_t)rxBuf;			// set mem address
	DMA1_Stream1->CR 	&= ~DMA_SxCR_DIR;			// per to mem
	DMA1_Stream1->FCR	&= ~DMA_SxFCR_DMDIS;		// fifo dis
	DMA1_Stream1->CR 	&= ~DMA_SxCR_MBURST;
	DMA1_Stream1->CR 	&= ~DMA_SxCR_PBURST;
	DMA1_Stream1->PAR 	= (uint32_t)(&(USART3->RDR));// set per address
	DMA1_Stream1->NDTR	= RXBUF_SIZE;		// 64 bytes
	DMA1_Stream1->CR 	&= ~DMA_SxCR_PINC;			// don't inc per
	DMA1_Stream1->CR 	|= DMA_SxCR_MINC;			// increment mem
	DMA1_Stream1->CR 	&= ~DMA_SxCR_MSIZE;			// 8 bit size
	DMA1_Stream1->CR 	&= ~DMA_SxCR_PSIZE;			// 8 bit size
	DMA1_Stream1->CR 	|= DMA_SxCR_CIRC;			// circ mode en
	DMA1_Stream1->CR 	&= ~DMA_SxCR_PL_0;			// medium priority

	USART3->CR1			|= USART_CR1_UE;			// enable usart

	lwrb_init(&rxRingBuf, rxRingBufData, sizeof(rxRingBufData));

	usart_read(rxBuf, RXBUF_SIZE);


}

static bool usart_read(uint8_t *pData, uint8_t size){
	if(!(USART3->ISR & USART_ISR_BUSY)){		// wait for UART to be ready
		DMA1_Stream1->CR	&= ~DMA_SxCR_EN;	// disable DMA
		while(DMA1_Stream1->CR & DMA_SxCR_EN);
		DMA1_Stream1->CR	|= (0x4 << 25U);	// set DMA channel
		DMA1_Stream1->NDTR	= size;				// set transfer size
		DMA1_Stream1->M0AR	= (uint32_t)pData;	// set memory address

		DMA1->LIFCR			|= (0x3F << 6U);	// clear flags

		DMA1_Stream1->CR 	|= DMA_SxCR_TCIE;	// set transfer complete interrupts
		DMA1_Stream1->CR	|= DMA_SxCR_HTIE;

		DMA1_Stream1->CR	|= DMA_SxCR_EN;		// enable DMA

		USART3->CR3			|= USART_CR3_DMAR;	// enable DMA for UART

		USART3->ICR			|= USART_ICR_IDLECF;// clear idle interrupt flag
		USART3->CR1			|= USART_CR1_IDLEIE;// enable idle line interrupts

		return true;
	}
	else return false;
}



/* Interrupt handlers here */

/**
 * \brief           DMA1 stream1 interrupt handler for USART3 RX
 */
void DMA1_Stream1_IRQHandler(void) {
    /* Check half-transfer complete interrupt */
	if(DMA1->LISR & DMA_LISR_TCIF1){
		DMA1->LIFCR		|= DMA_LIFCR_CTCIF1;	/* Clear half-transfer complete flag */
        usart_rx_check(DMA1_Stream1->NDTR);                       /* Check for data to process */
    }

    /* Check transfer-complete interrupt */
	if(DMA1->LISR & DMA_LISR_HTIF1){
		DMA1->LIFCR		|= DMA_LIFCR_CHTIF1;	/* Clear half-transfer complete flag */
        usart_rx_check(DMA1_Stream1->NDTR);                       /* Check for data to process */
    }

    /* Implement other events when needed */
}

/**
 * \brief           USART3 global interrupt handler
 */
void USART3_IRQHandler(void) {
    /* Check for IDLE line interrupt */
    if (USART3->ISR & USART_ISR_IDLE) {
    	USART3->ICR		|= USART_ICR_IDLECF;	/* Clear IDLE line flag */
        usart_rx_check(DMA1_Stream1->NDTR);                       /* Check for data to process */
    }

    /* Implement other events when needed */
}
#endif
