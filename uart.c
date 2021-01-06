#include "stm32f0xx.h"
#include "uart.h"
#include <stdio.h>

#define USART_BUFSIZE 256

#ifdef UART1_ISR
uint16_t uart1_index;
uint8_t uart1_Rx_buffer[USART_BUFSIZE];  //for receiving data in isr
#endif

#ifdef UART2_ISR
extern uint16_t uart2_index;
extern uint8_t uart2_buffer[USART_BUFSIZE];
#endif

void uart_init(void) {

	USART_InitTypeDef USART_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
#ifdef UART1

	RCC_AHBPeriphClockCmd(UART1_GPIO_CLOCK, ENABLE); //UART GPIO pin
	RCC_APB2PeriphClockCmd(USART1_CLOCK, ENABLE);

	GPIO_InitStruct.GPIO_Pin = UART1_RXD | UART1_TXD;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(UART1_GPIO_PORT, &GPIO_InitStruct);

	GPIO_PinAFConfig(UART1_GPIO_PORT, UART1_TXD_PIN, GPIO_AF_1);  	//uart1 txd
	GPIO_PinAFConfig(UART1_GPIO_PORT, UART1_RXD_PIN, GPIO_AF_1); 	//uart1 rxd

	USART_InitStruct.USART_BaudRate = USART1_BAUDRATE;
	USART_InitStruct.USART_WordLength = USART1_WORD_LENGTH;
	USART_InitStruct.USART_StopBits = USART1_STOP_BITS;
	USART_InitStruct.USART_Parity = USART1_PARITY;
	USART_InitStruct.USART_Mode = USART1_MODE;
	USART_InitStruct.USART_HardwareFlowControl = USART1_HARDWARE_FLOW;

	USART_Init(USART1, &USART_InitStruct);

#ifdef UART1_ISR
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART1_IRQn);
#endif

	USART_Cmd(USART1, ENABLE);
#endif

#ifdef UART2
	RCC_AHBPeriphClockCmd(UART2_GPIO_CLOCK, ENABLE); //UART GPIO pin
	RCC_APB1PeriphClockCmd(USART2_CLOCK, ENABLE);

	GPIO_InitStruct.GPIO_Pin = UART2_RXD | UART2_TXD;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(UART2_GPIO_PORT, &GPIO_InitStruct);

	GPIO_PinAFConfig(UART2_GPIO_PORT, UART2_TXD_PIN, GPIO_AF_1);//uart2 rxd
	GPIO_PinAFConfig(UART2_GPIO_PORT, UART2_RXD_PIN, GPIO_AF_1);//uart2 txd

	USART_InitStruct.USART_BaudRate = USART2_BAUDRATE;
	USART_InitStruct.USART_WordLength = USART2_WORD_LENGTH;
	USART_InitStruct.USART_StopBits = USART2_STOP_BITS;
	USART_InitStruct.USART_Parity = USART2_PARITY;
	USART_InitStruct.USART_Mode = USART2_MODE;
	USART_InitStruct.USART_HardwareFlowControl = USART2_HARDWARE_FLOW;

	USART_Init(USART2, &USART_InitStruct);

	USART_SWAPPinCmd(USART2, UART2_SWAP);

#ifdef UART2_ISR
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART2_IRQn);
#endif

	USART_Cmd(USART2, ENABLE);
#endif
}
#ifdef UART1
void uart1_string(char *data) {
	while (*data) {
		USART_SendData(USART1, *data++);
		delay_ms(2);
	}
}

void uart1_SendData(uint8_t data) {
	USART_SendData(USART1, data);
	delay_ms(2);
}

#endif

#ifdef UART2
void uart2_TxString(uint8_t *data) {
	while (*data) {
		USART_SendData(USART2, *data++);
		while ((USART2->ISR & USART_FLAG_TC) == (uint16_t) RESET)
		;
	}
}
#endif

#ifdef UART1_ISR
void USART1_IRQHandler(void) {
	__disable_irq();
	uint32_t isr_flags = USART1->ISR;
	uint32_t error_flags;
	uint8_t dummy = dummy;

	error_flags = isr_flags
			& (uint32_t) (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE
					| USART_ISR_NE);

	if (error_flags != RESET) {				//If any error occour in frame

		if (((isr_flags & USART_ISR_FE) != RESET)) {
			USART_ClearFlag(USART1, USART_FLAG_FE);
		} else if (((isr_flags & USART_ISR_PE) != RESET)) {
			USART_ClearFlag(USART1, USART_FLAG_PE);
		} else if (((isr_flags & USART_ISR_ORE) != RESET)) {
			USART_ClearFlag(USART1, USART_FLAG_ORE);
		} else if (((isr_flags & USART_ISR_NE) != RESET)) {
			USART_ClearFlag(USART1, USART_FLAG_NE);
		}
		dummy = (uint8_t) USART_ReceiveData(USART1); //TO clr error dummy read needed
	} else {		//if no error
		if (((isr_flags & USART_ISR_RXNE) != RESET)) {
			uart1_Rx_buffer[uart1_index++] = (uint8_t) USART_ReceiveData(
			USART1);
			uart1_SendData(uart1_Rx_buffer[uart1_index - 1]);
		}
	}
	__enable_irq();
}
#endif

#ifdef UART2_ISR
void USART2_IRQHandler(void) {
	__disable_irq();
	uint32_t isr_flags = USART2->ISR;
	uint32_t error_flags;
	uint8_t dummy = dummy;

	error_flags = isr_flags
	& (uint32_t) (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE
			| USART_ISR_NE);

	if (error_flags != RESET) {			//If any error occour in frame

		if (((isr_flags & USART_ISR_FE) != RESET)) {
			USART_ClearFlag(USART2, USART_FLAG_FE);
		} else if (((isr_flags & USART_ISR_PE) != RESET)) {
			USART_ClearFlag(USART2, USART_FLAG_PE);
		} else if (((isr_flags & USART_ISR_ORE) != RESET)) {
			USART_ClearFlag(USART2, USART_FLAG_ORE);
		} else if (((isr_flags & USART_ISR_NE) != RESET)) {
			USART_ClearFlag(USART2, USART_FLAG_NE);
		}
		dummy = (uint8_t) USART_ReceiveData(USART2); //TO clr error dummy read needed
	} else {		//if no error
		if (((isr_flags & USART_ISR_RXNE) != RESET)) {
			uart2_buffer[uart2_index++] = (uint8_t) USART_ReceiveData(
					USART2);
		}
	}
	__enable_irq();
}
#endif

#if 0 // mam isr
/* USART1 IRQ Handler*/
void USART2_IRQHandler(void) {
	// disable irq to make section atomic
	__disable_irq();
	uint32_t isr_flags = USART2->ISR;
	uint32_t cr1_its = USART2->CR1;

	uint32_t error_flags;
	uint8_t dummy = dummy;
	/* If no error occurs */
	error_flags = (isr_flags
			& (uint32_t) (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE
					| USART_ISR_NE));
	if (error_flags == RESET) {
		if (((isr_flags & USART_ISR_RXNE) != RESET)
				&& ((cr1_its & USART_CR1_RXNEIE) != RESET)) /* Receive Data Ready */
		{
			/* If no error on ISR, normal data ready, save into the data buffer. */
			/* Note: read from RDR will clear the interrupt */
			uart2_buffer[uart2_index] = (uint8_t) USART_ReceiveData(USART2);
			uart2_index++;
			if (uart2_index == USART_BUFSIZE) {
				uart2_index = 0; /* buffer overflow */
			}
		}
	} else {
		//cr3_its = MODBUS_USART->CR3;
		/* USART frame error interrupt occurred --------------------------------------*/
		if (((isr_flags & USART_ISR_FE) != RESET) /*&& ((cr3_its & USART_CR3_EIE) != RESET)*/) {
			/*Clear the Frame error Flag*/
			USART_ClearFlag(USART2, USART_FLAG_FE);
		}
		/* USART noise error interrupt occurred --------------------------------------*/
		else if (((isr_flags & USART_ISR_NE) != RESET) /*&& ((cr3_its & USART_CR3_EIE) != RESET)*/) {
			/*Clear the noise error Flag*/
			USART_ClearFlag(USART2, USART_FLAG_NE);
		}
		/* USART Over-Run interrupt occurred -----------------------------------------*/
		else if (((isr_flags & USART_ISR_ORE) != RESET)/* &&
				 (((cr1_its & USART_CR1_RXNEIE) != RESET) || ((cr3_its & USART_CR3_EIE) != RESET))*/) {
			/*Clear the Over-Run error Flag*/
			USART_ClearFlag(USART2, USART_FLAG_ORE);
		}
		dummy = USART_ReceiveData(USART2);
	}
	// re-enable irqs
	__enable_irq();
}
#endif
