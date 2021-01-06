/*
 * uart.h
 *
 *  Created on: Nov 25, 2020
 *      Author: Arjun
 */

#ifndef UART_H_
#define UART_H_

#define UART1
#define UART1_ISR
//#define UART2
//#define UART2_ISR

/********************************USART1***************************/
/*---------UART1 CLOCK Seting's--------------*/
#define USART1_CLOCK RCC_APB2Periph_USART1
#define UART1_GPIO_CLOCK RCC_AHBPeriph_GPIOA

/*---------UART1 GPIO Setting's---------------*/
#define UART1_TXD GPIO_Pin_9
#define UART1_RXD GPIO_Pin_10
#define UART1_TXD_PIN GPIO_PinSource9
#define UART1_RXD_PIN GPIO_PinSource10
#define UART1_GPIO_PORT GPIOA

/*---------UART1 Setting's---------------*/
#define USART1_BAUDRATE  9600
#define USART1_WORD_LENGTH  USART_WordLength_8b
#define USART1_STOP_BITS  USART_StopBits_1
#define USART1_PARITY  USART_Parity_No
#define USART1_MODE  (USART_Mode_Rx | USART_Mode_Tx)
#define USART1_HARDWARE_FLOW  USART_HardwareFlowControl_None

/********************************USART2***************************/
/*---------UART2 CLOCK Seting's--------------*/
#define USART2_CLOCK RCC_APB1Periph_USART2
#define UART2_GPIO_CLOCK RCC_AHBPeriph_GPIOA

/*---------UART2 GPIO Setting's---------------*/
#define UART2_TXD GPIO_Pin_15
#define UART2_RXD GPIO_Pin_2
#define UART2_TXD_PIN GPIO_PinSource15
#define UART2_RXD_PIN GPIO_PinSource2
#define UART2_GPIO_PORT GPIOA

/*---------UART2 Setting's---------------*/
#define USART2_BAUDRATE  9600
#define USART2_WORD_LENGTH  USART_WordLength_8b
#define USART2_STOP_BITS  USART_StopBits_1
#define USART2_PARITY  USART_Parity_No
#define USART2_MODE  USART_Mode_Rx | USART_Mode_Tx
#define USART2_HARDWARE_FLOW  USART_HardwareFlowControl_None
#define UART2_SWAP ENABLE

extern void uart_init(void);
extern void uart1_string(char *data);
extern void uart2_TxString(uint8_t *data);
extern void uart1_SendData(uint8_t data);

#endif /* UART_H_ */
