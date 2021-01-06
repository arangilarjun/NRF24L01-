/*
 * main.c Full duplex
 *
 *  Created on: 17-Dec-2020
 *      Author: Arjun Arangil
 */
#include <stm32f0xx.h>
#include "nrf24l01.h"
#include "uart.h"

uint8_t sent_size, receive;
extern uint8_t uart1_Rx_buffer[];
extern uint16_t uart1_index;

int main(void) {
	uint8_t to_addr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 }; // Pipe address (any address)
	uint8_t rxbuf[5] = { 0 };
	uint8_t iter,status;

	uart_init();
	delay_init();
	nrf24l01_rx_Init();
	nrf24l01_clear_receive_interrupts();
	nrf_Irq_Init(); //if send or rxd data the nrf IRQ pin pulls down

	nRF24L01_listen(PIPE0, to_addr); // Rx mode
	uart1_string("******* Messenger ********\r\n");
	while (1) {
		/*------------------RXD--------------------------*/
		if (receive == 1) { // interrupt

			nRF24L01_read_received_data(rxbuf); //reading nrf rx buffer
			for (iter = 0; iter < sent_size; iter++) {
				uart1_SendData(rxbuf[iter]); //sending data to serial terminal
			}
			uart1_string("\r\n                          Client\r\n");
			receive = uart1_index = 0; //reset value
		}
		/*------------------TXD--------------------------*/
		/*if any data received through uart*/
		if (uart1_Rx_buffer[(uart1_index) - 1] == '\r') {

			nrf24l01_tx_Init(); //TXD init

			resend: if (nRF24L01_check_Tx_FIFO_Empty()) {

				/*transmit data via nRF*/
				nRF24L01_transmit(to_addr, uart1_Rx_buffer, uart1_index);
				delay_ms(10);
				status = nRF24L01_check_transmit_success();

//				if (!receive) {//if data not send successfully
				if (status == RETRY_FAIL) {
					CE_LOW;
					nRF24L01_flush_receive_message();
					nRF24L01_flush_transmit_message();
					nrf24l01_clear_interrupts();
					goto resend;
					//resend
				}
			}
			uart1_index = receive = 0; //reset values
			uart1_string("\r\n                          Server\r\n");
			nrf24l01_rx_Init(); //rx init
			nRF24L01_listen(PIPE0, to_addr); //listen mode
		}
	}
}
