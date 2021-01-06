/*
 * nrf24l01.h
 *
 *  Created on: 12-Jun-2017
 *      Author: devsys
 */

#ifndef NRF24L01_H_
#define NRF24L01_H_

#include <stm32f0xx.h>
#include <string.h>
#include "Hardware_Abstraction.h"

//********************* SPI RELATED MACROS ************************//
#define NRF_SPI_PORT_MOSI_SCK	GPIOA
#define NRF_SPI_PORT_MISO	GPIOB
#define NRF_SPI_PORT_SS		GPIOB
#define NRF_SPI_PORT_CE		GPIOC
#define NRF_IRQ_PORT		GPIOC
#define NRF_SPI_SCK_PIN 	GPIO_Pin_5
#define NRF_SPI_MISO_PIN 	GPIO_Pin_4
#define NRF_SPI_MOSI_PIN 	GPIO_Pin_7
#define NRF_SPI_SS_PIN 		GPIO_Pin_3
#define NRF_CE_PIN			GPIO_Pin_11
#define NRF_IRQ_PIN			GPIO_Pin_10
#define NRF_SPI_PINSOURCE_SCK_MOSI	GPIO_PinSource7|GPIO_PinSource5
#define NRF_SPI_PINSOURCE_MISO	GPIO_PinSource4
#define NRF_GPIO_RCC_MOSI_SCK		RCC_AHBPeriph_GPIOA
#define NRF_GPIO_RCC_MISO_SS		RCC_AHBPeriph_GPIOB
#define NRF_GPIO_RCC_CE		RCC_AHBPeriph_GPIOC
#define NRF_IRQ_RCC			RCC_AHBPeriph_GPIOC
#define CSN_HI			__CSN_HIGH()
#define CSN_LOW				__CSN_LOW()
#define CE_HI				__CE_HIGH()
#define CE_LOW				__CE_LOW()

#define CSN_TIME      2

//***************************** NRF24l01+ SPI command macros************************//
#define R_REGISTER 0x00
#define W_REGISTER 0x20
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define R_RX_PL_WID 0x60
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define NOP 0xff

//***************************** NRF24l01+ register value macros************************//
#define CONFIG      0x00
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0

#define EN_AA       0x01
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0

#define EN_RXADDR   0x02
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0

#define SETUP_AW    0x03
#define AW          0

#define SETUP_RETR  0x04
#define ARD         4
#define ARC         0

#define RF_CH       0x05

#define RF_SETUP    0x06
#define CONT_WAVE   7
#define RF_DR_LOW   5
#define PLL_LOCK    4
#define RF_DR_HIGH  3
#define RF_PWR      1

#define STATUS              0x07
#define RX_DR               6
#define TX_DS               5
#define MAX_RT              4
#define RX_P_NO_MASK        0x0E
#define STATUS_TX_FULL      0

#define OBSERVE_TX  0x08
#define PLOS_CNT    4
#define ARC_CNT     0

#define RPD         0x09

#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F

#define TX_ADDR     0x10

#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16

#define FIFO_STATUS         0x17
#define TX_REUSE            6
#define FIFO_TX_FULL        5
#define TX_EMPTY            4
#define RX_FULL             1
#define RX_EMPTY            0

#define DYNPD       0x1C
#define DPL_P5      5
#define DPL_P4      4
#define DPL_P3      3
#define DPL_P2      2
#define DPL_P1      1
#define DPL_P0      0

#define FEATURE     0x1D
#define EN_DPL      2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

#define ADD5b 3
#define PIPE0 0

typedef enum {
	RETRY_FAIL = 200, EMPTY_ACK, ACK_PAYLOAD, RF_ERROR, WAIT_OVER
} Tx_State;

//********************other macros****************************//
#define _LS(x) (1 << x)

//*********************nrf24l01 SPI functions**********************//
void __CSN_HIGH(void);
void __CSN_LOW(void);
void __CE_HIGH(void);
void __CE_LOW(void);
void nrf_Enable_PWR(void);
void spi_Rx_Init(void);
uint8_t spi_Send(uint8_t byte);
void nrf_Irq_Init(void);

//******************** nrf24l01 generic functions***************//
//void nrf24l01_start_delay(void);
void nrf24l01_tx_Init(void);
void nrf24l01_rx_Init(void);
void spi_nRF2401_Init(void);
void nrf24l01_load_default(void);
void nrf24l01_sendcmd(uint8_t cmd);
uint8_t nrf24l01_getStatus(void);
int nrf24l01_readregister(uint8_t command, uint8_t *rxbuf, int numbytes);
int nrf24l01_writeregister(uint8_t command, uint8_t *data, int numbytes);
int nrf24l01_writePayload(uint8_t *data, int numbytes);
int nrf24l01_readPayload(uint8_t *data, int numbytes);
void nrf24l01_clear_interrupts(void);
//******************** nrf24l01 transmitter functions***************//
void nrf24l01_clear_transmit_interrupts(void);
void nRF24L01_transmit(uint8_t *to_addr, uint8_t *msg, int msglen);
uint8_t nRF24L01_check_transmit_success(void);
void nRF24L01_flush_transmit_message(void);
int nRF24L01_check_Tx_FIFO_Empty(void);
int check_Tx_FIFO_FULL(void);
//******************** nrf24l01 receiver functions***************//
void nrf24l01_clear_receive_interrupts(void);
void nRF24L01_flush_receive_message(void);
void nRF24L01_listen(int pipe, uint8_t *address);
int nRF24L01_check_data_received(void);
int nRF24L01_get_received_pipe_number(void);
int nRF24L01_read_received_data(uint8_t *receive_msg);
int nRF24L01_check_Rx_FIFO_Empty(void);
int nRF24L01_check_Rx_FIFO_Full(void);
uint8_t nRF24L01_rx_payload_size(void);
int nRF24L01_Write_Ack_Payload(uint8_t pipe, uint8_t *data, int datalen);

#endif /* NRF24L01_H_ */
