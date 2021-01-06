/*
 * nrf24l01.c
 *
 *  Created on: 19-12-2020
 *      Author: devsys
 */

#include "nrf24l01.h"
#include "rf_protocol.h"

// Chip Select -> high
void __CSN_HIGH(void) {
	GPIO_WriteBit(NRF_SPI_PORT_SS, NRF_SPI_SS_PIN, Bit_SET);
}
// Chip Select -> low
void __CSN_LOW(void) {
	GPIO_WriteBit(NRF_SPI_PORT_SS, NRF_SPI_SS_PIN, Bit_RESET);
}

// Chip enable High
void __CE_HIGH(void) {
	GPIO_WriteBit(NRF_SPI_PORT_CE, NRF_CE_PIN, Bit_SET);
}
// Chip enable low
void __CE_LOW(void) {
	GPIO_WriteBit(NRF_SPI_PORT_CE, NRF_CE_PIN, Bit_RESET);
}

void nrf_Enable_PWR(void) {
	RCC_AHBPeriphClockCmd(NRF_CLOCK_PERIPHERAL, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = NRF_PWR_CONTROL_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_Init(NRF_PWR_CONTROL_PORT, &GPIO_InitStruct);

	GPIO_WriteBit(NRF_PWR_CONTROL_PORT, NRF_PWR_CONTROL_PIN, Bit_SET);
}

void spi_Rx_Init(void) {
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable the SPI periph */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Enable GPIOs clock */
	RCC_AHBPeriphClockCmd(NRF_GPIO_RCC_MOSI_SCK, ENABLE);
	RCC_AHBPeriphClockCmd(NRF_GPIO_RCC_MISO_SS, ENABLE);
	RCC_AHBPeriphClockCmd(NRF_GPIO_RCC_CE, ENABLE);

	// chip select  and chip enable --- output mode
	GPIO_InitStructure.GPIO_Pin = NRF_SPI_SCK_PIN | NRF_SPI_MOSI_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(NRF_SPI_PORT_MOSI_SCK, &GPIO_InitStructure);

	/* Configure SPI pins */
	GPIO_InitStructure.GPIO_Pin = NRF_SPI_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(NRF_SPI_PORT_MISO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = NRF_SPI_SS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(NRF_SPI_PORT_SS, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = NRF_CE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(NRF_SPI_PORT_CE, &GPIO_InitStructure);

	/* Connect SCK, MISO and MOSI pins to SPI alternate */
	GPIO_PinAFConfig(NRF_SPI_PORT_MOSI_SCK, NRF_SPI_PINSOURCE_SCK_MOSI,
	GPIO_AF_0);
	GPIO_PinAFConfig(NRF_SPI_PORT_MISO, NRF_SPI_PINSOURCE_MISO, GPIO_AF_0);

	/* Configure SPI */
	SPI_I2S_DeInit(SPI1);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;

	/* SPI baudrate is set to 6 MHz maximum (PCLK/SPI_BaudRatePrescaler = 48/8 = 6 MHz) */
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_Init(SPI1, &SPI_InitStructure);

	/* Configure the RX FIFO Threshold */
	SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);

	/* Enable SPI */
	SPI_Cmd(SPI1, ENABLE);

	//enable power to nrf chip
	nrf_Enable_PWR();

	//configure IRQ for EXT0
	//nrf_Irq_Init();

	//set chip select high and chip enable high
	CSN_HI;
	CE_LOW;
}

#if 0
void nrf_Irq_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable GPIOs clock */
	RCC_AHBPeriphClockCmd(NRF_IRQ_RCC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = NRF_IRQ_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(NRF_IRQ_PORT, &GPIO_InitStructure);

	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Tell system that you will use PB1 for EXTI_Line1 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);

	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStructure);

	/* PB1 is connected to EXTI_Line1, which has EXTI1_IRQn vector */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_EnableIRQ(ENABLE);
}
#endif

uint8_t spi_Send(uint8_t byte) {
	uint8_t ret;
	// Loop while DR register in not empty
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
		;

	// Send byte through the SPI1 peripheral
	SPI_SendData8(SPI1, byte);

	// Wait to receive a byte
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
		;

	// Return the byte read from the SPI bus
	ret = (uint8_t) SPI1->DR;

	return ret;
}

/**
 * @brief  This function handles External line 0 to 1 interrupt request.
 * @param  None
 * @retval None
 */

void EXTI0_1_IRQHandler(void)
{

	volatile int ret;
	ret = nRF24L01_check_transmit_success();
	if(ret == 1)
	{
		transmit = 1;
	}
	/* Clear the EXTI line 0 pending bit */
	EXTI_ClearITPendingBit(EXTI_Line0);

}


#if 0
void EXTI0_1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		transmit = WAIT_OVER;
		/* Clear the EXTI line 10 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line1);
		NVIC_DisableIRQ(EXTI0_1_IRQn);
	}

}
#endif

int nrf24l01_readregister(uint8_t address, uint8_t *rxbuf, int numbytes) {
	unsigned int count = 0;
	CSN_LOW;
	spi_Send(R_REGISTER | address);
	while (count < numbytes) {
		*(rxbuf + count) = spi_Send(NOP);
		count++;
	}
	CSN_HI;
	return 0;
}

uint8_t nrf24l01_getStatus(void) {
	uint8_t status;
	CSN_LOW;
	status = spi_Send(NOP);
	CSN_HI;
	return status;
}

int nrf24l01_writeregister(uint8_t address, uint8_t *data, int numbytes) {
	unsigned int count = 0;
	CSN_LOW;
	spi_Send(W_REGISTER | address);
	while (count < numbytes) {
		spi_Send(data[count]);
		count++;
	}
	CSN_HI;
	return 0;
}

void nrf24l01_sendcmd(uint8_t cmd) {
	CSN_LOW;
	spi_Send(cmd);
	CSN_HI;
}

int nrf24l01_writePayload(uint8_t *data, int numbytes) {
	unsigned int count = 0;
	CSN_LOW;
	spi_Send(W_TX_PAYLOAD);
	while (count < numbytes) {
		spi_Send(data[count]);
		count++;
	}
	CSN_HI;
	return 0;
}

int nrf24l01_readPayload(uint8_t *data, int numbytes) {
	unsigned int count = 0;
	CSN_LOW;
	spi_Send(R_RX_PAYLOAD);
	while (count < numbytes) {
		*(data + count) = spi_Send(NOP);
		count++;
	}
	CSN_HI;
	return 0;
}

void nrf24l01_clear_interrupts(void) {
	uint8_t data = _LS(RX_DR) | _LS(TX_DS) | _LS(MAX_RT);
	nrf24l01_writeregister(STATUS, &data, 1);
}

void nrf24l01_clear_transmit_interrupts(void) {
	uint8_t data = _LS(TX_DS) | _LS(MAX_RT);
	nrf24l01_writeregister(STATUS, &data, 1);
}

void nrf24l01_clear_receive_interrupts(void) {
	uint8_t status;
	status = nrf24l01_getStatus();
	status |= _LS(RX_DR);
	nrf24l01_writeregister(STATUS, &status, 1);
}

void nrf24l01_load_default(void) {
	uint8_t data = 0;
	uint8_t addr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };

	// load default value of config register
	data = 0x08;
	nrf24l01_writeregister(CONFIG, &data, 1);
	data = 0;

	// load default value of EN_AA register
	data = 0x3f;
	nrf24l01_writeregister(EN_AA, &data, 1);
	data = 0;

	// load default value of EN_RXADDR register
	data = 0x03;
	nrf24l01_writeregister(EN_RXADDR, &data, 1);

	// load default value of SETUP_AW register
	nrf24l01_writeregister(SETUP_AW, &data, 1);

	// load default value of SETUP_RETR register
	nrf24l01_writeregister(SETUP_RETR, &data, 1);
	data = 0;

	// load default value of RF_CH register
	data = 0x02;
	nrf24l01_writeregister(RF_CH, &data, 1);
	data = 0;

	// load default value of RF_SETUP register
	data = 0x0e;
	nrf24l01_writeregister(RF_SETUP, &data, 1);
	data = 0;

	// load default value of DYNPD register
	nrf24l01_writeregister(DYNPD, &data, 1);

	// load default value of FEATURE register
	nrf24l01_writeregister(FEATURE, &data, 1);

	// load default address value of  rx pipe0 register
	nrf24l01_writeregister(RX_ADDR_P0, addr, 5);

	// load default address value of tx pipe0 register
	nrf24l01_writeregister(TX_ADDR, addr, 5);
}

void nrf24l01_tx_Init(void) {
	uint8_t setup[1];
	uint8_t data;

	//spi peripheral Init
	spi_Rx_Init();
	//make slave select high
	CSN_HI;
	//make CE low
	CE_LOW;

	//need 100ms as start uo delay for nrf chip
	//nrf24l01_start_delay();
	delay_ms(500);
	setup[0] = 0;
	//load default value to nrf24l01 chip
	nrf24l01_load_default();

	// clear all interrupts
	nrf24l01_clear_interrupts();

	// setup auto retransmit delay to 1500us and max retransmit
	setup[0] = 0x5F;
	nrf24l01_writeregister(SETUP_RETR, setup, 1);
	setup[0] = 0;

	//setdata rate to 250KBPS and max output power(0 dbm)
	//setup[0] |= _LS(RF_DR_LOW) | _LS(2) | _LS(1);
	setup[0] = 0x26;
	nrf24l01_writeregister(RF_SETUP, setup, 1);

	//enable 2-bytes CRC,powerup mode
	data = _LS(EN_CRC) | _LS(CRCO) | _LS(PWR_UP);
	nrf24l01_writeregister(CONFIG, &data, 1);

	// enable Dynamic Payload on all pipes
	data =
			_LS(
					DPL_P0) | _LS(DPL_P1) | _LS(DPL_P2) | _LS(DPL_P3) | _LS(DPL_P4) | _LS(DPL_P5);
	nrf24l01_writeregister(DYNPD, &data, 1);
	data = 0;

	// enable dynamic payload (global) and enable payload in ack
	data = _LS(EN_DPL) | _LS(EN_ACK_PAY);
	nrf24l01_writeregister(FEATURE, &data, 1);
	data = 0;

	//enable auto ack for all channels
	data =
			_LS(
					ENAA_P0) | _LS(ENAA_P1) | _LS(ENAA_P2) | _LS(ENAA_P3) | _LS(ENAA_P4) | _LS(ENAA_P5);
	nrf24l01_writeregister(EN_AA, &data, 1);
	data = 0;

	//set rf channel
	data = 76;
	nrf24l01_writeregister(RF_CH, &data, 1);
	data = 0;

	//flush rx and tx FIFO
	nRF24L01_flush_transmit_message();
	nRF24L01_flush_receive_message();
}

void nrf24l01_rx_Init(void) {

	uint8_t setup[1];
	uint8_t data;

	//spi peripheral Init
	spi_Rx_Init();
	//make slave select high
	CSN_HI;
	//make CE low
	CE_LOW;

	//need 100ms as start uo delay for nrf chip
	delay_ms(100);

	setup[0] = 0;
	//load default value to nrf24l01 chip
	nrf24l01_load_default();

	// clear all interrupts
	nrf24l01_clear_interrupts();
	// setup auto retransmit delay to 1500us and max retransmit
	setup[0] |= 0x5F;
	nrf24l01_writeregister(SETUP_RETR,setup,1);
	setup[0]=0;

	//setdata rate to 250KBPS and max output power(0 dbm)
	//setup[0] |= _LS(RF_DR_LOW) | _LS(2) | _LS(1) | _LS(0);
	setup[0] = 0x26;
	nrf24l01_writeregister(RF_SETUP,setup,1);

	//enable 2-bytes CRC,powerup mode,PRX
	data = _LS(EN_CRC) | _LS(CRCO) | _LS(PWR_UP) | _LS(PRIM_RX);
	nrf24l01_writeregister(CONFIG,&data,1);

	// enable Dynamic Payload on all pipes
	data = _LS(DPL_P0) | _LS(DPL_P1) | _LS(DPL_P2) | _LS(DPL_P3) | _LS(DPL_P4) | _LS(DPL_P5);
	nrf24l01_writeregister(DYNPD,&data,1);
	data = 0;

	// enable dynamic payload (global) and enable payload in ack
	data = _LS(EN_DPL) | _LS(EN_ACK_PAY);
	nrf24l01_writeregister(FEATURE,&data,1);
	data=0;

	//set rf channel
	data = 76;
	nrf24l01_writeregister(RF_CH,&data,1);
	data = 0;

	//enable auto ack for all channels
	data = _LS(ENAA_P0) | _LS(ENAA_P1) | _LS(ENAA_P2) | _LS(ENAA_P3) | _LS(ENAA_P4) | _LS(ENAA_P5);
	nrf24l01_writeregister(EN_AA,&data,1);
	data = 0;

	//flush rx and tx FIFO
	nRF24L01_flush_transmit_message();
	nRF24L01_flush_receive_message();

#if 0
	uint8_t setup[1];
	uint8_t data;

	//spi peripheral Init
	//spi peripheral Init
	spi_Rx_Init();
	//make slave select high
	CSN_HI;
	//make CE low
	CE_LOW;

	//need 100ms as start uo delay for nrf chip
	//nrf24l01_start_delay();
	delay_ms(100);

	setup[0] = 0;
	//load default value to nrf24l01 chip
	nrf24l01_load_default();

	// clear all interrupts
	nrf24l01_clear_interrupts();

	// setup auto retransmit delay to 1500us and max retransmit
	setup[0] = 0x5F;
	nrf24l01_writeregister(SETUP_RETR, setup, 1);
	setup[0] = 0;

	//setdata rate to 250KBPS and max output power(0 dbm)
	//setup[0] |= _LS(RF_DR_LOW) | _LS(2) | _LS(1);
	setup[0] = 0x26;
	nrf24l01_writeregister(RF_SETUP, setup, 1);

	//enable 2-bytes CRC,powerup mode,PRX
	data = _LS(EN_CRC) | _LS(CRCO) | _LS(PWR_UP);
	nrf24l01_writeregister(CONFIG, &data, 1);

	// enable Dynamic Payload on all pipes
	data =
			_LS(
					DPL_P0) | _LS(DPL_P1) | _LS(DPL_P2) | _LS(DPL_P3) | _LS(DPL_P4) | _LS(DPL_P5);
	nrf24l01_writeregister(DYNPD, &data, 1);
	data = 0;

	// enable dynamic payload (global) and enable payload in ack
	data = _LS(EN_DPL) | _LS(EN_ACK_PAY);
	nrf24l01_writeregister(FEATURE, &data, 1);
	data = 0;

	//enable auto ack for all channels
	data =
			_LS(
					ENAA_P0) | _LS(ENAA_P1) | _LS(ENAA_P2) | _LS(ENAA_P3) | _LS(ENAA_P4) | _LS(ENAA_P5);
	nrf24l01_writeregister(EN_AA, &data, 1);
	data = 0;

	//set rf channel
	data = 76;
	nrf24l01_writeregister(RF_CH, &data, 1);
	data = 0;

	//flush rx and tx FIFO
	nRF24L01_flush_transmit_message();
	nRF24L01_flush_receive_message();
#endif
}

void nRF24L01_transmit(uint8_t *to_addr, uint8_t *msg, int msglen) {
	uint8_t config[1], temp[1];
	nrf24l01_clear_transmit_interrupts();
	nrf24l01_writeregister(TX_ADDR, to_addr, 5);//to set address
	nrf24l01_writeregister(RX_ADDR_P0, to_addr, 5); //to read ack (same add)
	nrf24l01_writePayload(msg, msglen);
	nrf24l01_readregister(CONFIG, config, 1);
	config[0] &= ~(_LS(PRIM_RX));
	nrf24l01_writeregister(CONFIG, config, 1);
	nrf24l01_readregister(EN_RXADDR, temp, 1);
	delay_ms(1500);
	CE_HI;
	delay_ms(10);
	CE_LOW;
	delay_ms(10);
	CE_HI;
}

uint8_t nRF24L01_check_transmit_success(void) {
	uint8_t success;
	uint8_t status;
	//CE_LOW;
	status = nrf24l01_getStatus();
	if ((status & _LS(TX_DS))) {
		if (status & _LS(RX_DR)) {
			success = ACK_PAYLOAD;
			nrf24l01_clear_interrupts();
		} else {
			success = EMPTY_ACK;
			nrf24l01_clear_transmit_interrupts();
		}
	} else if (status & _LS(MAX_RT)) {
		nrf24l01_clear_interrupts();
		success = RETRY_FAIL;
	} else {
		success = RF_ERROR;
	}
	return success;
}

void nRF24L01_flush_transmit_message(void) {
	nrf24l01_sendcmd(FLUSH_TX);
}
void nRF24L01_flush_receive_message(void) {
	nrf24l01_sendcmd(FLUSH_RX);
}

void nRF24L01_listen(int pipe, uint8_t *address) {
	uint8_t pipes[1];
	nrf24l01_writeregister(RX_ADDR_P0 + pipe, address, 5);
	//pipes[0]=0;
	//nrf24l01_readregister(EN_RXADDR,pipes,1);
	//pipes[0] |= _LS(pipe);
	//nrf24l01_writeregister(EN_RXADDR,pipes,1);
	nrf24l01_readregister(EN_AA, pipes, 1);
	nrf24l01_readregister(EN_RXADDR, pipes, 1);
	CE_HI;
}

int nRF24L01_check_data_received(void) {
	uint8_t status;
	status = nrf24l01_getStatus();
	if ((status & _LS(RX_DR)) && (status & _LS(TX_DS)))
		return 2;
	else if (status & _LS(RX_DR))
		return 1;
	else
		return 0;
}

int nRF24L01_get_received_pipe_number(void) {
	uint8_t status;
	int pipe_number;
	status = nrf24l01_getStatus();
	pipe_number = ((status & RX_P_NO_MASK) >> 1);
	return pipe_number <= 5 ? pipe_number : -1;
}

int nRF24L01_read_received_data(uint8_t *receive_msg) {
	int pipe_num;
	uint8_t msglen;
	//CE_LOW;
	pipe_num = nRF24L01_get_received_pipe_number();
	nrf24l01_clear_receive_interrupts();
	if (pipe_num < 0)
		return 0;
	nrf24l01_readregister(R_RX_PL_WID, &msglen, 1);

	if (msglen > 0) {
		nrf24l01_readPayload(receive_msg, msglen);
		sent_size = msglen;
	}
	return 1;
}

uint8_t nRF24L01_rx_payload_size(void) {
	uint8_t msglen;
	nrf24l01_readregister(R_RX_PL_WID, &msglen, 1);
	return msglen;
}

int nRF24L01_check_Tx_FIFO_Empty(void) {
	uint8_t fifo_status = 0;
	nrf24l01_readregister(FIFO_STATUS, &fifo_status, 1);
	if (fifo_status & _LS(TX_EMPTY))
		return 1;
	else
		return 0;
}

int nRF24L01_check_Tx_FIFO_FULL(void) {
	uint8_t fifo_status = 0;
	nrf24l01_readregister(FIFO_STATUS, &fifo_status, 1);
	if (fifo_status & _LS(FIFO_TX_FULL))
		return 1;
	else
		return 0;
}

int nRF24L01_check_Rx_FIFO_Empty(void) {
	uint8_t fifo_status = 0;
	nrf24l01_readregister(FIFO_STATUS, &fifo_status, 1);
	if (fifo_status & _LS(RX_EMPTY))
		return 1;
	else
		return 0;
}

int nRF24L01_check_Rx_FIFO_Full(void) {
	uint8_t fifo_status = 0;
	nrf24l01_readregister(FIFO_STATUS, &fifo_status, 1);
	if (fifo_status & _LS(RX_FULL))
		return 1;
	else
		return 0;
}

int nRF24L01_Write_Ack_Payload(uint8_t pipe, uint8_t *data, int datalen) {
	unsigned int count = 0;
	CSN_LOW;
	spi_Send(W_ACK_PAYLOAD | (pipe & 0x07));
	while (count < datalen) {
		spi_Send(data[count]);
		count++;
	}
	CSN_HI;
	return 0;
}

