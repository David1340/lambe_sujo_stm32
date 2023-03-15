/*
 * nrf24l01.c
 *
 *  Created on: Mar 16, 2022
 *      Author: carlos
 *      Modificado:
 */

/* Includes ------------------------------------------------------------------*/
#include "nrf24l01.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void nrf24_CE_enable(nrf24 *node);
static void nrf24_CE_disable(nrf24 *node);
static void nrf24_CSN_enable(nrf24 *node);
static void nrf24_CSN_disable(nrf24 *node);
static void nrf24_writeReg(nrf24 *node, uint8_t reg, uint8_t data);
static void nrf24_writeRegMulti(nrf24 *node, uint8_t reg, uint8_t *data,
		uint16_t len);
static uint8_t nrf24_readReg(nrf24 *node, uint8_t reg);
static void nrf24_readRegMulti(nrf24 *node, uint8_t reg, uint8_t *data,
		uint16_t len);
static void nrf_sendCmd(nrf24 *node, uint8_t cmd);
static void nrf24_reset(nrf24 *node, uint8_t reg);

/* Exported functions --------------------------------------------------------*/
void nrf24_init(nrf24 *node) {
	nrf24_CE_disable(node);	//disable the chip before configure the device

	nrf24_writeReg(node, CONFIG, 0);  			//No IRQ, no CRC
	node->crc = no_CRC;
	nrf24_writeReg(node, EN_AA, 0x00); 			//No auto ACK
	nrf24_writeReg(node, EN_RXADDR, 0x00);	//Not enabling any data pipe
	nrf24_writeReg(node, SETUP_AW, 0x03);  	//5 Bytes for the TX/RX address
	nrf24_writeReg(node, SETUP_RETR, 0);   	//No retransmission

	nrf24_setDataRate(node, _250kbs);		//set minimum dataRate
	nrf24_setPALevel(node, high);				//set maximum PALevel 0dBm
	nrf24_setChannel(node, DEFAULT_CHANNEL);	//default channel 100
	nrf24_setMode(node, standby);		//set STANDBY mode

	nrf24_CE_enable(node);	// Enable the chip after configuring the device
}
// -----------------------------------------------------------------------------

//set module bitrate
void nrf24_setDataRate(nrf24 *node, DataRate _bitRate) {
	nrf24_CE_disable(node);
	uint8_t config = nrf24_readReg(node, RF_SETUP);

	switch (_bitRate) {
		case _250kbs:
			config |= (1 << 5);		//write 1 in the RF_DR_LOW and 0 in RF_DR_HIGH bits
			config &= ~(1 << 3);
			node->bitRate = _250kbs;
			break;

		case _1mbs:
			config &= ~(1 << 5);	//write 0 in the RF_DR_LOW and 0 in RF_DR_HIGH bits
			config &= ~(1 << 3);
			node->bitRate = _1mbs;
			break;

		case _2mbs:
			config &= ~(1 << 5);	//write 0 in the RF_DR_LOW and 1 in RF_DR_HIGH bits
			config |= (1 << 3);
			node->bitRate = _2mbs;
			break;

		default:
			break;
	}
	nrf24_writeReg(node, RF_SETUP, config);
	nrf24_CE_enable(node);
}
// -----------------------------------------------------------------------------

//set PAlevel module in TX mode
void nrf24_setPALevel(nrf24 *node, PaLevel pwr) {
	nrf24_CE_disable(node);	//disable the chip before configure the device
	uint8_t config = nrf24_readReg(node, RF_SETUP);

	switch (pwr) {
		case veryLow:
			config &= ~(1 << 2);	//write 0 in the RF_PWR bits
			config &= ~(1 << 1);
			node->pa = veryLow;
			break;

		case low:
			config &= ~(1 << 2);	//write 1 in the RF_PWR bits
			config |= (1 << 1);
			node->pa = low;
			break;

		case mid:
			config |= (1 << 2);		//write 2 in the RF_PWR bits
			config &= ~(1 << 1);
			node->pa = mid;
			break;

		case high:
			config |= (1 << 2) | (1 << 1);	//write 3 in the RF_PWR bits
			node->pa = high;
			break;

		default:
			break;
	}
	nrf24_writeReg(node, RF_SETUP, config);
	nrf24_CE_enable(node);	//Enable the chip after configuring the device
}
// -----------------------------------------------------------------------------

//Set the channel device
void nrf24_setChannel(nrf24 *node, uint8_t channel) {
	nrf24_CE_disable(node);	//disable the chip before configure the device
	nrf24_writeReg(node, RF_CH, channel);  // select the channel
	nrf24_CE_enable(node);	//Enable the chip after configuring the device
}
// -----------------------------------------------------------------------------

//Set the module operation mode
void nrf24_setMode(nrf24 *node, Mode _mode) {
	uint8_t config = nrf24_readReg(node, CONFIG);

	switch (_mode) {
		case pwrDown:
			config &= ~(1 << 1);	//write 0 in the PWR_UP bit
			node->mode = pwrDown;
			break;

		case standby:
			nrf24_CE_disable(node);
			config |= (1 << 1);		//write 1 in the PWR_UP bit
			node->mode = standby;
			break;

		case txMode:
			nrf24_CE_enable(node);
			config |= (1 << 1);		//write 1 in the PWR_UP bit
			config &= ~(1 << 0);	//write 0 in the PRIM_RX bit
			node->mode = txMode;
			break;

		case rxMode:
			nrf24_CE_enable(node);
			config |= (1 << 1) | (1 << 0); //write 1 in the PWR_UP and PRIM_RX bits
			node->mode = rxMode;
			break;

		default:
			break;
	}
	nrf24_writeReg(node, CONFIG, config);
}
// -----------------------------------------------------------------------------

//set the CRC lentgh
void nrf24_setCrcLentgh(nrf24 *node, Crclen _len) {
	nrf24_CE_disable(node);	//disable the chip before configure the device
	uint8_t config = nrf24_readReg(node, CONFIG);

	switch (_len) {
		case no_CRC:
			config &= ~(1 << 3);	//write 0 in the EN_CRC bit
			config &= ~(1 << 2);	//write 0 in the CRC0 bit
			nrf24_writeReg(node, CONFIG, config);
			node->crc = no_CRC;
			break;

		case CRC_8:
			config |= (1 << 3);		//write 1 in the EN_CRC bit
			config &= ~(1 << 2);	//write 0 in the CRC0 bit
			node->crc = CRC_8;
			break;

		case CRC_16:
			config |= (1 << 3) | (1 << 2);  //write 1 in the EN_CRC and CRC0 bits
			node->crc = CRC_16;
			break;

		default:
			break;
	}
	nrf24_writeReg(node, CONFIG, config);
	nrf24_CE_enable(node);	//Enable the chip after configuring the device
}
// -----------------------------------------------------------------------------

// set up the Tx mode
void nrf24_setTxAddr(nrf24 *node, uint8_t *Address) {
	nrf24_CE_disable(node);	//disable the chip before configure the device
	nrf24_writeRegMulti(node, TX_ADDR, Address, 5);  // Write the TX address
	nrf24_CE_enable(node);	//Enable the chip after configuring the device
}
// -----------------------------------------------------------------------------

//transmit the data
uint8_t nrf24_Transmit(nrf24 *node, uint8_t *data, uint8_t len) {
	uint8_t cmdtosend = 0;

	nrf24_CSN_enable(node);		//select the device
	cmdtosend = W_TX_PAYLOAD;	//payload command
	HAL_SPI_Transmit(node->hSPIx, &cmdtosend, 1, NRF_TX_TIMEOUT);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);
	// send the payload
	if (len < 33) HAL_SPI_Transmit(node->hSPIx, data, len, NRF_TX_TIMEOUT);
	nrf24_CSN_disable(node);	//Unselect the device

	uint8_t timeout = 0;
	while (timeout++ < 10) {
		for (int i = 0; i < 200; ++i) {
			__ASM("NOP");
		}
		uint8_t fifostatus = nrf24_readReg(node, FIFO_STATUS);

		//check the fourth bit of FIFO_STATUS to know if the TX fifo is empty
		if ((fifostatus & (1 << 4)) && (!(fifostatus & (1 << 3)))) {
			cmdtosend = FLUSH_TX;
			nrf_sendCmd(node, cmdtosend);
			nrf24_reset(node, FIFO_STATUS);	//reset FIFO_STATUS
			return (1);
		}
	}
	return (0);
}
// -----------------------------------------------------------------------------

void nrf24_setRxPipe(nrf24 *node, uint8_t *addr, uint8_t pipe, uint8_t payload) {
	nrf24_CE_disable(node);	//disable the chip before configure the device

	nrf24_reset(node, STATUS);
	uint8_t en_rxaddr = nrf24_readReg(node, EN_RXADDR);	//select data pipe
	en_rxaddr |= (1 << pipe);
	nrf24_writeReg(node, EN_RXADDR, en_rxaddr);

	/* We must write the address for Data Pipe 1, if we want to use any pipe from 2 to 5
	 The Address from DATA Pipe 2 to Data Pipe 5 differs only in the LSB
	 Their 4 MSB Bytes will still be same as Data Pipe 1
	 For Example:
	 Pipe 1 ADDR = 0xAABBCCDD11, Pipe 2 ADDR = 0xAABBCCDD22 Pipe 3 ADDR = 0xAABBCCDD33*/
	switch (pipe) {
		case 0:
			nrf24_writeRegMulti(node, RX_ADDR_P0, addr, 5); //Write the Pipe0 address
			nrf24_writeReg(node, RX_PW_P0, payload); //payload size for pipe 0
			break;

		case 1:
			nrf24_writeRegMulti(node, RX_ADDR_P1, addr, 5); //Write the Pipe1 address
			nrf24_writeReg(node, RX_PW_P1, payload); 	//payload size for pipe 1
			break;

		case 2:
			nrf24_writeReg(node, RX_ADDR_P2, addr[0]);	//Write the Pipe2 address
			nrf24_writeReg(node, RX_PW_P2, payload); 		//payload size for pipe 2
			break;

		case 3:
			nrf24_writeReg(node, RX_ADDR_P3, addr[0]);	// Write the Pipe3 address
			nrf24_writeReg(node, RX_PW_P3, payload); 		//payload size for pipe 3
			break;

		case 4:
			nrf24_writeReg(node, RX_ADDR_P4, addr[0]); 	//Write the Pipe4 address
			nrf24_writeReg(node, RX_PW_P4, payload); 		//payload size for pipe 4
			break;

		case 5:
			nrf24_writeReg(node, RX_ADDR_P5, addr[0]); 	// Write the Pipe5 address
			nrf24_writeReg(node, RX_PW_P5, payload); 		//payload size for pipe 5
			break;

		default:
			break;
	}
	nrf24_CE_enable(node);	//Enable the chip after configuring the device
}
// -----------------------------------------------------------------------------

uint8_t isDataAvailable(nrf24 *node, uint8_t pipenum) {
	uint8_t status = nrf24_readReg(node, STATUS);
	if ((status - 64 == 0) && pipenum == 0) {
		nrf24_writeReg(node, STATUS, (1 << 6));
		return 1;
	}
	else if ((status & (1 << 6)) && (status & (pipenum << 1))) {
		nrf24_writeReg(node, STATUS, (1 << 6));
		return 1;
	}
	return 0;
}
// -----------------------------------------------------------------------------

void nrf24_Receive(nrf24 *node, uint8_t *data, uint8_t len) {
	uint8_t cmdtosend = 0;

	nrf24_CSN_enable(node);		//select the device
	cmdtosend = R_RX_PAYLOAD;	//payload command
	HAL_SPI_Transmit(node->hSPIx, &cmdtosend, 1, 100);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);
	//Receive the payload
	HAL_SPI_Receive(node->hSPIx, data, len, NRF_RX_TIMEOUT);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);
	nrf24_CSN_disable(node);	//Unselect the device
	HAL_Delay(1);

	cmdtosend = FLUSH_RX;
	nrf_sendCmd(node, cmdtosend);
}
// -----------------------------------------------------------------------------

// Read all the Register data
void nrf24_ReadAll(nrf24 *node, uint8_t *data) {
	for (int i = 0; i < 10; i++)
		*(data + i) = nrf24_readReg(node, i);

	nrf24_readRegMulti(node, RX_ADDR_P0, (data + 10), 5);
	nrf24_readRegMulti(node, RX_ADDR_P1, (data + 15), 5);

	*(data + 20) = nrf24_readReg(node, RX_ADDR_P2);
	*(data + 21) = nrf24_readReg(node, RX_ADDR_P3);
	*(data + 22) = nrf24_readReg(node, RX_ADDR_P4);
	*(data + 23) = nrf24_readReg(node, RX_ADDR_P5);

	nrf24_readRegMulti(node, RX_ADDR_P0, (data + 24), 5);

	for (int i = 29; i < 38; i++)
		*(data + i) = nrf24_readReg(node, i - 12);
}
// -----------------------------------------------------------------------------

/* privated functions --------------------------------------------------------*/
static void nrf24_CE_enable(nrf24 *node) {
	HAL_GPIO_WritePin(node->CE_port, node->CE_pin, GPIO_PIN_SET);
}
// -----------------------------------------------------------------------------

static void nrf24_CE_disable(nrf24 *node) {
	HAL_GPIO_WritePin(node->CE_port, node->CE_pin, GPIO_PIN_RESET);
}
// -----------------------------------------------------------------------------

static void nrf24_CSN_enable(nrf24 *node) {
	HAL_GPIO_WritePin(node->CSN_port, node->CSN_pin, GPIO_PIN_RESET);
}
// -----------------------------------------------------------------------------

static void nrf24_CSN_disable(nrf24 *node) {
	HAL_GPIO_WritePin(node->CSN_port, node->CSN_pin, GPIO_PIN_SET);
}
// -----------------------------------------------------------------------------

//write a value(s) in a register(s) by an address
static void nrf24_writeReg(nrf24 *node, uint8_t reg, uint8_t data) {
	uint8_t buf[2];
	buf[0] = (reg | 1 << 5);
	buf[1] = data;
	nrf24_CSN_enable(node);
	HAL_SPI_Transmit(node->hSPIx, buf, 2, NRF_TX_TIMEOUT);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);
	nrf24_CSN_disable(node);
}
// -----------------------------------------------------------------------------

//write multiple values in a registers by an address
static void nrf24_writeRegMulti(nrf24 *node, uint8_t reg, uint8_t *data,
		uint16_t len) {
	uint8_t buf[2];
	buf[0] = (reg | 1 << 5);
	nrf24_CSN_enable(node);
	HAL_SPI_Transmit(node->hSPIx, buf, 1, NRF_TX_TIMEOUT);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(node->hSPIx, data, len, NRF_TX_TIMEOUT);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);
	nrf24_CSN_disable(node);
}
// -----------------------------------------------------------------------------

//read a value from a register by an address
static uint8_t nrf24_readReg(nrf24 *node, uint8_t reg) {
	uint8_t data = 0;
	nrf24_CSN_enable(node);
	HAL_SPI_Transmit(node->hSPIx, &reg, 1, NRF_TX_TIMEOUT);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);
	HAL_SPI_Receive(node->hSPIx, &data, 1, NRF_RX_TIMEOUT);
	nrf24_CSN_disable(node);
	return data;
}
// -----------------------------------------------------------------------------

//read multiple values from a registers by an address
static void nrf24_readRegMulti(nrf24 *node, uint8_t reg, uint8_t *data,
		uint16_t len) {
	nrf24_CSN_enable(node);
	HAL_SPI_Transmit(node->hSPIx, &reg, 1, NRF_TX_TIMEOUT);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);
	HAL_SPI_Receive(node->hSPIx, data, len, NRF_RX_TIMEOUT);
	nrf24_CSN_disable(node);
}
// -----------------------------------------------------------------------------

static void nrf_sendCmd(nrf24 *node, uint8_t cmd) {
	nrf24_CSN_enable(node);
	HAL_SPI_Transmit(node->hSPIx, &cmd, 1, NRF_TX_TIMEOUT);
	while (HAL_SPI_GetState(node->hSPIx) != HAL_SPI_STATE_READY);
	nrf24_CSN_disable(node);
}
// -----------------------------------------------------------------------------

static void nrf24_reset(nrf24 *node, uint8_t reg) {
	if (reg == STATUS)
		nrf24_writeReg(node, STATUS, 0x00);
	else if (reg == FIFO_STATUS)
		nrf24_writeReg(node, FIFO_STATUS, 0x11);

	else {
		nrf24_writeReg(node, CONFIG, 0x08);
		nrf24_writeReg(node, EN_AA, 0x3F);
		nrf24_writeReg(node, EN_RXADDR, 0x03);
		nrf24_writeReg(node, SETUP_AW, 0x03);
		nrf24_writeReg(node, SETUP_RETR, 0x03);
		nrf24_writeReg(node, RF_CH, 0x02);
		nrf24_writeReg(node, RF_SETUP, 0x0E);
		nrf24_writeReg(node, STATUS, 0x00);
		nrf24_writeReg(node, OBSERVE_TX, 0x00);
		nrf24_writeReg(node, RPD, 0x00);
		uint8_t rx_addr_p0_def[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };
		nrf24_writeRegMulti(node, RX_ADDR_P0, rx_addr_p0_def, 5);
		uint8_t rx_addr_p1_def[5] = { 0xC2, 0xC2, 0xC2, 0xC2, 0xC2 };
		nrf24_writeRegMulti(node, RX_ADDR_P1, rx_addr_p1_def, 5);
		nrf24_writeReg(node, RX_ADDR_P2, 0xC3);
		nrf24_writeReg(node, RX_ADDR_P3, 0xC4);
		nrf24_writeReg(node, RX_ADDR_P4, 0xC5);
		nrf24_writeReg(node, RX_ADDR_P5, 0xC6);
		uint8_t tx_addr_def[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };
		nrf24_writeRegMulti(node, TX_ADDR, tx_addr_def, 5);
		nrf24_writeReg(node, RX_PW_P0, 0);
		nrf24_writeReg(node, RX_PW_P1, 0);
		nrf24_writeReg(node, RX_PW_P2, 0);
		nrf24_writeReg(node, RX_PW_P3, 0);
		nrf24_writeReg(node, RX_PW_P4, 0);
		nrf24_writeReg(node, RX_PW_P5, 0);
		nrf24_writeReg(node, FIFO_STATUS, 0x11);
		nrf24_writeReg(node, DYNPD, 0);
		nrf24_writeReg(node, FEATURE, 0);
	}
}
// -----------------------------------------------------------------------------
