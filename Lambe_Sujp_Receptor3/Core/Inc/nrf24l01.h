/*
 * nrf24l01.h
 *
 *  Created on: Mar 16, 2022
 *      Author: carlos
 *      Modificado:
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
//------ POWER GAIN ------//
/*VERY_LOW -> power lv -18dBm
 LOW       -> power lv -12dBm
 MID       -> power lv -6dBm
 HIGH      -> power lv 0dBm*/
typedef enum {
    veryLow, low, mid, high
} PaLevel;

//------ DATA RATE ------//
/*_250KBPS -> data rate 250 KBit/s
 _1MBPS    -> data rate 1 MBit/s
 _2MBPS    -> data rate 2 MBit/s*/
typedef enum {
    _250kbs, _1mbs, _2mbs
} DataRate;

//--------- MODES ---------//
/*PWRDOWM   -> 900nA
 STANBY     -> 320uA
 TX_MODE    -> 7 - 11.3mA no LNA, 150mA LNA
 RX_MODE    -> 12.6 - 13.5mA*/
typedef enum {
    pwrDown, standby, txMode, rxMode
} Mode;

//--------- CRC ---------//
/*NO_CRC    -> no CRC checksum is used
 CRC_8      -> CRC 8 bit checksum is used
 CRC_16     -> CRC 16 bit checksum is used*/
typedef enum {
    no_CRC, CRC_8, CRC_16
} Crclen;

//------ MODULE PARAMETERS ------//
typedef struct {
//Hardware settings:
GPIO_TypeDef *CSN_port;
uint16_t CSN_pin;
GPIO_TypeDef *CE_port;
uint16_t CE_pin;
GPIO_TypeDef *IRQ_port;
uint16_t IRQ_pin;
SPI_HandleTypeDef *hSPIx;

//Module settings:
PaLevel pa;
DataRate bitRate;
Mode mode;
Crclen crc;
} nrf24;

/* Exported constants --------------------------------------------------------*/
// Constant parameters
#define NRF_TX_TIMEOUT      2000
#define NRF_RX_TIMEOUT      2000
#define DEFAULT_CHANNEL     100

//------- REGISTERS -------//
#define CONFIG              0x00
#define EN_AA               0x01
#define EN_RXADDR           0x02
#define SETUP_AW            0x03
#define SETUP_RETR          0x04
#define RF_CH               0x05
#define RF_SETUP            0x06
#define STATUS              0x07
#define OBSERVE_TX          0x08
#define RPD                 0x09
#define RX_ADDR_P0          0x0A
#define RX_ADDR_P1          0x0B
#define RX_ADDR_P2          0x0C
#define RX_ADDR_P3          0x0D
#define RX_ADDR_P4          0x0E
#define RX_ADDR_P5          0x0F
#define TX_ADDR             0x10
#define RX_PW_P0            0x11
#define RX_PW_P1            0x12
#define RX_PW_P2            0x13
#define RX_PW_P3            0x14
#define RX_PW_P4            0x15
#define RX_PW_P5            0x16
#define FIFO_STATUS         0x17
#define DYNPD               0x1C
#define FEATURE             0x1D

//------ NRF24 STATUS ------//
#define NRF_OK              200
#define NRF_NOT_FOUND       404
#define NRF_LARGE_PAYLOAD   413
#define NRF_UNAVAILABLE     503

//------Instruction Mnemonics ------//
#define R_REGISTER          0x00
#define W_REGISTER          0x20
#define REGISTER_MASK       0x1F
#define ACTIVATE            0x50
#define R_RX_PL_WID         0x60
#define R_RX_PAYLOAD        0x61
#define W_TX_PAYLOAD        0xA0
#define W_ACK_PAYLOAD       0xA8
#define FLUSH_TX            0xE1
#define FLUSH_RX            0xE2
#define REUSE_TX_PL         0xE3
#define NOP                 0xFF

/* Exported macros -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Variables------------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void nrf24_init(nrf24 *node);
void nrf24_setDataRate(nrf24 *node, DataRate _bitRate);
void nrf24_setPALevel(nrf24 *node, PaLevel pwr);
void nrf24_setChannel(nrf24 *node, uint8_t channel);
void nrf24_setMode(nrf24 *node, Mode _mode);
void nrf24_setCrcLentgh(nrf24 *node, Crclen _len);
void nrf24_setTxAddr(nrf24 *node, uint8_t *Address);
uint8_t nrf24_Transmit(nrf24 *node, uint8_t *data, uint8_t len);
void nrf24_setRxPipe(nrf24 *node, uint8_t *addr, uint8_t pipe, uint8_t payload);
uint8_t isDataAvailable(nrf24 *node, uint8_t pipenum);
void nrf24_Receive(nrf24 *node, uint8_t *data, uint8_t len);
void nrf24_ReadAll(nrf24 *node, uint8_t *data);

#endif /* INC_NRF24L01_H_ */
