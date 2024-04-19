//////////////////////////////////////////////
//
// MAC_Rpt_SX1276.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef MAC_RPT_SX1276_H

#define MAC_RPT_SX1276_H

#include "MAC_Rpt.h"		// for LORAWAN_CMD_LIST_LENGTH, LORAWAN_CMD_LIST_ENTRY_LENGTH] defs...

// register defs - FSK use

// register defs - LORA use
#define LREG_FIFO					0x00
#define LREG_OPMODE					0x01
#define LREG_BITRATE_MSB			0x02
#define LREG_BITRATE_LSB			0x03
#define LREG_FDEV_MSB				0x04
#define LREG_FDEV_LSB				0x05

#define LREG_FRF_MSB				0x06
#define LREG_FRF_MID				0x07
#define LREG_FRF_LSB				0x08

#define LREG_PA_CFG					0x09
#define LREG_PA_RAMP				0x0A
#define LREG_PA_OCP					0x0B
#define LREG_PA_LNA					0x0C

#define LREG_FIFO_ADDR_PTR			0x0D
#define LREG_FIFO_TX_BASE_ADDR		0x0E
#define LREG_FIFO_RX_BASE_ADDR		0x0F

#define LREG_FIFO_RX_CURR_ADDR		0x10

#define LREG_IRQ_MASK				0x11
#define LREG_IRQ_FLAGS				0x12
#define LREG_RX_NUM_BYTES			0x13

#define LREG_RX_HDR_COUNT_MSB		0x14
#define LREG_RX_HDR_COUNT_LSB		0x15
#define LREG_RX_PKT_COUNT_MSB		0x16
#define LREG_RX_PKT_COUNT_LSB		0x17

#define LREG_MDM_STAT				0x18
#define LREG_PKT_SNR_VAL			0x19
#define LREG_PKT_RSSI_VAL			0x1A
#define LREG_RSSI_VAL				0x1B
#define LREG_HOP_CHAN				0x1C

#define LREG_MDM_CFG1				0x1D
#define LREG_MDM_CFG2				0x1E
#define LREG_RX_TO_VAL				0x1F

#define LREG_PREAMBLE_MSB			0x20
#define LREG_PREAMBLE_LSB			0x21
#define LREG_PAYLOAD_LEN			0x22
#define LREG_MAX_PAYLOAD_LEN		0x23
#define LREG_HOP_PERIOD				0x24

#define LREG_FIFO_RX_BYTE_ADDR		0x25
#define LREG_MDM_CFG3				0x26

#define LREG_RSVD_1					0x27

#define LREG_FERR_MSB				0x28
#define LREG_FERR_MID				0x29
#define LREG_FERR_LSB				0x2A
#define LREG_RSVD_2					0x2B
#define LREG_WB_RSSI_VAL			0x2C
#define LREG_RSVD_3					0x2D
#define LREG_RSVD_4					0x2E
#define LREG_IF_FREQ_1				0x2F
#define LREG_IF_FREQ_2				0x30
#define LREG_LORA_DET_OPT			0x31
#define LREG_RSVD_5					0x32
#define LREG_INV_IQ					0x33
#define LREG_RSVD_6					0x34
#define LREG_RSVD_7					0x35
#define LREG_HI_BW_OPT_1			0x36
#define LREG_DET_THRESH				0x37

#define LREG_RSVD_8					0x38
#define LREG_SYNC_WORD				0x39
#define LREG_HI_BW_OPT_2			0x3A
#define LREG_INV_IQ_2				0x3B
#define LREG_RSVD_9					0x3C
#define LREG_RSVD_10				0x3D
#define LREG_RSVD_11				0x3E
#define LREG_RSVD_12				0x3F

#define LREG_DIO_MAP_1				0x40
#define LREG_DIO_MAP_2				0x41
#define LREG_VERSION				0x42

#define LREG_RSVD_13				0x43
#define LREG_RSVD_14				0x44

#define LREG_TCXO					0x4B

#define LREG_PA_DAC					0x4D

#define LREG_FORMER_TEMP			0x5B
#define LREG_RSVD_15				0x5D

#define LREG_AGC_REF				0x61
#define LREG_AGC_THRESH_1			0x62
#define LREG_AGC_THRESH_2			0x63
#define LREG_AGC_THRESH_3			0x64

#define LREG_PLL_BW					0x70

// FSK regs
#define REG_CAL_RX                  0x3B
#define REG_TEMP                    0x3C
#define REG_LOWBAT                  0x3D

#define CAL_RX_START                0x40
#define CAL_RX_RUNNING              0x20

#define SPI_READ		0x00
#define SPI_WRITE		0x80
#define SPI_ADDR_MASK	0x7F

#define LORA_DIOO_RXDONE			0x00
#define LORA_DIOO_TXDONE			0x01
#define LORA_DIOO_CADDONE			0x02

#define LORA_DIO1_RXTIMEOUT			0x00
#define LORA_DIO1_HOPCHANGE			0x01
#define LORA_DIO1_CADDETECT			0x02

#define LORA_DIO2_HOPCHANGE			0x00

#define LORA_DIO3_CADDONE			0x00
#define LORA_DIO3_HDRVALID			0x01
#define LORA_DIO3_PAYCRCERR			0x02

#define LORA_DIO_NOTALLOWED			0x03

#endif

// NOTE array sizes must both use the [BLUETOOTH_CMD_LIST_LENGTH][BLUETOOTH_CMD_LIST_ENTRY_LENGTH] 
// so that they macth the parameters of the listmatch() routine in MAC_rpt_rtns.cpp...

extern char lorawan_cmd_whitelist[LORAWAN_CMD_LIST_LENGTH][LORAWAN_CMD_LIST_ENTRY_LENGTH];
extern char lorawan_cmd_blacklist[LORAWAN_CMD_LIST_LENGTH][LORAWAN_CMD_LIST_ENTRY_LENGTH];


// function prototypes
void SX1276_reset(void);
esp_err_t SX1276_read_reg(spi_device_handle_t spi, unsigned char reg, unsigned char *regval, unsigned char len);
esp_err_t SX1276_write_reg(spi_device_handle_t spi, unsigned char reg, unsigned char *regval, unsigned char len);

esp_err_t SX1276_init(spi_device_handle_t spi, unsigned char *init_array);
esp_err_t SX1276_set_868(spi_device_handle_t spi);
esp_err_t SX1276_set_freq(spi_device_handle_t spi, unsigned long freq, unsigned char no_lora_check);


unsigned char SX1276_send(spi_device_handle_t spi, unsigned char *data, unsigned char len, unsigned char async);
unsigned char SX1276_set_idle(spi_device_handle_t spi);
unsigned char SX1276_set_lora(spi_device_handle_t spi);
unsigned char SX1276_set_tx(spi_device_handle_t spi);
unsigned char SX1276_set_rx(spi_device_handle_t spi);
unsigned char SX1276_is_txmode(spi_device_handle_t spi);
unsigned char SX1276_get_mode(spi_device_handle_t spi, unsigned char *mode);
unsigned char SX1276_set_mode(spi_device_handle_t spi, unsigned char *mode);

void SX1276_get_regs(spi_device_handle_t spi, unsigned char start, unsigned char end);

void SX1276_get_dio_map(spi_device_handle_t spi, unsigned char *dio1_map, unsigned char *dio2_map);
void SX1276_set_dio_map(spi_device_handle_t spi, unsigned char dio1_map, unsigned char dio2_map);
signed char SX1276_get_rssi(spi_device_handle_t spi);
signed int SX1276_get_tx_power(spi_device_handle_t spi);


unsigned char SX1276_tx_packet_start(spi_device_handle_t spi);
unsigned char SX1276_tx_packet_write(spi_device_handle_t spi, unsigned char c);
unsigned char SX1276_tx_packet_block_write(spi_device_handle_t spi, unsigned char *data, unsigned char len);
unsigned char SX1276_tx_packet_end(spi_device_handle_t spi, unsigned char *data, unsigned char len);

void SX1276_calibrate_rx(spi_device_handle_t spi);











