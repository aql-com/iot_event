//////////////////////////////////////////////
//
// MAC_Rpt_SX1276_defs.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef MAC_RPT_SX1276_DEFS_H

#define MAC_RPT_SX1276_DEFS_H


unsigned char SX1276_init_vals[] = 
{
0x00,		// LREG_FIFO					0x00
0x00,		// LREG_OPMODE					0x01
0x00,		// LREG_BITRATE_MSB				0x02	R
0x0B,		// LREG_BITRATE_LSB				0x03	R
0x00,		// LREG_FDEV_MSB				0x04
0x52,		// LREG_FDEV_LSB				0x05	R

0x6C,		// LREG_FRF_MSB					0x06		// 6C5E00 = 433MHz: D90000 = 868MHHz
0x5E,		// LREG_FRF_MID					0x07
0x00,		// LREG_FRF_LSB					0x08

0xCF,		// LREG_PA_CFG					0x09	// MUST HAVE b7 set for SX1276 modules!
0x09,		// LREG_PA_RAMP					0x0A
0x2B,		// LREG_PA_OCP					0x0B
0x20,		// LREG_PA_LNA					0x0C

0x00,		// LREG_FIFO_ADDR_PTR			0x0D
0x80,		// LREG_FIFO_TX_BASE_ADDR		0x0E
0x00,		// LREG_FIFO_RX_BASE_ADDR		0x0F

0x00,		// LREG_FIFO_TX_CURR_ADDR		0x10	R

0x00,		// LREG_IRQ_MASK				0x11
0x00,		// LREG_IRQ_FLAGS				0x12	RC
0x00,		// LREG_RX_NUM_BYTES			0x13	R

0x00,		// LREG_RX_HDR_COUNT_MSB		0x14	R
0x00,		// LREG_RX_HDR_COUNT_LSB		0x15	R
0x00,		// LREG_RX_PKT_COUNT_MSB		0x16	RC
0x00,		// LREG_RX_PKT_COUNT_LSB		0x17	R

0x00,		// LREG_MDM_STAT				0x18	R
0x00,		// LREG_PKT_SNR_VAL				0x19	R
0x00,		// LREG_PKT_RSSI_VAL			0x1A	R
0x00,		// LREG_RSSI_VAL				0x1B	R
0x00,		// LREG_HOP_CHAN				0x1C	R

0x72,		// LREG_MDM_CFG1				0x1D
0x70,		// LREG_MDM_CFG2				0x1E
0x64,		// LREG_RX_TO_VAL				0x1F

0x00,		// LREG_PREAMBLE_MSB			0x20
0x08,		// LREG_PREAMBLE_LSB			0x21
0x01,		// LREG_PAYLOAD_LEN				0x22
0xFF,		// LREG_MAX_PAYLOAD_LEN			0x23
0x00,		// LREG_HOP_PERIOD				0x24

0x00,		// LREG_FIFO_RX_BYTE_ADDR		0x25	R
0x04,		// LREG_MDM_CFG3				0x26

0x00,		// LREG_RSVD_1					0x27	R

0x00,		// LREG_FERR_MSB				0x28	R
0x00,		// LREG_FERR_MID				0x29	R
0x00,		// LREG_FERR_LSB				0x2A	R
0x00,		// LREG_RSVD_2					0x2B	R
0x00,		// LREG_WB_RSSI_VAL				0x2C	R
0x50,		// LREG_RSVD_3					0x2D	R
0x14,		// LREG_RSVD_4					0x2E	R
0x40,		// LREG_IF_FREQ_1				0x2F	R
0x00,		// LREG_IF_FREQ_2				0x30	R
0xC3,		// LREG_LORA_DET_OPT			0x31	R
0x05,		// LREG_RSVD_5					0x32	R
0x27,		// LREG_INV_IQ					0x33	R	// 0x27 = no invert (GW Rx, Sensor Tx): 0x66 = invert (GW Tx, Sensor Rx)
0x1C,		// LREG_RSVD_6					0x34	R
0x0A,		// LREG_RSVD_7					0x35	R
0x32,		// LREG_HI_BW_OPT_1				0x36	R
0x0A,		// LREG_DET_THRESH				0x37	R

0x42,		// LREG_RSVD_8					0x38
0x12,		// LREG_SYNC_WORD				0x39
0x40,		// LREG_HI_BW_OPT_2				0x3A
0x1D,		// LREG_INV_IQ_2				0x3B
0x00,		// LREG_RSVD_9					0x3C
0xA1,		// LREG_RSVD_10					0x3D
0x00,		// LREG_RSVD_11					0x3E
0x00,		// LREG_RSVD_12					0x3F

0x00,		// LREG_DIO_MAP_1				0x40
0x00,		// LREG_DIO_MAP_2				0x41
0x00		// LREG_VERSION					0x42	R
};

#endif











