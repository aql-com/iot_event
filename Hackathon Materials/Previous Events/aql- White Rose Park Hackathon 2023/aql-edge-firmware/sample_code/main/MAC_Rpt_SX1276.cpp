//////////////////////////////////////////////
//
// MAC_Rpt_SX1276.cpp
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

//#include "driver/spi_common.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "memory.h"
#include "esp32/rom/ets_sys.h"	// ets usec delays


#include "MAC_Rpt.h"
#include "MAC_Rpt_Rtns.h"
#include "MAC_Rpt_SX1276.h"
//#include "MAC_Rpt_SX1276_defs.h"

extern unsigned long int dbgbits;
extern unsigned char dbgflag;

extern unsigned char lora_reset_timer;
extern unsigned char lora_reset_complete_flag;
extern unsigned char lora_tx_packet_open;
extern unsigned char SX1276_dio1_map,SX1276_dio2_map;


void SX1276_reset(void)
{
// needs 1 msec...
// on shift reg pin on V1_0_D, so must use set_output function...
set_output(DEV_LORA_RADIO_RESET,0,0);	// 
lora_reset_timer = 0;
lora_reset_complete_flag = 0;
if(debug_do(DBG_LORA))
	printf("LORA reset start...\n");
ets_delay_us(1000);
}

unsigned char setup_spi_bus(unsigned char reg, unsigned char *regval, unsigned char len)
{
unsigned char err = 0;


return err;	
}

esp_err_t SX1276_read_reg(spi_device_handle_t spi, unsigned char reg, unsigned char *regval, unsigned char len)
{
static spi_transaction_t trans;
  	
esp_err_t err = 0;

memset(&trans, 0, sizeof(spi_transaction_t));

trans.cmd = 0;
trans.addr = SPI_READ | (reg & SPI_ADDR_MASK);
trans.length= len * 8;	// total msg length in bits..???
trans.rxlength = len * 8;	// rx length in bits..???
trans.user = (void*)0;
trans.flags = SPI_TRANS_USE_RXDATA;	// put data into trans.rx_data[4];

//memcpy(&trans.tx_data,regval,len); 
 
//err = spi_device_queue_trans(spi, &trans, portMAX_DELAY);
err = spi_device_transmit(spi, &trans);
if (err)
	{
	printf("SPI RD Err: %04X\n",err);
	}

//*regval = trans.rx_data[0];
memcpy(regval,trans.rx_data,len); 
	
//err = spi_device_get_trans_result(spi,&trans,10);
//if (err)
//	printf("SPI Rx Err: %04X\n",err);
	 
return err;	
}

esp_err_t SX1276_write_reg(spi_device_handle_t spi, unsigned char reg, unsigned char *regval, unsigned char len)
{
static spi_transaction_t trans;
  	
esp_err_t err = 0;

memset(&trans, 0, sizeof(spi_transaction_t));

trans.cmd = 0;
trans.addr = SPI_WRITE | (reg & SPI_ADDR_MASK);
trans.length= len * 8;	// total msg length in bits..???
trans.rxlength = 0;				// WRITE - zero return data - rx length in bits..???
trans.user=(void*)0;
trans.flags=SPI_TRANS_USE_TXDATA;

memcpy(trans.tx_data,regval,len); 
 
//err=spi_device_queue_trans(spi, &trans, portMAX_DELAY);
err = spi_device_transmit(spi, &trans);
if (err)
	{
	printf("SPI WR Err: %04X\n",err);
	}


return err;	
}


esp_err_t SX1276_init(spi_device_handle_t spi, unsigned char *init_array)
{
unsigned char i,val,x;
char tmpstr[15];

esp_err_t spi_err = 0;
esp_err_t err;

nvs_handle nvs_handle;


if(debug_do(DBG_LORA))
	{
	printf("LoRa Init\n");
	spi_err = SX1276_read_reg(spi, LREG_VERSION,&val,1);
	printf("LoRa Chip Ver: 0x%02X\n",val);
	}

// calibrate Rx chain


// set DIO pin mapping...
x = 0x00;	//AA;
SX1276_write_reg(spi,LREG_DIO_MAP_1,&x,1);
x = 0x00;	//A0;
SX1276_write_reg(spi,LREG_DIO_MAP_2,&x,1);

// switch mode to LoRa
val = 0x00;		// sleep mode
spi_err = SX1276_write_reg(spi, LREG_OPMODE,&val,1);
val = 0x80;		// LoRa mode
spi_err = SX1276_write_reg(spi, LREG_OPMODE,&val,1);

// open LORA REG LIST for values which are modified by system cmds...
err = nvs_open("LORA_REG_LIST",NVS_READWRITE, &nvs_handle);

// now fill the registers with default values
for (i=LREG_BITRATE_MSB;i<LREG_FIFO_RX_CURR_ADDR;i++)
	{
	spi_err = SX1276_write_reg(spi,i,&init_array[i],1);
	}

spi_err = SX1276_write_reg(spi,LREG_IRQ_MASK,&init_array[LREG_IRQ_MASK],1);

// write LREG_PA_CFG register from NVS value
sprintf(tmpstr,"LORA_REG_%02d",LREG_PA_CFG);
nvs_get_u8(nvs_handle,tmpstr,(uint8_t*)&i);
spi_err = SX1276_write_reg(spi,LREG_PA_CFG,&i,1);

for (i=LREG_MDM_CFG1;i<LREG_FIFO_RX_BYTE_ADDR;i++)
	{
	spi_err = SX1276_write_reg(spi,i,&init_array[i],1);
	}

// write LREG_MDM_CFG1 register from NVS value
sprintf(tmpstr,"LORA_REG_%02d",LREG_MDM_CFG1);
nvs_get_u8(nvs_handle,tmpstr,(uint8_t*)&i);
spi_err = SX1276_write_reg(spi,LREG_MDM_CFG1,&i,1);

// write LREG_MDM_CFG2 register from NVS value
sprintf(tmpstr,"LORA_REG_%02d",LREG_MDM_CFG2);
nvs_get_u8(nvs_handle,tmpstr,(uint8_t*)&i);
spi_err = SX1276_write_reg(spi,LREG_MDM_CFG2,&i,1);

spi_err = SX1276_write_reg(spi,LREG_MDM_CFG3,&init_array[LREG_MDM_CFG3],1);


//for (i=LREG_RSVD_8;i<LREG_DIO_MAP_2;i++)
for (i=LREG_RSVD_8;i<LREG_VERSION;i++)
	{
	spi_err = SX1276_write_reg(spi,i,&init_array[i],1);
	}

nvs_close(nvs_handle);

// update local DIO state variables
SX1276_dio1_map = init_array[LREG_DIO_MAP_1];
SX1276_dio2_map = init_array[LREG_DIO_MAP_2];

return spi_err;	
}


esp_err_t SX1276_set_868(spi_device_handle_t spi)
{
return 0;
}

esp_err_t SX1276_set_freq(spi_device_handle_t spi, unsigned long freq, unsigned char no_lora_check)
{
esp_err_t spi_err = 0;
unsigned char i,val[4];
unsigned long long f;

if(debug_do(DBG_LORA))
	printf("Set LoRa Freq\n");

// select rounded or unrounded freq value...
#if 0
f = (unsigned long long)freq * 65536 * 8 / 32000000;
#else
f = (unsigned long long)freq * 65536 * 8 * 10 / 32000000;

// round up value for better accuracy
if (f%10 > 4)
	i = 1;
else
	i = 0;

f = (f/10) + i;	// rounding in i
#endif

printf("Frf: %llu    0x%llX\n",f,f);

// check in LoRa mode...
spi_err = SX1276_read_reg(spi, LREG_OPMODE,val,1);

if(debug_do(DBG_LORA))
	printf("OPMODE = %02X : ",val[0]);

if (((val[0] & 0x80) || (no_lora_check)) && ((val[0] & 0x07) < 2))	// LoRa mode and sleep or standby
	{
	val[0] = f>>16;			// 0xD9;
	val[1] = (f>>8) & 0xFF;	// 0x00;
	val[2] = f & 0xFF;		// 0x00;

	spi_err = SX1276_write_reg(spi,LREG_FRF_MSB,val,3);

#if 1
	if (spi_err)
			printf("SPI Wr Error! %d\n",spi_err);
#endif

	if(debug_do(DBG_LORA))
		{
		printf("set %d.%dMHz [%02X %02X %02X]\n",(unsigned int)(freq/1000000),(unsigned int)((freq%1000000)/1000),val[0],val[1],val[2]);
#if 0
		val[0] = 0x01;
		val[1] = 0x02;
		val[2] = 0x03;

		spi_err = SX1276_read_reg(spi, LREG_FRF_MSB,val,3);
		
		printf("Regs [%02X %02X %02X]\n",val[0],val[1],val[2]);

/*
		spi_err = SX1276_read_reg(spi, LREG_FRF_MSB,&val[0],1);
		spi_err = SX1276_read_reg(spi, LREG_FRF_MID,&val[1],1);
		spi_err = SX1276_read_reg(spi, LREG_FRF_LSB,&val[2],1);
		
		printf("Regs [%02X %02X %02X]\n",val[0],val[1],val[2]);
*/
#endif
		}
	
	}
else
	{
	if(debug_do(DBG_LORA))
		{
		printf("LoRa mode error!  \n");
		if ((val[0] & 0x80) == 0)
			printf("not in LoRa mode!\n");
		if ((val[0] & 0x07) > 1)
			printf("not in Sleep / Stdby mode!\n");
		}
	}

return spi_err;	
}

unsigned char SX1276_send(spi_device_handle_t spi, unsigned char *data, unsigned char len, unsigned char async)
{
unsigned char err = 0;
unsigned char i;
esp_err_t spi_err = 0;
unsigned char val;

if (SX1276_is_txmode(spi))
	{
	err = 1;
	}
else
	{
	SX1276_set_idle(spi);
// reset Tx FIFO 
	i = 0;
//	spi_err = SX1276_write_reg(spi, LREG_FIFO_TX_BASE_ADDR,&i,1);
	spi_err = SX1276_read_reg(spi, LREG_FIFO_TX_BASE_ADDR,&i,1);
	spi_err = SX1276_write_reg(spi, LREG_FIFO_ADDR_PTR,&i,1);	
// buffer data
	for (i=0;i<len;i++)
		{
//		spi_err = SX1276_write_reg(spi, LREG_FIFO_TX_BASE_ADDR,&data[i],1);
		spi_err = SX1276_write_reg(spi, LREG_FIFO,&data[i],1);
		}

	SX1276_set_tx(spi);
	
// wait for TX_DONE
	if (!async)
		{
		SX1276_read_reg(spi, LREG_IRQ_FLAGS,&val,1);
		while ((val & 0x08) == 0)
			{	
			SX1276_read_reg(spi, LREG_IRQ_FLAGS,&val,1);
			}
		}
		
// clear IRQ flags	
	val = 0x08;	// need to write a '1' to clear an int bit...
	SX1276_write_reg(spi,LREG_IRQ_FLAGS,&val,1);

	}

return err;	
}

unsigned char SX1276_set_idle(spi_device_handle_t spi)
{
unsigned char err = 0;
esp_err_t spi_err = 0;
unsigned char val;

spi_err = SX1276_read_reg(spi, LREG_OPMODE,&val,1);

val = val & 0xF8;		// sleep mode
spi_err = SX1276_write_reg(spi, LREG_OPMODE,&val,1);

return err;	
}

unsigned char SX1276_set_lora(spi_device_handle_t spi)
{
unsigned char err = 0;
esp_err_t spi_err = 0;
unsigned char val;

spi_err = SX1276_read_reg(spi, LREG_OPMODE,&val,1);

// switch mode to LoRa
val = val & 0xF8;		// sleep mode
spi_err = SX1276_write_reg(spi, LREG_OPMODE,&val,1);

spi_err = SX1276_read_reg(spi, LREG_OPMODE,&val,1);
val = (val & 0xF8) | 0x80;		// LoRa mode
spi_err = SX1276_write_reg(spi, LREG_OPMODE,&val,1);
return err;	
}


unsigned char SX1276_set_tx(spi_device_handle_t spi)
{
unsigned char err = 0;
esp_err_t spi_err = 0;
unsigned char val;

spi_err = SX1276_read_reg(spi, LREG_OPMODE,&val,1);

val = (val & 0xF8) | 0x03;		// tx mode
spi_err = SX1276_write_reg(spi, LREG_OPMODE,&val,1);

return err;	
}

unsigned char SX1276_set_rx(spi_device_handle_t spi)
{
unsigned char err = 0;
esp_err_t spi_err = 0;
unsigned char val;

spi_err = SX1276_read_reg(spi, LREG_OPMODE,&val,1);

val = (val & 0xF8) |  0x05;	// Rx_Continuous mode
spi_err = SX1276_write_reg(spi, LREG_OPMODE,&val,1);


return err;	
}

unsigned char SX1276_is_txmode(spi_device_handle_t spi)
{
unsigned char ret = 0;
esp_err_t spi_err = 0;
unsigned char val;

// check in LoRa mode...
spi_err = SX1276_read_reg(spi, LREG_OPMODE,&val,1);

if ((val & 0x07) == 0x03)
	ret = 1;
return ret;	
}

unsigned char SX1276_get_mode(spi_device_handle_t spi, unsigned char *mode)
{
unsigned char err = 0;
esp_err_t spi_err = 0;

spi_err = SX1276_read_reg(spi, LREG_OPMODE,mode,1);
*mode = *mode & 0x07;

return err;	
}

unsigned char SX1276_set_mode(spi_device_handle_t spi, unsigned char *mode)
{
unsigned char val;
unsigned char err = 0;
esp_err_t spi_err = 0;

spi_err = SX1276_read_reg(spi, LREG_OPMODE,&val,1);
*mode = (val & 0xF8) | *mode;
spi_err = SX1276_write_reg(spi, LREG_OPMODE,mode,1);

return err;	
}

unsigned char SX1276_tx_packet_start(spi_device_handle_t spi)
{
// resets SX1276 buffer pointers; loads packet header in to SX1276 payload register
unsigned char ret = 0;
esp_err_t spi_err = 0;
unsigned char c;

if ((lora_tx_packet_open) || (SX1276_is_txmode(spi)))
	{
	ret = 1;
	}
else
	{
	SX1276_set_idle(spi);
// reset Tx FIFO 
	c = 0;
//	spi_err = SX1276_write_reg(spi, LREG_FIFO_TX_BASE_ADDR,&c,1);
	spi_err = SX1276_read_reg(spi, LREG_FIFO_TX_BASE_ADDR,&c,1);
	spi_err = SX1276_write_reg(spi, LREG_FIFO_ADDR_PTR,&c,1);	
	
	lora_tx_packet_open = 1;

	}
	
return ret;
}

unsigned char SX1276_tx_packet_write(spi_device_handle_t spi, unsigned char c)
{
// writes a byte to SX1276 payload register
unsigned char ret = 0;
esp_err_t spi_err = 0;

if (lora_tx_packet_open)
	spi_err = SX1276_write_reg(spi, LREG_FIFO_TX_BASE_ADDR,&c,1);
else
	ret = 1;

return ret;
}

unsigned char SX1276_tx_packet_block_write(spi_device_handle_t spi, unsigned char *data, unsigned char len)
{
// writes a block of bytes to SX1276 payload register
unsigned char ret = 0;
esp_err_t spi_err = 0;
unsigned char i;

if (lora_tx_packet_open)
	{
	for (i=0;i<len;i++)
		{
		spi_err = SX1276_write_reg(spi, LREG_FIFO_TX_BASE_ADDR,&data[i],1);
		}
	}
else
	ret = 1;
	
return ret;
}


unsigned char SX1276_tx_packet_end(spi_device_handle_t spi, unsigned char *data, unsigned char len)
{
// loads packet tail into SXC1276 payload register; sets Tx mode; sends packet; sets rx mode
unsigned char ret = 0;
esp_err_t spi_err = 0;

lora_tx_packet_open = 0;

// put into Tx mode
spi_err = SX1276_set_tx(spi);	

return ret;
}

void SX1276_calibrate_rx(spi_device_handle_t spi)
{
	// can only be done in FSK mode as regs are only accessible there...
esp_err_t spi_err = 0;
unsigned char x;

	if(debug_do(DBG_LORA))
		{
		printf("LoRa Calibration begin...\n");
		}
		
// cal lo band
   SX1276_set_freq(spi, 433000000, 1);		// Sets a Frequency in LF band
	
//	SX1276Write( REG_IMAGECAL, ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
// start the calibration
spi_err = SX1276_read_reg(spi, REG_CAL_RX, &x,1);
x = x | CAL_RX_START;
spi_err = SX1276_write_reg(spi, REG_CAL_RX, &x,1);

// wait for calibration to finish
x = CAL_RX_RUNNING;
while(x == CAL_RX_RUNNING)	// ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
	spi_err = SX1276_read_reg(spi, REG_CAL_RX, &x,1);
	x = x & CAL_RX_RUNNING;
    }


// cal hi band	
    SX1276_set_freq(spi, 868000000, 1);		// Sets a Frequency in HF band

// start the calibration
spi_err = SX1276_read_reg(spi, REG_CAL_RX, &x,1);
x = x | CAL_RX_START;
spi_err = SX1276_write_reg(spi, REG_CAL_RX, &x,1);

// wait for calibration to finish
x = CAL_RX_RUNNING;
while(x == CAL_RX_RUNNING)	// ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
	spi_err = SX1276_read_reg(spi, REG_CAL_RX, &x,1);
	x = x & CAL_RX_RUNNING;
    }

	if(debug_do(DBG_LORA))
		{
		printf("LoRa Calibration end\n");
		}

}

void SX1276_get_regs(spi_device_handle_t spi, unsigned char start, unsigned char end)
{
unsigned char i,x;
esp_err_t spi_err = 0;

printf("Lora registers %d - %d:\n",start,end);
printf("    ");
for (i=0;i<16;i++)
	{
	printf("%02X ",i);
	}
printf("\n");

x = start%16;
if (x)
	{
	printf("%02X: ",x&0xF0);
	
	for (i=0;i<x;i++)
		{
		printf("   ");
		}
	}
for (i=start;i<=end;i++)
	{
	spi_err = SX1276_read_reg(spi, i, &x,1);
	if (i%16==0)
		{
		if (i)
			printf("\n");

		printf("%02X: ",i&0xF0);
		}

	printf("%02X ",x);
	}

printf("\n\n");
}

void SX1276_get_dio_map(spi_device_handle_t spi, unsigned char *dio1_map, unsigned char *dio2_map)
{
unsigned char state,curr_opmode;
esp_err_t spi_err = 0;

// place in FSK mode to access DIO regs
spi_err = SX1276_read_reg(spi, LREG_OPMODE, &state,1);
curr_opmode = state;
state = state & 0x7F;			

spi_err = SX1276_write_reg(spi, LREG_OPMODE, &state,1);

// read DIO regs
spi_err = SX1276_read_reg(spi, LREG_DIO_MAP_1, dio1_map,1);
spi_err = SX1276_read_reg(spi, LREG_DIO_MAP_2, dio2_map,1);

// set back in LoRa mode if required
if (curr_opmode & 0x80)		// if previously in LoRa mode
	{
	spi_err = SX1276_write_reg(spi, LREG_OPMODE, &curr_opmode,1);	
	}		
}

void SX1276_set_dio_map(spi_device_handle_t spi, unsigned char dio1_map, unsigned char dio2_map)
{
unsigned char state,curr_opmode;
esp_err_t spi_err = 0;

// place in FSK mode to access DIO regs
spi_err = SX1276_read_reg(spi, LREG_OPMODE, &state,1);
curr_opmode = state;
state = state & 0x7F;			

spi_err = SX1276_write_reg(spi, LREG_OPMODE, &state,1);

// write DIO regs as necessary
if (dio1_map != SX1276_dio1_map)
	{
	spi_err = SX1276_write_reg(spi, LREG_DIO_MAP_1,&dio1_map,1);
	}
	
if (dio2_map != SX1276_dio2_map)
	{
	spi_err = SX1276_write_reg(spi, LREG_DIO_MAP_2,&dio2_map,1);
	}

// set back in LoRa mode if required
if (curr_opmode & 0x80)		// if previously in LoRa mode
	{
	spi_err = SX1276_write_reg(spi, LREG_OPMODE, &curr_opmode,1);	
	}		

}

signed char SX1276_get_rssi(spi_device_handle_t spi)
{
unsigned char x;
signed char rssi;

esp_err_t spi_err = 0;

spi_err = SX1276_read_reg(spi, LREG_PKT_RSSI_VAL, &x,1);

rssi = -157 + x;
	
return rssi;
}

signed int SX1276_get_tx_power(spi_device_handle_t spi)
{
// returns SX1276 PA power * 10 (to allow for 1 dp accuracy)	

unsigned char val,pa_boost,pa_max,pa_index,i,wr_flag;
signed int pa_power;
			
esp_err_t err;
			
err = SX1276_read_reg(spi, LREG_PA_CFG,&val,1);

pa_boost = val>>7;
// following calcs use 10 * pa_max, pa_power values for 1 dp accuracy			
if (pa_boost)
	pa_max = 170;
else
	pa_max = 108 + 6*((val>>4) & 0x07);

pa_index = val & 0x0F;
pa_power = pa_max - 150 + (10*pa_index);
	
return pa_power;
}