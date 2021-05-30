#include <uartcom.h>

uint8_t ack_send(uint8_t msg, uint8_t * txbuf)
{
	uint16_t crc_value;

	txbuf[0] = START;
	txbuf[1] = 0 + OFF_ASCII;
	txbuf[2] = 2 + OFF_ASCII; //ASCII '2'
	txbuf[3] = ACK;
	txbuf[4] = msg;

	crc_value = CRC16_buf(txbuf,5);

	txbuf[5] = (crc_value >> 8) & 0xFF;			//split crc value into 2x8bit
	txbuf[6] = (crc_value >> 0) & 0xFF;

	return 7;									//length


}


uint8_t er_send(uint8_t er_code,uint8_t * txbuf)
{
	uint16_t crc_value;

	txbuf[0] = START;
	txbuf[1] = 0 + OFF_ASCII;
	txbuf[2] = 1 + OFF_ASCII; //ASCII '1'
	txbuf[3] = ERROR;
	txbuf[4] = er_code;

	crc_value = CRC16_buf(txbuf,5);

	txbuf[5] = (crc_value >> 8) & 0xFF;			//split crc value into 2x8bit
	txbuf[6] = (crc_value >> 0) & 0xFF;

	return 6;									//length


}

uint8_t an_send(uint16_t value,uint8_t * txbuf)
{
	uint8_t adc_send_buffer[5]; //maximum length is 5 because of 16bit adc_value
	uint8_t len = 0;
	uint16_t calc_length;
	uint16_t crc_value;

	calc_length = value;

	/*calculate number of digits*/
	while(calc_length!=0)
	{
		calc_length /= 10;
		len++;
	}

	sprintf(adc_send_buffer,"%d",value);

	txbuf[0] = START;
	txbuf[1] = 0 + OFF_ASCII;
	txbuf[2] = len + OFF_ASCII; //maximum length is 5, simple ascii conversion is enough
	for (uint8_t i = 0; i < len; i++)
	{
		txbuf[i+3] = adc_send_buffer[i];
	}
	crc_value = CRC16_buf(txbuf, len+3);

	txbuf[len+3] = (crc_value >> 8) & 0xFF;			//split crc value into 2x8bit
	txbuf[len+4] = (crc_value >> 0) & 0xFF;

	return len+5;									//return length
}

uint8_t stat_send(uint8_t * txbuf)
{
	uint16_t crc_value;

	txbuf[0] = START;
	txbuf[1] = 0 + OFF_ASCII;
	txbuf[2] = 3 + OFF_ASCII;
	txbuf[3] = BOARD;
	txbuf[4] = VERSION;
	txbuf[5] = SENSOR;
	crc_value = CRC16_buf(txbuf,6);
	txbuf[6] = (crc_value >> 8) & 0xFF;				//split crc value into 2x8bit
	txbuf[7] = (crc_value >> 0) & 0xFF;

	return 8;										//return length
}

uint8_t data_check(uint8_t *dat)
{
	uint8_t len = 0;
	uint16_t crc16_calc;
	if (dat[0]=='<')								//detect startbyte
	{
		len = (dat[1]-OFF_ASCII)*10 + (dat[2]-OFF_ASCII);			//ASCII to int
		//crc16 = (dat[len+OFF_CRC_0]<<8 | dat[len+OFF_CRC_1]);		//CRC values are the values after the message AND the first three bytes
		//crc16_calc = CRC16_buf(dat, len+3);
		crc16_calc = CRC16_buf(dat, len+OFF_TELE);
	}
	if (crc16_calc == 0)						//Check if crc is correct
	{
		return len;
	}
	else
	{
		return 0;									//CRC is wrong, return 0
	}


}

void tx_wait(DMA_STRUCT * dma)
{
	while(!dma->tx_flag);						//Wait for the tx flag to be set
	dma->tx_flag=0;								//Reset it
}
