#ifndef __UARTCOM_H
#define __UARTCOM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "crc16ccitt.h"
#include "main.h"
#include <stdio.h>
#include <stdint.h>




#define START '<'
/*TX functions*/

/*Send acknowledge for current message*/
uint8_t ack_send(uint8_t msg,uint8_t * txbuf);

/*Send the value of the sensor*/
uint8_t an_send(uint16_t value,uint8_t * txbuf); //send analog value

/*Send the current status of the device (Version)*/
uint8_t stat_send(uint8_t * txbuf);


uint8_t dat_send(uint8_t * txbuf); //send data


/*Send an error code (#defines in main.h) and write the message into the tx buffer*/
uint8_t er_send(uint8_t er_code,uint8_t * txbuf); //send error

/*Wait for the UART TX to be free*/
void tx_wait(DMA_STRUCT * dma);


/*RX functions*/

/*CRC check the integrity of the data - returns the length if successful, 0 if not  */
uint8_t data_check(uint8_t * dat);



#ifdef __cplusplus
}
#endif

#endif
