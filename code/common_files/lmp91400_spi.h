/*
 * lmp91400.h
 *
 *  Created on: Nov 21, 2013
 *      Author: a0415888
 */

#ifndef LMP91400_H_
#define LMP91400_H_
typedef unsigned char   uint8_t;
void TI_LMP91400_SPIWriteReg(uint16_t addr, uint16_t data);
uint16_t TI_LMP91400_SPIReadReg(uint16_t addr);
void spi_xmit(Uint16 a);
void spi_fifo_init(void);
void spi_init(void);




#endif /* LMP91400_H_ */
