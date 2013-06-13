/*
 * drv_spi.h
 *
 *  Created on: 29.03.2013
 *      Author: Serg
 */
#pragma once

void init_SPI1(void);
void init_SPI2(void);
uint8_t spi_readByte(void);
uint8_t spi_writeByte(uint8_t Data);
