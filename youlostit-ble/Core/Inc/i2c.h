/*
 * ic2.h
 *
 *  Created on: Jan 29, 2025
 *      Author: B
 */

#ifndef IC2_H_
#define IC2_H_
#include <stm32l475xx.h>


#endif /* IC2_H_ */


#define LSM6_ADDR 0x6A

void ic2_init();

uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len);
