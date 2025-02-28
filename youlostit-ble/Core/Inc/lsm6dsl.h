/*
 * lsm6dsl.h
 *
 *  Created on: Jan 31, 2025
 *      Author: Kaiji
 */

#ifndef LSM6DSL_H_
#define LSM6DSL_H_

#endif /* LSM6DSL_H_ */


#include "i2c.h"

void lsm6dsl_init();

void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z);
