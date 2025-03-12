/*
 * lsm6dsl.c
 *
 *  Created on: Jan 31, 2025
 *      Author: Kaiji
 */


#include "lsm6dsl.h"

#define CTRL1_XL 0x10
#define CTRL6_C 0x15
#define INT1_CTRL 0x0D

#define OUTX_L_XL 0x28
#define OUTX_H_XL 0x29
#define OUTY_L_XL 0x2A
#define OUTY_H_XL 0x2B
#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D



void lsm6dsl_init(){

	uint8_t operating_frequency = 0x30;
	uint8_t interrupt_ctrl = 0x1;

	uint8_t result;

	//set to low Power Mode
	uint8_t operating_mode = 0x10;

	//get the current CTRL6_C
	uint8_t ctrl6_c;


	result = i2c_transaction(CTRL6_C, 0x1, &ctrl6_c, 1);
	printf("original ctrl6_c=0x%x\n", ctrl6_c);


	ctrl6_c |= operating_mode;
	result = i2c_transaction(CTRL6_C, 0x0, &ctrl6_c, 1);
	printf("new ctrl6_c=0x%x\n", ctrl6_c);


	result = i2c_transaction(CTRL1_XL, 0x0, &operating_frequency, 1);


	result = i2c_transaction(INT1_CTRL, 0x0, &interrupt_ctrl, 1);

}



void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z) {

	uint8_t result;

	//AccelX
	result = i2c_transaction(OUTX_L_XL, 0x1, &((uint8_t*)x)[0], 1);

	result = i2c_transaction(OUTX_H_XL, 0x1, &((uint8_t*)x)[1], 1);



	//AccelY
	result = i2c_transaction(OUTY_L_XL, 0x1, &((uint8_t*)y)[0], 1);

	result = i2c_transaction(OUTY_H_XL, 0x1, &((uint8_t*)y)[1], 1);



	//AccelZ
	result = i2c_transaction(OUTZ_L_XL, 0x1, &((uint8_t*)z)[0], 1);

	result = i2c_transaction(OUTZ_H_XL, 0x1, &((uint8_t*)z)[1], 1);


}
