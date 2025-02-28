/*
 * i2c.c
 *
 *  Created on: Jan 29, 2025
 *      Author: Kaiji
 */
#include "i2c.h"

#define SCLDEL 0x4 << I2C_TIMINGR_SCLDEL_Pos
#define SDADEL 0x2 << I2C_TIMINGR_SDADEL_Pos


#define SCLL 0x13 << I2C_TIMINGR_SCLL_Pos
#define SCLH 0xF << I2C_TIMINGR_SCLH_Pos

// T_scll in ns
#define T_SCLL 5000

//APB clock period = 1/4Mhz
#define T_I2CCLK 250



enum ErrorCode{

	TRANSACTION_SUCCESS = 0,
	INVALID_ADDR,
	INVALID_DIR,
	NACK_RECEIVED,
	PREMATURE_COMPLETE


};


/* normal mode = 100khz
 *
 * PRESC = 1
 *
 *
 * */

void ic2_init(){

	/*---------Initial setup begin--------------*/
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;

	// PB10 and PB11 (set to alternate function mode 10)
	GPIOB->MODER &= ~GPIO_MODER_MODE10;
	GPIOB->MODER |= GPIO_MODER_MODE10_1;
	GPIOB->MODER &= ~GPIO_MODER_MODE11;
	GPIOB->MODER |= GPIO_MODER_MODE11_1;

	//

	//Configure OTYPER to Push Pull (0)
	GPIOB->OTYPER |= GPIO_OTYPER_OT10;
	GPIOB->OTYPER |= GPIO_OTYPER_OT11;

	// configure Alternate function (AF4 = 4b'0100) for PB10 and PB11
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL10_2;
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL11_2;

	// Set the I/O configuration to pull up (default to high)
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD10_0;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD11_0;
	/*---------Initial setup end----------------*/

	/*----------I2C initialization begin--------------*/
	//clear the PE bit
	I2C2->CR1 &= ~I2C_CR1_PE;


	I2C2->TIMINGR |= SCLDEL;
	I2C2->TIMINGR |= SDADEL;

	I2C2->TIMINGR |= SCLL;
	I2C2->TIMINGR |= SCLH;

	uint32_t PRESC;

	//T_SCLL  = (SCLL + 1)/t_PRESC
	//t_PRESC = (PRESC + 1)*t_i2cclk
	PRESC = T_SCLL / ((SCLL >> I2C_TIMINGR_SCLL_Pos) + 1) / T_I2CCLK - 1;

	I2C2->TIMINGR |= (PRESC << I2C_TIMINGR_PRESC_Pos);


	//set the pe bit
	I2C2->CR1 |= I2C_CR1_PE;
	/*----------I2C initialization end-------------------*/



//	uint32_t scldel = (I2C2->TIMINGR & I2C_TIMINGR_SCLDEL_Msk) >> I2C_TIMINGR_SCLDEL_Pos;
//	uint32_t sdadel = (I2C2->TIMINGR & I2C_TIMINGR_SDADEL_Msk) >> I2C_TIMINGR_SDADEL_Pos;
//
//	uint32_t scll = (I2C2->TIMINGR & I2C_TIMINGR_SCLL_Msk) >> I2C_TIMINGR_SCLL_Pos;
//	uint32_t sclh = (I2C2->TIMINGR & I2C_TIMINGR_SCLH_Msk) >> I2C_TIMINGR_SCLH_Pos;
//	PRESC = (I2C2->TIMINGR & I2C_TIMINGR_PRESC_Msk) >> I2C_TIMINGR_PRESC_Pos;
//	printf("scldel: %u; sdadel: %u; SCLL: %u; SCLH: %u; PRESC: %u\n", scldel, sdadel, scll, sclh, PRESC);


}


/*
 * @brief a single transaction from the master to the currently selected slave (7 bit Slave address is a global var as implicit first parameter)
 *
 * @param address: 1 byte I2C register Address
 *
 * @param dir: transaction direction: 0 for write, 1 for read
 *
 * @param len: size of data to transfer
 *
 * @return 0 for success, otherwise check ErrorCode
 *
 *
 *
 */


uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len){

	//clear slave address
	I2C2->CR2 &= ~I2C_CR2_SADD_Msk;
	// set slave address
	I2C2->CR2 |= (LSM6_ADDR << (I2C_CR2_SADD_Pos+1));


	//clear transaction direction
	I2C2->CR2 &= ~I2C_CR2_RD_WRN;


	//last bit = W
	if (dir == 0x0) {

		//set transaction direction
		I2C2->CR2 |= (dir << I2C_CR2_RD_WRN_Pos);


		//clear and set NBYTES
		I2C2->CR2 &= ~I2C_CR2_NBYTES;
		I2C2->CR2 |= ((len + 0x1) << I2C_CR2_NBYTES_Pos);

		//Clear & Set automatic end mode
		I2C2->CR2 &= ~I2C_CR2_AUTOEND;
		I2C2->CR2 |= I2C_CR2_AUTOEND;

		//Set the START bit
		I2C2->CR2 |= I2C_CR2_START;

		uint8_t bytesWritten = 0;

		//Send Register Address
		while ((I2C2->ISR & I2C_ISR_TXIS) == 0) {

			if (I2C2->ISR & I2C_ISR_NACKF) {

				I2C2->ICR |= I2C_ICR_NACKCF;
				return NACK_RECEIVED;
			}


		}

		I2C2->TXDR = address;


		while (bytesWritten < len){

			while ((I2C2->ISR & I2C_ISR_TXIS) == 0) {

				if (I2C2->ISR & I2C_ISR_NACKF) {

					I2C2->ICR |= I2C_ICR_NACKCF;
					return NACK_RECEIVED;
				}


			}


			//printf("data[bytesWritten]:0x%x\n", data[bytesWritten]);


			I2C2->TXDR = data[bytesWritten];

			bytesWritten++;

		}


	}
	//last bit = R
	else if (dir == 0x1) {

		uint8_t bytesReceived = 0;

		//set transaction direction
		I2C2->CR2 |= (0x0 << I2C_CR2_RD_WRN_Pos);

		//clear and set NBYTES to 1 for  setting register address
		I2C2->CR2 &= ~I2C_CR2_NBYTES;
		I2C2->CR2 |= (0x1 << I2C_CR2_NBYTES_Pos);

		//Clear automatic end mode
		I2C2->CR2 &= ~I2C_CR2_AUTOEND;


		//Set the START bit
		I2C2->CR2 |= I2C_CR2_START;


		//Send Register Address
		while ((I2C2->ISR & I2C_ISR_TXIS) == 0) {

			if (I2C2->ISR & I2C_ISR_NACKF) {

				I2C2->ICR |= I2C_ICR_NACKCF;
				return NACK_RECEIVED;
			}


		}

		I2C2->TXDR = address;

		//set transaction direction
		I2C2->CR2 |= (dir << I2C_CR2_RD_WRN_Pos);

		//set auto end mode
		I2C2->CR2 |= I2C_CR2_AUTOEND;

		//set NBYTES to len
		I2C2->CR2 |= (len << I2C_CR2_NBYTES_Pos);

		//Set the START bit
		I2C2->CR2 |= I2C_CR2_START;

		while (bytesReceived < len)
		{

			//Check the RXNE flag: if RXNE = 0x1: new data is ready
			while (!(I2C2->ISR & I2C_ISR_RXNE));

			//Read data from RXDR
			data[bytesReceived] = I2C2->RXDR;
			bytesReceived++;

		}

	}
	else
		return INVALID_DIR;



	return TRANSACTION_SUCCESS;


}
