/*
 * Bn0.c
 *
 *  Created on: Jul 26, 2019
 *      Author: infl3
 */
#include "bn0.h"
#include "satellite.h"
#include "stm32f4xx_hal_i2c.h"
I2C_HandleTypeDef * phi2c = 0;
int dataReady = 0;
uint8_t BNO055_ID = 0xA0;
void bnO_init(I2C_HandleTypeDef * i2c_handle, bnO_OPModes mode){
	phi2c = i2c_handle;
	volatile uint8_t id = 0;
	id = bnO_read(BNO055_CHIP_ID_ADDR);
	if(id != BNO055_ID){
		HAL_Delay(300);
		id = bnO_read(BNO055_CHIP_ID_ADDR);
		if(id != BNO055_ID){
			//handle error
		}
	}
	bnO_setMode(OPERATION_MODE_CONFIG);
	bnO_write(BNO055_SYS_TRIGGER_ADDR, 0x20);
	HAL_Delay(50);
	bnO_setPWRMode(POWER_MODE_NORMAL);
	HAL_Delay(10);

	bnO_write(BNO055_PAGE_ID_ADDR, 0x0);

	bnO_write(BNO055_SYS_TRIGGER_ADDR, 0x0);
	HAL_Delay(10);
	bnO_setMode(mode);
	HAL_Delay(100);
}

int bnO_write(bnO_Registers reg, uint8_t pValue){
	while(HAL_I2C_IsDeviceReady(phi2c, bnO_Address,1,0xff)!= HAL_OK){
		HAL_Delay(50);
	}
	if(HAL_I2C_Mem_Write(phi2c, bnO_Address, reg,sizeof(reg), &pValue, sizeof(pValue),0xff) != HAL_OK){
		//error handling
		for(;;);
		return 0;
	}
	return 1;
}

signed char bnO_read(bnO_Registers reg){
	uint8_t dataBuffer = 0;
	if(HAL_I2C_Mem_Read(phi2c, bnO_Address, reg, sizeof(reg), &dataBuffer, sizeof(uint8_t),0xff) != HAL_OK){
		//error handling
		return 0;
	}
	return dataBuffer;
}

int bnO_readMulti(bnO_Registers reg, int length, uint8_t dataBuffer[]){
	while(HAL_I2C_IsDeviceReady(phi2c, bnO_Address,1,0xff)!= HAL_OK){
			HAL_Delay(50);
	}
	if(HAL_I2C_Mem_Read(phi2c, bnO_Address, reg, sizeof(reg),dataBuffer,length,0xff) != HAL_OK){
		//handle error
		return 0;
	}
	return 1;
}

int bnO_setMode(bnO_OPModes mode){
	if(bnO_write(BNO055_OPR_MODE_ADDR, mode) != 1){
		//hanlde error
		return 0;
	}
	HAL_Delay(50);
	return 1;
}

int bnO_setPWRMode(bnO_PWRModes mode){
	if(bnO_write(BNO055_PWR_MODE_ADDR, mode) != 1){
		//handle error
		return 0;
	}
	HAL_Delay(50);
	return 1;
}

uint8_t bnO_getTemp(){
	uint8_t temp = 0;
	temp = bnO_read(BNO055_TEMP_ADDR);
	return temp;
}

uint8_t bnO_getStatus(){
	uint8_t status = 0;
	status = bnO_read(BNO055_SYS_STAT_ADDR);
	return status;
}


void test_i2c(I2C_HandleTypeDef * i2c_handle){
	phi2c = i2c_handle;
	uint8_t data[2];
	volatile char status;
 	status = HAL_I2C_Master_Receive(phi2c,8<<1,data,2,0xff);
 	volatile int x = 0;
}

calib_Status getCalibrationStatus(){
	calib_Status Status;
	unsigned char calib = 0;
	calib = bnO_read(BNO055_CALIB_STAT_ADDR);
	Status.SYS = (calib & 0b11000000) >> 6; //SYS
	Status.GYR = (calib & 0b00110000) >> 4; //GYR
	Status.ACC = (calib & 0b00001100) >> 2; //ACC
	Status.MAG = (calib & 0b00000011); //MAG
	return Status;
}

vector getAccelVector(){
	volatile vector Accel;
	volatile char LSB[3];
	volatile char MSB[3];
	volatile int i = 0;
	for(i = 0; i < 3; i++){
		LSB[i] = bnO_read(BNO055_ACCEL_DATA_X_LSB_ADDR+(i*2));
		MSB[i] = bnO_read(BNO055_ACCEL_DATA_X_MSB_ADDR+(i*2));
	}
	Accel.Z = (float)twosCompliment((MSB[0]<<8)|(LSB[0]))/100.0;
	Accel.X = (float)twosCompliment((MSB[1]<<8)|(LSB[1]))/100.0;
	Accel.Y = (float)twosCompliment((MSB[2]<<8)|(LSB[2]))/100.0;


	return Accel;
}

vector getGyroVector(){
	vector Gyro;
	char LSB[3];
	char MSB[3];
	for(int i = 0; i < 3; i++){
		LSB[i] = bnO_read(BNO055_GYRO_DATA_X_LSB_ADDR+(i*2));
		MSB[i] = bnO_read(BNO055_GYRO_DATA_X_MSB_ADDR+(i*2));
	}
	Gyro.X = (float)((MSB[0]<<8) | (LSB[0]))/900.0;
	Gyro.Y = (float)((MSB[1]<<8) | (LSB[1]))/900.0;
	Gyro.Z = (float)((MSB[2]<<8) | (LSB[2]))/900.0;
	return Gyro;
}


vector getMagVector(){
	vector Mag;
	char LSB[3];
	char MSB[3];
	for(int i = 0; i < 3; i++){
		LSB[i] = bnO_read(BNO055_MAG_DATA_X_LSB_ADDR+(i*2));
		MSB[i] = bnO_read(BNO055_MAG_DATA_X_MSB_ADDR+(i*2));
	}
	Mag.X = (float)twosCompliment((MSB[0]<<8) | (LSB[0]))/16.0;
	Mag.Y = (float)twosCompliment((MSB[1]<<8) | (LSB[1]))/16.0;
	Mag.Z = (float)twosCompliment((MSB[2]<<8) | (LSB[2]))/16.0;
	return Mag;
}

vector getGravVector(){
	vector Grav;
	volatile char LSB[3];
	volatile char MSB[3];
	for(int i = 0; i < 3; i++){
		LSB[i] = bnO_read(46+(i*2));
		MSB[i] = bnO_read(47+(i*2));
	}
	Grav.X = (float)twosCompliment((MSB[0]<<8) | (LSB[0]))/100.0;
	Grav.Y = (float)twosCompliment((MSB[1]<<8) | (LSB[1]))/100.0;
	Grav.Z = (float)twosCompliment((MSB[2]<<8) | (LSB[2]))/100.0;
	return Grav;
}

quaternion getQuatVector(){
	quaternion Quat;
	char LSB[4];
	char MSB[4];
	for(int i = 0; i < 4; i++){
		LSB[i] = bnO_read(BNO055_QUATERNION_DATA_X_LSB_ADDR+(i*2));
		MSB[i] = bnO_read(BNO055_QUATERNION_DATA_X_MSB_ADDR+(i*2));
	}
	Quat.W = (float)((MSB[0]<<8) | (LSB[0]))/16384.0;
	Quat.X = (float)((MSB[1]<<8) | (LSB[1]))/16384.0;
	Quat.Y = (float)((MSB[2]<<8) | (LSB[2]))/16384.0;
	Quat.Z = (float)((MSB[3]<<8) | (LSB[3]))/16384.0;
	return Quat;
}



