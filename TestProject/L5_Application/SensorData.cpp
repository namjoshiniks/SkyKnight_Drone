/*
 * SensorData.cpp/
 *
 *  Created on: Apr 1, 2017
 *      Author: Admin
 */
#include "SensorData.hpp"

char reg_address; //command register
char dev_address; //device address
uint8_t dataMSB;
uint8_t dataLSB;
uint16_t data;

I2C2& i2c2 = I2C2::getInstance();


void initSensor(volatile float &yawAngleInitial)
{
	dev_address = 0x50;
	reg_address = 0x3D;
	bool flag3 = i2c2.writeReg(dev_address,reg_address,0x1C);
	dev_address = 0x51;
	reg_address = 0x1B;
	dataMSB = i2c2.readReg(dev_address,reg_address);
	reg_address = 0x1A;
	dataLSB = i2c2.readReg(dev_address,reg_address);
	data = ((dataMSB & 0xFFFF) << 8) | dataLSB;
	yawAngleInitial = (float)data/15.9972;
}


void get_pitch(volatile float &pitchAngle)
{

	dev_address = 0x51;

	reg_address = 0x1F;
	dataMSB = i2c2.readReg(dev_address,reg_address);
	reg_address = 0x1E;
	dataLSB = i2c2.readReg(dev_address,reg_address);
	data = ((dataMSB & 0xFFFF) << 8) | dataLSB;
	pitchAngle = (float)data/7.96;
	if(pitchAngle > 8000)
		pitchAngle = -(8232 - pitchAngle);
	pitchAngle = pitchAngle - 1.5;
	printf("Pitch %0.2f\n",pitchAngle);
}

void get_roll(volatile float &rollAngle)
{
	dev_address = 0x51;
	reg_address = 0x1D;
	dataMSB = i2c2.readReg(dev_address,reg_address);
	reg_address = 0x1C;
	dataLSB = i2c2.readReg(dev_address,reg_address);
	data = ((dataMSB & 0xFFFF) << 8) | dataLSB;
	if(data > 60000)
		rollAngle = (65536 - (float)data)/15.89;
	else
		rollAngle = -1 * (float)data/15.89;
	rollAngle = rollAngle - 0.2;
	//printf("Roll %0.2f\n",rollAngle);
}

void get_yaw(volatile float &yawAngle,volatile float &yawAngleInitial)
{
	dev_address = 0x51;
	reg_address = 0x1B;
	dataMSB = i2c2.readReg(dev_address,reg_address);
	reg_address = 0x1A;
	dataLSB = i2c2.readReg(dev_address,reg_address);
	data = ((dataMSB & 0xFFFF) << 8) | dataLSB;
	yawAngle = (float)data/15.9972;
	if(yawAngle < yawAngleInitial)
		yawAngle = 360 + yawAngle - yawAngleInitial;
	else
		yawAngle = yawAngle - yawAngleInitial;
	if(yawAngle > 180)
		yawAngle = yawAngle - 360;
	//printf("Yaw %0.2f\n",float(data));
	//printf("Yaw %0.2f\n",yawAngle);
}




