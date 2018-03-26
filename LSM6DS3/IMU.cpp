#include "IMU.h"


/******************************************************************************
* Function Name  : IMU
* Description    : Constructor
* Input          :
* Return         :
******************************************************************************/
IMU::IMU(Serial *serialPtr, SPI *spiPtr){
    serialport = serialPtr;
	spi  = spiPtr;
}

/******************************************************************************
* Function Name  : ~IMU
* Description    : Distructor
* Input          :
* Return         :
******************************************************************************/
IMU::~IMU(){}

/******************************************************************************
* Function Name  : configIMU
* Description    : config inertial sensor
* Input          :
* Return         :
******************************************************************************/
void IMU::configIMU(){
	IMU_register(0x11, 0x12);
	IMU_register(0x19, 0x38);
	IMU_register(0x10, 0x10);
	IMU_register(0x18, 0x38);
}

/******************************************************************************
* Function Name  : IMU_register
* Description    : access to imu register
* Input          : register name, data to write
* Return         : read data
******************************************************************************/
char IMU::IMU_register(char reg_name, char data){
	char buf[2];
	if (data == READ){
		reg_name |= READ_1B;
		spi->write(&reg_name, 1, &data, 1);
	}
	else{
		buf[0] = reg_name;
		buf[1] = data;
		spi->write(buf, 2, &data, 0);
	}
	return data;
}

/******************************************************************************
* Function Name  : readTempIMU
* Description    : read IMU temperature
* Input          : none
* Return         : temperature in °C
******************************************************************************/
float IMU::readTempIMU(){
	int16_t temp = 0;
	temp  = IMU::IMU_register(OUT_TEMP_H, READ);
	temp <<= 8;
	temp |= IMU::IMU_register(OUT_TEMP_L, READ);
	return (25.0+(((float)temp)/16));
}

/******************************************************************************
* Function Name  : readAccIMU
* Description    : read IMU temperature
* Input          : none
* Return         : temperature in °C
******************************************************************************/
void IMU::readAccIMU(int16_t * buf){
	buf[0]  = IMU::IMU_register(OUT_X_H_A, READ);
	buf[0] <<= 8;
	buf[0] |= IMU::IMU_register(OUT_X_L_A, READ);
	buf[1]  = IMU::IMU_register(OUT_Y_H_A, READ);
	buf[1] <<= 8;
	buf[1] |= IMU::IMU_register(OUT_Y_L_A, READ);
	buf[2]  = IMU::IMU_register(OUT_Z_H_A, READ);
	buf[2] <<= 8;
	buf[2] |= IMU::IMU_register(OUT_Z_L_A, READ);
}

/******************************************************************************
* Function Name  : readGyroIMU
* Description    : read IMU temperature
* Input          : none
* Return         : temperature in °C
******************************************************************************/
void IMU::readGyroIMU(int16_t * buf){
	buf[0]  = IMU::IMU_register(OUT_X_H_G, READ);
	buf[0] <<= 8;
	buf[0] |= IMU::IMU_register(OUT_X_L_G, READ);
	buf[1]  = IMU::IMU_register(OUT_Y_H_G, READ);
	buf[1] <<= 8;
	buf[1] |= IMU::IMU_register(OUT_Y_L_G, READ);
	buf[2]  = IMU::IMU_register(OUT_Z_H_G, READ);
	buf[2] <<= 8;
	buf[2] |= IMU::IMU_register(OUT_Z_L_G, READ);
}

/******************************************************************************
* Function Name  : printSensor
* Description    : Print sensor values on serial port.
* Input          : None.
* Return         : None.
******************************************************************************/
void IMU::printSensor(){
    int16_t buf[3];
    float a = 0.000061, g = 0.004375;
    // acc_sensor
    IMU::readAccIMU(buf);
	serialport->printf(" ACC(xyz): %.4f  %.4f  %.4f\r\n", buf[0]*a, buf[1]*a, buf[2]*a);
    // gyro_sensor
    IMU::readGyroIMU(buf);
	serialport->printf("GYRO(xyz): %.4f  %.4f  %.4f\r\n", buf[0]*g, buf[1]*g, buf[2]*g);
    // temp_sensor
    serialport->printf("TEMP [°C]: %.1f \r\n", IMU::readTempIMU());
}

