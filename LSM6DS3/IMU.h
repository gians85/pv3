#ifndef _IMU_H_
#define _IMU_H_

/*******************************************************************************
*                                 INCLUDES
*******************************************************************************/
#include <mbed.h>
#include "LSM6DS3.h"


/*******************************************************************************
*                               CLASS IMU
*******************************************************************************/
class IMU{

private:
    SPI *spi;
    Serial *serialport;
    char IMU_register(char, char);


public:
    IMU(Serial *, SPI *);
    ~IMU();
    /* imu section */
    void configIMU(void);
    float readTempIMU(void);
    void readAccIMU(int16_t *);
    void readGyroIMU(int16_t *);
    void printSensor(void);

}; // end class

#endif //_IMU_H_
