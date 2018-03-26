#ifndef _BLUE1_H_
#define _BLUE1_H_

/*******************************************************************************
*                                 INCLUDES
*******************************************************************************/
//#include "mbed.h"
#include <events/mbed_events.h>

#include <mbed.h>
#include "ble/BLE.h"
#include "ble/Gap.h"
//#include "ButtonService.h"
#include "LSM6DS3.h"


/*******************************************************************************
*                                 DEFINES
*******************************************************************************/
#define BLE_SENSOR_VERSION_STRING "1.0.0"
#define UPDATE_CONN_PARAM 0 // Can be set to 1 only when no low power mode is used
#define ADV_INTERVAL_MIN_MS  1000
#define ADV_INTERVAL_MAX_MS  1200


/*******************************************************************************
*                                     DEBUG
*******************************************************************************/
#ifndef DEBUG
#define DEBUG 1
#endif
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


/*******************************************************************************
*                               CLASS BLUE1
*******************************************************************************/
class Blue1{

private:
    /* board section */
    Serial *serialport;
    DigitalOut *led1;
    DigitalOut *led2;
    DigitalOut *led3;
    DigitalIn *but1;
    DigitalIn *but2;
    SPI *spi; // mosi, miso, sclk
    /* imu section */
    char IMU_register(char, char);
    /* ble section */
    uint8_t ret;
    //stackInit
    //sensorDeviceInit
    uint8_t Sensor_DeviceInit(void);
    //appTick
    uint8_t request_free_fall_notify;


public:
    Blue1(Serial *, DigitalOut *, DigitalOut *, DigitalOut *, SPI *);
    ~Blue1();
    /* board section */
    void boardInit(void);
    /* imu section */
    void configIMU(void);
    float readTempIMU(void);
    void readAccIMU(int16_t *);
    void readGyroIMU(int16_t *);
    void printSensor(void);
    /* ble section */
    //stackInit
    void stackInit(void);
    //sensorDeviceInit
    #define SENSOR_TIMER 1
    static uint16_t acceleration_update_rate;
    static uint8_t sensorTimer_expired;
    void sensorDeviceInit(void);
    //setDeviceConnectable
    void setDeviceConnectable(void);
    //BlueNRG-1 Stack Callbacks: connection handlers
    static uint8_t set_connectable;
    static int connected;
    //btleStackTick
    void btleStackTick(void);
    //appTick
    void appTick(void);


}; // end class BLue1

#endif //_BLUE1_H_
