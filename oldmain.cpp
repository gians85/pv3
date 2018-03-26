#ifdef oldmain
#warning oldmain
#include "Blue1.h"


Serial serialport(USBTX, USBRX);
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
SPI spi(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_CS); // mosi, miso, sclk

Blue1 blue1(&serialport, &led1, &led2, &led3, &spi);

int oldmain() {

    /* board init */
    blue1.boardInit();

    /* BlueNRG-1 stack init */
    blue1.stackInit();

    /* Device Init */
    blue1.sensorDeviceInit();

    while(1) {
        /* BLE Stack Tick */
        blue1.btleStackTick();

        /* Application Tick */
        blue1.appTick();

        /* Power Save management */
        sleep();
    }

}


#endif //oldmain