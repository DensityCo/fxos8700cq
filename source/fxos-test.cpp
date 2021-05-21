#include "fxos8700cq.h"
#include <iostream>
#include <unistd.h>

int main(int argc, char **argv)
{
    FXOS8700CQ fxos(0x1c, "/dev/i2c-2");

    fxos.open_sensor();
    std::cout << "fxos opened." << std::endl;

    fxos.active();

    while(true)
    {
        sleep(0.1);
        fxos.readAccelData();
//    fxos.readMagData();
//    fxos.readTempData();

//    std::cout << "tempdata : " << fxos.tempData << std::endl;
        std::cout << "acceldata: " << fxos.accelData.x << "," << fxos.accelData.y << "," << fxos.accelData.z << std::endl;
//    std::cout << "magdata  : " << fxos.magData.x << "," << fxos.magData.y << "," << fxos.magData.z << std::endl;
    }
    return 0;
}

