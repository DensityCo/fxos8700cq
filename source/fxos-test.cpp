#include "fxos8700cq.h"
#include <iostream>

int main(int argc, char **argv)
{
    FXOS8700CQ fxos(0x01, "/dev/i2c-1");

    fxos.open();
    std::cout << "fxos opened." << std::endl;

    fxos.active();

    std::cout << "data before sensor read" << std::endl;
    std::cout << "----------------------------------------------------------------------------------------" << std::endl;   
    std::cout << "tempdata : " << fxos.tempData << std::endl;
    std::cout << "acceldata: " << fxos.accelData.x << "," << fxos.accelData.y << "," << fxos.accelData.z << std::endl;
    std::cout << "magdata  : " << fxos.magData.x << "," << fxos.magData.y << "," << fxos.magData.z << std::endl;

    fxos.readAccelData();
    fxos.readMagData();
    fxos.readTempData();

    std::cout << "data after sensor read" << std::endl;
    std::cout << "----------------------------------------------------------------------------------------" << std::endl;   
    std::cout << "tempdata : " << fxos.tempData << std::endl;
    std::cout << "acceldata: " << fxos.accelData.x << "," << fxos.accelData.y << "," << fxos.accelData.z << std::endl;
    std::cout << "magdata  : " << fxos.magData.x << "," << fxos.magData.y << "," << fxos.magData.z << std::endl;

    return 0;
}

