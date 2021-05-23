#include <stdint.h>
#ifdef ARDUINO
#include <Wire.h>
#else
#include <math.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include "fxos8700cq.h"
#include <iostream>
#include <sys/stat.h>
#include <fcntl.h>
#endif

// Public Methods //////////////////////////////////////////////////////////////

FXOS8700CQ::FXOS8700CQ(uint8_t addr, std::string device_path)
{
    path = device_path;
	address = addr;
	accelFSR = AFS_2g;     // Set the scale below either 2, 4 or 8
	accelODR = AODR_200HZ; // In hybrid mode, accel/mag data sample rates are half of this value
	magOSR = MOSR_5;     // Choose magnetometer oversample rate
}

FXOS8700CQ::FXOS8700CQ(uint8_t addr)
{
	address = addr;
	accelFSR = AFS_2g;     // Set the scale below either 2, 4 or 8
	accelODR = AODR_200HZ; // In hybrid mode, accel/mag data sample rates are half of this value
	magOSR = MOSR_5;     // Choose magnetometer oversample rate
}

FXOS8700CQ::FXOS8700CQ()
{
	accelFSR = AFS_2g;     // Set the scale below either 2, 4 or 8
	accelODR = AODR_200HZ; // In hybrid mode, accel/mag data sample rates are half of this value
	magOSR = MOSR_5;     // Choose magnetometer oversample rate
}

// Writes a register
//
bool FXOS8700CQ::writeReg(uint8_t reg, uint8_t value)
{
    bool result = true;
#ifdef ARDUINO
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
#else
    unsigned char outbuf[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[1];

    messages[0].addr  = address;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = outbuf;

    outbuf[0] = reg;
    outbuf[1] = value;

    packets.msgs  = messages;
    packets.nmsgs = 1;

    if(ioctl(file_handle, I2C_RDWR, &packets) < 0) 
    {
        result = true;
    }

#endif
    return result;
}

static int get_i2c_register(int file, unsigned char addr, unsigned char reg, unsigned char *val) {
    unsigned char inbuf, outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    outbuf = reg;
    messages[0].addr  = addr;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = &outbuf;

    messages[1].addr  = addr;
    messages[1].flags = I2C_M_RD;
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = &inbuf;

    packets.msgs      = messages;
    packets.nmsgs     = 2;
    if(ioctl(file, I2C_RDWR, &packets) < 0) {
        return 0;
    }
    *val = inbuf;

    return 1;
}

// Reads a register
bool FXOS8700CQ::readReg(uint8_t reg, uint8_t &value)
{
    bool result = false;
#ifdef ARDUINO
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(address, (uint8_t)1);
	value = Wire.read();
	Wire.endTransmission();
#else
    if(!get_i2c_register(file_handle, address, reg, &value)) 
    {
        std::cerr << "error reading register.  reg: " << reg << "address" << address << std::endl;
    }
    else
    {
        result = true;
    }
#endif
    return result;
}


bool FXOS8700CQ::readRegs(uint8_t reg, uint8_t count, uint8_t dest[])
{
    bool result = true;
#ifdef ARDUINO
	uint8_t i = 0;
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(reg);            	   // Put slave register address in Tx buffer
	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, count);  // Read uint8_ts from slave register address 

	while (Wire.available()) 
    {
		dest[i++] = Wire.read();   // Put read results in the Rx buffer
	}
#else
    unsigned char outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    outbuf = reg;
    messages[0].addr  = address;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = &outbuf;

    messages[1].addr  = address;
    messages[1].flags = I2C_M_RD;
    messages[1].len   = count;
    messages[1].buf   = dest;

    packets.msgs      = messages;
    packets.nmsgs     = 2;
    if(ioctl(file_handle, I2C_RDWR, &packets) < 0) 
    {
        result = false;
    }
    return result;
#endif
}

// Read the accelerometer data
void FXOS8700CQ::readAccelData()
{
	uint8_t rawData[6];  // x/y/z accel register data stored here

	readRegs(FXOS8700CQ_OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
	accelData.x = ((int16_t) rawData[0] << 8 | rawData[1]) >> 2;
	accelData.y = ((int16_t) rawData[2] << 8 | rawData[3]) >> 2;
	accelData.z = ((int16_t) rawData[4] << 8 | rawData[5]) >> 2;
}

// Read the magnometer data
void FXOS8700CQ::readMagData()
{
	uint8_t rawData[6];  // x/y/z accel register data stored here

	readRegs(FXOS8700CQ_M_OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
	magData.x = ((int16_t) rawData[0] << 8 | rawData[1]) >> 2;
	magData.y = ((int16_t) rawData[2] << 8 | rawData[3]) >> 2;
	magData.z = ((int16_t) rawData[4] << 8 | rawData[5]) >> 2;
}

// Read the temperature data
void FXOS8700CQ::readTempData()
{
    uint8_t t;

	if (!readReg(FXOS8700CQ_TEMP, t))
    {
        std::cerr << "error reading temperature data";
    }
    else
    {
        tempData = (int8_t)t;
    }
}

// Put the FXOS8700CQ into standby mode.
// It must be in standby for modifying most register
void FXOS8700CQ::standby()
{
    uint8_t c;

	if (readReg(FXOS8700CQ_CTRL_REG1, c))
    {
	    writeReg(FXOS8700CQ_CTRL_REG1, c & ~(0x01));
    }
    else
    {
        std::cerr << "error switching to standby mode" << std::endl;
    }
}

// Put the FXOS8700CQ into active mode.
// Needs to be in this mode to output data.
void FXOS8700CQ::active()
{
    uint8_t c;

	if (readReg(FXOS8700CQ_CTRL_REG1, c))
    {
	    writeReg(FXOS8700CQ_CTRL_REG1, c | 0x01);
    }
    else
    {
        std::cerr << "error switching to active mode" << std::endl;
    }
}

bool FXOS8700CQ::open_sensor()
{
    bool result = false;

    file_handle = open(path.c_str(), O_RDWR); 

    if (file_handle >= 0)
    {
    standby();  // Must be in standby to change registers

	// Configure the accelerometer
	writeReg(FXOS8700CQ_XYZ_DATA_CFG, accelFSR);  // Choose the full scale range to 2, 4, or 8 g.

	if (accelODR <= 7) 
    {
        uint8_t temp_value;
        if (readReg(FXOS8700CQ_CTRL_REG1, temp_value))
        {
		    writeReg(FXOS8700CQ_CTRL_REG1,  temp_value | (accelODR << 3));
        }
        else
        {
            std::cerr << "error opening fxos8700cq acclerometer" << std::endl;
        }
    }

	// Configure the magnetometer
	writeReg(FXOS8700CQ_M_CTRL_REG1, 0x80 | magOSR << 2 | 0x03); // Set auto-calibration, set oversampling, enable hybrid mode 
		                                     
	active();  // Set to active to start reading
    result = true;
    }

    return result;
}

// Get accelerometer resolution
float FXOS8700CQ::getAres(void)
{
	switch (accelFSR)
	{
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 gs (00), 4 gs (01), 8 gs (10). 
		case AFS_2g:
			return 2.0/8192.0;
		break;
		case AFS_4g:
			return 4.0/8192.0;
		break;
		case AFS_8g:
			return 8.0/8192.0;
		break;
	}

	return 0.0;
}

// Get magnometer resolution
float FXOS8700CQ::getMres(void)
{
	return 10./32768.;
}
// Private Methods //////////////////////////////////////////////////////////////
