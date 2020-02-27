/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *    ======== i2cbmp180.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include "Board.h"
#include <math.h>


#define Board_BMP180_ADDR 0x77

I2C_Handle      i2c;
I2C_Params      i2cParams;
I2C_Transaction i2cTransaction;

uint8_t         txBuffer[4];
uint8_t         rxBuffer[30];

short AC1, AC2, AC3, B1, B2, MB, MC, MD;    // calibration variables
unsigned short AC4, AC5, AC6;               // calibration variables
long UT, UP;    //uncompensated temperature and pressure
float B3, B4, B6, B7, X1t, X1p, X2t, X2p, X3p, B5t, B5p, Altitude;

void BMP180_getPressureCalibration(void)
{
    txBuffer[0] = 0xAA;
    i2cTransaction.slaveAddress = Board_BMP180_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 22;

    if (I2C_transfer(i2c, &i2cTransaction)) {
           //System_printf("Calibration data acquired\n");

           AC1 = rxBuffer[0]<<8 | rxBuffer[1];
           AC2 = rxBuffer[2]<<8 | rxBuffer[3];
           AC3 = rxBuffer[4]<<8 | rxBuffer[5];
           AC4 = rxBuffer[6]<<8 | rxBuffer[7];
           AC5 = rxBuffer[8]<<8 | rxBuffer[9];
           AC6 = rxBuffer[10]<<8 | rxBuffer[11];
           B1 = rxBuffer[12]<<8 | rxBuffer[13];
           B2 = rxBuffer[14]<<8 | rxBuffer[15];
           MB = rxBuffer[16]<<8 | rxBuffer[17];
           MC = rxBuffer[18]<<8 | rxBuffer[19];
           MD = rxBuffer[20]<<8 | rxBuffer[21];
    }
}

void BMP180_startTemperatureAcquisition(void)
{
    txBuffer[0] = 0xf4;                                 // control register
    txBuffer[1] = 0x2e;                                 // temperature conversion command
    i2cTransaction.slaveAddress = Board_BMP180_ADDR;    // 0x77
    i2cTransaction.writeBuf = txBuffer;                 // transmit buffer
    i2cTransaction.writeCount = 2;                      // two bytes will be sent
    i2cTransaction.readBuf = rxBuffer;                  // receive buffer
    i2cTransaction.readCount = 0;                       // we are expecting 2 bytes

    if (I2C_transfer(i2c, &i2cTransaction)) {
       //System_printf("Temperature acquisition initiated\n");
    }
}

void BMP180_startPressureAcquisition(void)
{
    txBuffer[0] = 0xf4;                                 // control register
    txBuffer[1] = 0x34;                                 // pressure conversion command
    i2cTransaction.slaveAddress = Board_BMP180_ADDR;    // 0x77
    i2cTransaction.writeBuf = txBuffer;                 // transmit buffer
    i2cTransaction.writeCount = 2;                      // two bytes will be sent
    i2cTransaction.readBuf = rxBuffer;                  // receive buffer
    i2cTransaction.readCount = 0;                       // we are expecting 2 bytes

    if (I2C_transfer(i2c, &i2cTransaction)) {
       //System_printf("Pressure acquisition initiated\n");
    }
}

float BMP180_getTemperature(void)
{
    float temp;

    txBuffer[0] = 0xf6;                                 // temperature register
    i2cTransaction.slaveAddress = Board_BMP180_ADDR;    // 0x77
    i2cTransaction.writeBuf = txBuffer;                 // transmit buffer
    i2cTransaction.writeCount = 1;                      // two bytes will be sent
    i2cTransaction.readBuf = rxBuffer;                  // receive buffer
    i2cTransaction.readCount = 2;                       // we are expecting 2 bytes

    if (I2C_transfer(i2c, &i2cTransaction)) {
       //System_printf("Temperature value acquired\n");
    }

    UT = rxBuffer[0]<<8 | rxBuffer[1];  //UT = raw temperature data
    //System_printf("Uncompansated Temperature : %d\n", UT);

    //compute temperature
    X1t = ((UT - AC6) * AC5) >> 15;
    X2t = (MC << 11) / (X1t + MD);
    B5t = X1t + X2t;
    temp = ((B5t + 8) / 16) / 10;

    return temp;
}

float BMP180_getPressure(void)
{
    float pressure;

    txBuffer[0] = 0xf6;                                 // temperature register
    i2cTransaction.slaveAddress = Board_BMP180_ADDR;    // 0x77
    i2cTransaction.writeBuf = txBuffer;                 // transmit buffer
    i2cTransaction.writeCount = 1;                      // two bytes will be sent
    i2cTransaction.readBuf = rxBuffer;                  // receive buffer
    i2cTransaction.readCount = 2;                       // we are expecting 2 bytes

    if (I2C_transfer(i2c, &i2cTransaction)) {
       //System_printf("Pressure value acquired\n");
    }

    UP = rxBuffer[0]<<8 | rxBuffer[1];  //UT = raw pressure data
    //System_printf("Uncompansated Pressure : %d\n", UP);

    //compute pressure
    B6 = B5t - 4000;
    X1p = (B2 * (B6 * B6 / 4096)) / 2048;
    X2p = AC2 * B6 / 2048;
    X3p = X1p = X2p;
    B3 = ((((long)AC1 * 4 + X3p)) + 2) / 4;
    X1p = AC3 * B6 / 8192;
    X2p = (B1 * (B6 * B6 / 4096)) / 65536;
    X3p = ((X1p + X2p) + 2) / 4;
    B4 = AC4 * (unsigned long)(X3p + 32768) / 32768;
    B7 = ((unsigned long)UP - B3) * (50000);
    if (B7 < 0x80000000) {
        pressure = (B7 * 2) / B4;
    }
    else {
        pressure = (B7 / B4) * 2;
    }
    X1p = (pressure / 256) * (pressure / 256);
    X1p = (X1p * 3038) / 65536;
    X2p = (-7357 * pressure) / 65536;
    pressure = pressure + (X1p + X2p + 3791) / 16;

    return pressure;
}

float BMP180_calculateAltitude(float pressure)
{
    float alt;

    // compute altitude; uses default sea level pressure; altitude will vary
    // depending on difference between default sea level pressure (101325 Pascal)
    // and the actual pressure
    //
    alt = 44330.0f * (1.0f - powf(pressure / 101325.0f, 1 / 5.255f));

    //System_printf("Altitude calculated\n");
    return alt;
}

void initializeI2C()
{
    // Create I2C interface for sensor usage
    //
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_100kHz;  // It can be I2C_400kHz orI2C_100kHz

    // Let's open the I2C interface
    //
    i2c = I2C_open(Board_I2C0, &i2cParams);  // actually I2C7
    if (i2c == NULL) {
        // error initializing IIC
        //
        System_abort("Error Initializing I2C\n");
    }

    //System_printf("I2C Initialized!\n");
}

void closeI2C(void)
{
    // close the interface
    //
    I2C_close(i2c);

    //System_printf("I2C interface closed\n");
}

