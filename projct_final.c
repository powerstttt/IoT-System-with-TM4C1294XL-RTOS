/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
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
 * This project is created for EEM 449 project.
 * It uses semaphores. There is also another project with
 * events which is project_final_event.
 * Name: Yunus Emre Esen
 * Student Id: 22280328940
 *
 */


/*
 *  ======== httpget.c ========
 *  HTTP Client GET example application
 */
    #include <string.h>
    #include <stdlib.h>

/* XDCtools Header files */
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* TI-RTOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Idle.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/GPIO.h>
#include <ti/net/http/httpcli.h>

#include <xdc/std.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/I2C.h>

/* Example/Board Header file */
#include "Board.h"

#include <sys/socket.h>

/* BMP 180den gelenler  */
#include "i2cbmp180.h"

#define Board_BMP180_ADDR 0x77
#define HOSTNAME          "api.openweathermap.org"
#define REQUEST_URI       "/data/2.5/forecast/?id=315202&APPID=bb8f49f514e12ad4e63ed9b05a8e5e1a"
#define HOSTNAME2         "eskisehir.mgm.gov.tr"
#define REQUEST_URI2      "/tahmin-gunluk.aspx"
#define USER_AGENT        "HTTPCli (ARM; TI-RTOS)"
#define SOCKETTEST_IP     "10.31.7.23"
#define TASKSTACKSIZE 4096

extern Semaphore_Handle semaphore0;
extern Semaphore_Handle sensorSem;
extern Semaphore_Handle httpSem;
extern Semaphore_Handle socketSem;
extern Semaphore_Handle mgmSem;

char   tempstr[20]; //openweather temperature
char   tempstr_tmp[20]; //tmp006 temperature
char   tempstr_bmp[20]; //bmp180 pressure
char   tempstr_http[20]; //MGM temperature
char   printString[1024];

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];
/*
 *  ======== printError ========
 */
void printError(char *errString, int code)
{
    System_printf("Error! code = %d, desc = %s\n", code, errString);
    BIOS_exit(code);
}

void sendData2Server(char *serverIP, int serverPort, char *data, int size)
{
    int sockfd;
    struct sockaddr_in serverAddr;

    sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfd == -1) {
        System_printf("Socket not created");
        BIOS_exit(-1);
    }

    memset(&serverAddr, 0, sizeof(serverAddr));  /* clear serverAddr structure */
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort);     /* convert port # to network order */
    inet_pton(AF_INET, serverIP, &(serverAddr.sin_addr));

    int connStat = connect(sockfd, (struct sockaddr *)&serverAddr, /* connecting….*/
                  sizeof(serverAddr));
    if(connStat < 0) {
        System_printf("Error while connecting to server\n");
        if (sockfd > 0)
            close(sockfd);
        BIOS_exit(-1);
    }

    int numSend = send(sockfd, data, size, 0);       /* send data to the server*/
    if(numSend < 0) {
        System_printf("Error while sending data to server\n");
        if (sockfd > 0) close(sockfd);
        BIOS_exit(-1);
    }

    if (sockfd > 0) close(sockfd);
}

Void socketTask(UArg arg0, UArg arg1)
{
    while(1) {
        // wait for the semaphore that httpTask() will signal
        // when temperature string is retrieved from api.openweathermap.org site
        //
        Semaphore_pend(socketSem, BIOS_WAIT_FOREVER);

        GPIO_write(Board_LED0, 1); // turn on the LED

        //put all variables in printString
        strcpy(printString, "Openweather: ");
        strcat(printString,tempstr);
        strcat(printString,"\nMGM: ");
        strcat(printString,tempstr_http);
        strcat(printString,"\nTMP006: ");
        strcat(printString,tempstr_tmp);
        strcat(printString,"\nBMP180: ");
        strcat(printString,tempstr_bmp);
        // connect to SocketTest program on the system with given IP/port
        sendData2Server(SOCKETTEST_IP, 5011, printString, strlen(printString));

        GPIO_write(Board_LED0, 0);  // turn off the LED
        /* Turn off sequence LED */
        GPIO_write(Board_LED1, Board_LED_OFF);
    }
}

/*
 *  ======== httpTask ========
 *  Makes a HTTP GET request
 */
Void httpTask(UArg arg0, UArg arg1)
{
    bool moreFlag = false;
    char data[64], *s1, *s2;
    int ret, temp_received=0, len;
    struct sockaddr_in addr;

    HTTPCli_Struct cli;
    HTTPCli_Field fields[3] = {
        { HTTPStd_FIELD_NAME_HOST, HOSTNAME },
        { HTTPStd_FIELD_NAME_USER_AGENT, USER_AGENT },
        { NULL, NULL }
    };
    while(1) {
        Semaphore_pend(httpSem, BIOS_WAIT_FOREVER);
        System_printf("Sending a HTTP GET request to '%s'\n", HOSTNAME);
        System_flush();

        HTTPCli_construct(&cli);

        HTTPCli_setRequestFields(&cli, fields);

        ret = HTTPCli_initSockAddr((struct sockaddr *)&addr, HOSTNAME, 0);
        if (ret < 0) {
            printError("httpTask: address resolution failed", ret);
        }

        ret = HTTPCli_connect(&cli, (struct sockaddr *)&addr, 0, NULL);
        if (ret < 0) {
            printError("httpTask: connect failed", ret);
        }

        ret = HTTPCli_sendRequest(&cli, HTTPStd_GET, REQUEST_URI, false);
        if (ret < 0) {
            printError("httpTask: send failed", ret);
        }

        ret = HTTPCli_getResponseStatus(&cli);
        if (ret != HTTPStd_OK) {
            printError("httpTask: cannot get status", ret);
        }

        System_printf("HTTP Response Status Code: %d\n", ret);

        ret = HTTPCli_getResponseField(&cli, data, sizeof(data), &moreFlag);
        if (ret != HTTPCli_FIELD_ID_END) {
            printError("httpTask: response field processing failed", ret);
        }

        len = 0;
        do {
            ret = HTTPCli_readResponseBody(&cli, data, sizeof(data), &moreFlag);
            if (ret < 0) {
                printError("httpTask: response body processing failed", ret);
            }
            else {
                // string is read correctly
                // find "temp:" string
                //
                s1=strstr(data, "temp");
                if(s1) {
                    if(temp_received) continue;     // temperature is retrieved before, continue
                    // is s1 is not null i.e. "temp" string is found
                    // search for comma
                    s2=strstr(s1, ",");
                    if(s2) {
                        *s2=0;                      // put end of string
                        strcpy(tempstr, s1+6);      // copy the string
                        temp_received = 1;
                    }
                }
            }

            len += ret;     // update the total string length received so far
        } while (moreFlag);

        System_printf("Recieved %d bytes of payload\n", len);
        System_printf("Temperature %s\n", tempstr);
        System_flush();                                         // write logs to console

        HTTPCli_disconnect(&cli);                               // disconnect from openweathermap

        Semaphore_post(mgmSem);                             // activate mgm httpTask2

        Task_sleep(1000);                                       // sleep 1 seconds
        HTTPCli_destruct(&cli);

    }


}

//http GET from eskisehir.mgm.gov.tr
Void httpTask2(UArg arg0, UArg arg1)
{
    bool moreFlag = false;
    char data[64], *s1, *s2;
    int ret, temp_received=0, len;
    struct sockaddr_in addr;

    HTTPCli_Struct cli;
    HTTPCli_Field fields[3] = {
        { HTTPStd_FIELD_NAME_HOST, HOSTNAME2 },
        { HTTPStd_FIELD_NAME_USER_AGENT, USER_AGENT },
        { NULL, NULL }
    };
    while(1) {
        Semaphore_pend(mgmSem, BIOS_WAIT_FOREVER);
        System_printf("Sending a HTTP GET request to '%s'\n", HOSTNAME2);
        System_flush();

        HTTPCli_construct(&cli);

        HTTPCli_setRequestFields(&cli, fields);

        ret = HTTPCli_initSockAddr((struct sockaddr *)&addr, HOSTNAME2, 0);
        if (ret < 0) {
            printError("httpTask: address resolution failed", ret);
        }

        ret = HTTPCli_connect(&cli, (struct sockaddr *)&addr, 0, NULL);
        if (ret < 0) {
            printError("httpTask: connect failed", ret);
        }

        ret = HTTPCli_sendRequest(&cli, HTTPStd_GET, REQUEST_URI2, false);
        if (ret < 0) {
            printError("httpTask: send failed", ret);
        }

        ret = HTTPCli_getResponseStatus(&cli);
        if (ret != HTTPStd_OK) {
            printError("httpTask: cannot get status", ret);
        }

        System_printf("HTTP Response Status Code: %d\n", ret);

        ret = HTTPCli_getResponseField(&cli, data, sizeof(data), &moreFlag);
        if (ret != HTTPCli_FIELD_ID_END) {
            printError("httpTask: response field processing failed", ret);
        }

        len = 0;
        do {
            ret = HTTPCli_readResponseBody(&cli, data, sizeof(data), &moreFlag);
            if (ret < 0) {
                printError("httpTask: response body processing failed", ret);
            }
            else {
                // string is read correctly
                // find "renkMax" string
                //
                s1=strstr(data, "renkMax");
                if(s1) {
                    if(temp_received) continue;     // temperature is retrieved before, continue
                    //find "<\" to reach temperature value
                    s2=strstr(s1, "</");
                    if(s2) {
                        *s2=0;                      // put end of string
                        strcpy(tempstr_http, s1+17);      // copy the string
                        temp_received = 1;

                    }
                }
            }

            len += ret;     // update the total string length received so far
        } while (moreFlag);

        System_printf("Recieved %d bytes of payload\n", len);
        System_printf("Temperature %s (C)\n\n", tempstr_http);
        System_flush();                                         // write logs to console

        HTTPCli_disconnect(&cli);                               // disconnect from mgm

        Semaphore_post(socketSem);                              // activate socketTask

        Task_sleep(1000);                                       // sleep 1 seconds
        HTTPCli_destruct(&cli);
    }

}


//TMP006 sensor
Void tmpTask(UArg arg0, UArg arg1)
{
    while(1){
        Semaphore_pend(semaphore0, BIOS_WAIT_FOREVER); // wait for the semaphore
        uint16_t        temperature;
        uint8_t         txBuffer[6];
        uint8_t         rxBuffer[6];
        I2C_Handle      i2c;
        I2C_Params      i2cParams;
        I2C_Transaction i2cTransaction;

        // Create I2C interface for sensor usage
        //
        I2C_Params_init(&i2cParams);
        i2cParams.bitRate = I2C_400kHz;  // It can be I2C_400kHz orI2C_100kHz

        // Let's open the I2C interface
        //
        i2c = I2C_open(Board_I2C_TMP, &i2cParams);  // Board_I2C_TMP is actually I2C7
        if (i2c == NULL) {
            // error initializing IIC
            //
            System_abort("Error Initializing I2C\n");
        }

        //System_printf("I2C Initialised!\n");

        // Point to the T ambient register and read its 2 bytes (actually 14 bits)
        // register number is 0x01.
        //
        txBuffer[0] = 0x01;                                 // Ambient temperature register
        i2cTransaction.slaveAddress = Board_TMP006_ADDR;    // For SENSHUB it is 0x41
        i2cTransaction.writeBuf = txBuffer;                 // transmit buffer
        i2cTransaction.writeCount = 1;                      // only one byte will be sent
        i2cTransaction.readBuf = rxBuffer;                  // receive buffer
        i2cTransaction.readCount = 2;                       // we are expecting 2 bytes

        // carry out the I2C transfer. The received 16 bits is in big endian format since IIC
        // protocol sends the most significant byte first (i.e. rxBuffer[0]) and then
        // least significant byte (i.e. rxBuffer[1]).
        //
        // Remember that temperature register is 14 bits and we need to shift right 2 bits
        // to get a number. We need to divide it by 32 to get the temperature value.
        //
        if (I2C_transfer(i2c, &i2cTransaction)) {

           // 14 bit to 16 bit conversion since least 2 bits are 0s
            //
           temperature = (rxBuffer[0] << 6) | (rxBuffer[1] >> 2);

           // This time we are going to check whether the number is negative or not.
           // If it is negative, we will sign extend to 16 bits.
           //
           if (rxBuffer[0] & 0x80) {
               temperature |= 0xF000;
           }

           // We need to divide by 32 to get the actual temperature value.
           // Check with the TMP006 datasheet
           //
           temperature /= 32;
           System_printf("Anlik sicaklik: %d (C)\n", temperature);
           sprintf(tempstr_tmp, " %d", temperature);
        }
        else {

            // no response from TMP006. Is it there?
            //
           System_printf("I2C Bus fault\n");
        }

        // flush everything to the console
        //
        System_flush();

        // close the interface
        //
        I2C_close(i2c);
        Semaphore_post(sensorSem);
    }
       System_printf("Olcme islemi bitirildi.");
}

Void bmpTask(UArg arg0, UArg arg1){
    while(1){
        Semaphore_pend(sensorSem, BIOS_WAIT_FOREVER);
        float temp, press, alt;
        uint32_t pressure;
        // initialize I2C interface
        //
        initializeI2C();

        // get pressure calibration data
        //
        BMP180_getPressureCalibration();

        // start temperature acquisition
        //
        BMP180_startTemperatureAcquisition();

        // wait for 5 mseconds for the acquisition
        //
        Task_sleep(5);

        // get the uncompensated temperature value
        //
        temp = BMP180_getTemperature();

        // start pressure acquisition
        //
        BMP180_startPressureAcquisition();

        // wait for 5 mseconds for the acquisition
        //
        Task_sleep(5);

        // get the uncompensated pressure value
        // The sea level pressure is 101325 pascal
        //
        press = BMP180_getPressure();

        // get the altitude
        //
        alt = BMP180_calculateAltitude(press);

        // Close I2C connection
        //
        closeI2C();
        pressure = (int)press;
        ltoa(pressure, tempstr_bmp); //int to string (global variable)

        System_printf("Pressure: %d\n", pressure);
        Semaphore_post(httpSem);
        System_flush();
    }
}

void netIPAddrHook(unsigned int IPAddr, unsigned int IfIdx, unsigned int fAdd)
{
       static Task_Handle taskHandle1, taskHandle2;
       Task_Params taskParams;
       Error_Block eb;

       // Create a HTTP task when the IP address is added
       if (fAdd && !taskHandle1 && !taskHandle2) {
          Error_init(&eb);

       Task_Params_init(&taskParams);
       taskParams.stackSize = TASKSTACKSIZE;
       taskParams.priority = 1;
       taskHandle1 = Task_create((Task_FuncPtr)httpTask, &taskParams, &eb);

       Task_Params_init(&taskParams);
       taskParams.stackSize = TASKSTACKSIZE;
       taskParams.priority = 1;
       taskHandle1 = Task_create((Task_FuncPtr)httpTask2, &taskParams, &eb);

       Task_Params_init(&taskParams);
       taskParams.stackSize = TASKSTACKSIZE;
       taskParams.priority = 1;
       taskHandle2 = Task_create((Task_FuncPtr)socketTask, &taskParams, &eb);



       if (taskHandle1 == NULL || taskHandle2 == NULL) {
           printError("netIPAddrHook: Failed to create HTTP and Socket Tasks\n", -1);
       }
   }
}

Void clockFunc(void){
    /* Turn on sequence LED */
    GPIO_write(Board_LED1, Board_LED_ON);
    Semaphore_post(semaphore0);
}
/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initI2C();
    Board_initEMAC();


    System_printf("Starting EEM449 project.\n");
    System_flush();


    /* Start BIOS */
    BIOS_start();

    return (0);
}
