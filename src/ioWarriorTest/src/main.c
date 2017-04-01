/* 
 * File:   main.c
 * Author: user
 *
 * Created on July 16, 2015, 7:20 AM
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <math.h>


#include "iowkit.h"

#define FALSE 0
#define TRUE 1
#define byte unsigned char


/*
 *  fucking c buttfuckery
 */

//fwd dec
uint WriteDirect(uint PipeNum, byte data[]);
byte SetServo(int ServoNr, byte Pos);

//globals
IOWKIT_HANDLE G_iowHandle;
byte G_ServoIC2Address = 0xC2;


int main(int argc, char** argv) 
{
   
    unsigned char _data[3];
    unsigned short sn[9];
    char snt[9];
    int j;
    byte servoPos = 0;
    int mon = 1;

    IOWKIT_HANDLE iowHandle = IowKitOpenDevice();	//Get first device handle which found
    G_iowHandle = iowHandle;


  


    if(iowHandle != NULL)
    {
        
        if(argc > 1)
        {
            //init I2C
            byte data[IOWKIT_SPECIAL_REPORT_SIZE];

            data[0]     = 0x01; //report id 1 => I2C Mode
            data[1]     = 0x01; //Enable
            data[2]     = 0x00; //Flags
            data[3]     = 0x00; //Timeout         

            int ret = IowKitWrite(G_iowHandle, IOW_PIPE_SPECIAL_MODE, data, IOWKIT_SPECIAL_REPORT_SIZE);
        
            if(ret == 0)
            {
                printf("i2c init failed \n");
                IowKitCloseDevice(iowHandle);
                return(EXIT_FAILURE);
                
            }
            
            servoPos = strtoul(argv[1],NULL,10);
            SetServo(1,servoPos);
        }

        _data[0] = 0;
        _data[1] = 0xff;
        _data[2] = 0xff;

        //Get serial number of device and output
        IowKitGetSerialNumber(iowHandle, sn);

        for (j = 0; j < 9; j++)
                snt[j] = sn[j];

        printf("S/N: %s\n", snt);

        if(mon) 
        {
            _data[1] = (byte) (_data[1] & 0xF7);
        }
        else
        {
            _data[1] = (byte)(_data[1] | 0x08);
        }

        WriteDirect(0, _data);

        IowKitCloseDevice(iowHandle);

    }
    else
    {
        printf("iow not found :( \n\n");
    }



    return (EXIT_SUCCESS);
}


uint WriteDirect(uint PipeNum, byte data[])
{
    
    return IowKitWrite(G_iowHandle, PipeNum, data, IOWKIT24_IO_REPORT_SIZE); // IOWKIT24_IO_REPORT_SIZE = 3

}

byte SetServo(int ServoNr, byte Pos)
{
    if(ServoNr == 1) // Physikalische Anschläge nicht überfahren
    {
        if (Pos < 95) Pos = 95;
        if (Pos > 246) Pos = 246;
    }

    //send servo
    byte data[IOWKIT_SPECIAL_REPORT_SIZE];
    byte dataR[IOWKIT_SPECIAL_REPORT_SIZE];

    data[0] = 0x02;              //I2C Write
    data[1] = 0xC3;              //Start&Stop / 3 Bytes Payload
    data[2] = G_ServoIC2Address; //Address
    data[3] = (byte)ServoNr;     //Servo1  
    data[4] = Pos;               //pos 0 


    IowKitWrite(G_iowHandle, IOW_PIPE_SPECIAL_MODE, data, IOWKIT_SPECIAL_REPORT_SIZE);

    IowKitRead(G_iowHandle, 1, dataR, IOWKIT_SPECIAL_REPORT_SIZE);
    return dataR[1];
}

