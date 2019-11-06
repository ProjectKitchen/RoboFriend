/* 
 * File:   main.c
 * Authors: CV, Dr.Wummi
 *
 * This interfaces with the IoWarrior board of the Robofriend platform via the iowkit SDK. 
 * The IoWarrior board controls the power relais for the CRT TV screen,
 * the RGB LEDs for the ears and the camera servo
 *
 *
 *
 *  IOWARRIOR direct Write bytes 1 and 2 usage:
 *  ===========================================
 *
 *  iovalue1 (hex) .... bit 7-4: LED intensity red (bits are active low)\n");
 *                               bit 3  : 0= CRT TV power ON;  1 = CRT TV power off\n");
 *  iovalue2 (hex) .... bit 7-4: LED intensity green (bits are active low)\n");
 *                      bit 3-0: LED intensity blue (bits are active low)\n");
 *
 *
 *  Special Mode used for SD20 I2C / Servo control
 *
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


// fwd dec
uint WriteDirect(uint PipeNum, byte data[]);
byte SetServo(int ServoNr, byte Pos);

// globals
IOWKIT_HANDLE G_iowHandle;
byte G_ServoIC2Address = 0xC2;


int main(int argc, char** argv)
{

    unsigned char _data[3];
    unsigned short sn[9];
    char snt[9];
    int j;
    byte servoPos = 0;
    uint r=0,g=0,b=0;

    IOWKIT_HANDLE iowHandle = IowKitOpenDevice();	//Get first device handle which found
    G_iowHandle = iowHandle;





    if(iowHandle != NULL)
    {

        //Get serial number of device and output
        IowKitGetSerialNumber(iowHandle, sn);

        for (j = 0; j < 9; j++)
                snt[j] = sn[j];

        //printf("\n\n  IoWarrior: SerialNumber = %s\n", snt);

        if(argc == 1) {
            printf("  IoWarrior interface! \n  usage: sudo ./iow [<red> <green> <blue>] [<servo>]\n");
            printf("  red, green, blue:   light intensity fo LEDs (0-15)\n");
            printf("             servo:   position of camera servo (1 - 160)\n\n");
            printf("  examples: iow 100        : set servo to position 100, no color change\n");
            printf("            iow 10 10 0    : LEDs yellow, no servo change\n");
            printf("            iow 7 0 0 50   : LEDs red, servo position 50\n\n");

        }
        if(argc > 2 )
        {
            _data[0] = 0;

            r = strtoul(argv[1],NULL,10);
            g = strtoul(argv[2],NULL,10);
            b = strtoul(argv[3],NULL,10);

            _data[1] = ~((byte) (((r&0x0f)<<4)+8));
            //printf("IoWarrior: set I/O value1 %d\n",_data[1]);

            _data[2] = ~((byte) (((g&0x0f)<<4) + (b&0x0f)));
            //printf("IoWarrior: set I/O value2 %d\n",_data[2]);

            //   _data[1] = (byte) (_data[1] & 0xF7);
            //   _data[1] = (byte)(_data[1] | 0x08);

           WriteDirect(0, _data);
        }

        if ((argc == 2) || (argc==5))
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
                printf("IoWarrior: i2c init failed \n");
                IowKitCloseDevice(iowHandle);
                return(EXIT_FAILURE);
            }

            servoPos = strtoul(argv[argc-1],NULL,10);
            //printf("IoWarrior: Setting Servo Position: %d\n", servoPos);
            SetServo(1,servoPos);
        }

        IowKitCloseDevice(iowHandle);
    }
    else
    {
        printf("IoWarrior: Could not open interface (did you use sudo?)\n\n");
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
        if (Pos < 1) Pos = 1;
        if (Pos > 160) Pos = 160;
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

