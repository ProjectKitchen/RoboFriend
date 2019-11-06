#include <PVision.h>

#include "IrSensor.h"
#include <Wire.h>
#include <Arduino.h>



    IrCam::IrCam()
    {
      
    }
    /****************************************************** Init IRCam*/
    void IrCam::init_ir (void) {
      //simple init
      /************
        write_loop_twi (ircam_W, 0x30, 0x01); //control register
        write_loop_twi (ircam_W, 0x06, 0x90); //MAXSIZE register
        write_loop_twi (ircam_W, 0x08, 0xC0); //GAIN register
        write_loop_twi (ircam_W, 0x1A, 0x40); //GAINLIMIT register
        write_loop_twi (ircam_W, 0x33, 0x03); //MODE register
        write_loop_twi (ircam_W, 0x30, 0x08); //control register

      ***********************************/
      Wire.begin();
      Wire.beginTransmission(0x58);
      Wire.endTransmission();

      Write_2bytes(0x30, 0x01); delay(10);
      Write_2bytes(0x30, 0x08); delay(10);
      Write_2bytes(0x06, 0x90); delay(10);
      Write_2bytes(0x08, 0xC0); delay(10);
      Write_2bytes(0x1A, 0x40); delay(10);
      Write_2bytes(0x33, 0x33); delay(10);
      delay(100);

    }

    byte  IrCam::read_ir( )
    {
      byte data_buf[16];
      byte returnByte;
      Wire.beginTransmission(0x58);
      Wire.write(0x36);
      Wire.endTransmission();

      Wire.requestFrom(0x58, 16);        // Request the 2 byte heading (MSB comes first)
      for (int i = 0; i < 16; i++)
      {
        data_buf[i] = 0;
      }

      int i = 0;
      while (Wire.available() && i < 16)
      {
        data_buf[i] = Wire.read();
        returnByte += data_buf[i];
        i++;
      }

      int buf;

      token1.x = data_buf[1];
      token1.y = data_buf[2];
      buf = data_buf[3];
      token1.x += (buf & 0x30) <<4;
      token1.y += (buf & 0xC0) <<2;
      token1.tokenSize = (buf & 0x0F);
      token1.valid=false;
      if(token1.tokenSize != 15)
        token1.valid=true;

      token2.x = data_buf[4];
      token2.y = data_buf[5];
      buf = data_buf[6];
      token2.x += (buf & 0x30) <<4;
      token2.y += (buf & 0xC0) <<2;
      token2.tokenSize = (buf & 0x0F);
      token2.valid=false;
      if(token2.tokenSize != 15)
        token2.valid=true;

      token3.x = data_buf[7];
      token3.y = data_buf[8];
      buf = data_buf[9];
      token3.x += (buf & 0x30) <<4;
      token3.y += (buf & 0xC0) <<2;
      token3.tokenSize = (buf & 0x0F);
      token3.valid=false;
      if(token3.tokenSize != 15)
        token3.valid=true;

      token4.x = data_buf[10];
      token4.y = data_buf[11];
      buf = data_buf[12];
      token4.x += (buf & 0x30) <<4;
      token4.y += (buf & 0xC0) <<2;
      token4.tokenSize = (buf & 0x0F);
      token4.valid=false;
      if(token4.tokenSize != 15)
        token4.valid=true;
      
      return returnByte;
    }

    void IrCam::Write_2bytes(byte d1, byte d2)
    {
      Wire.beginTransmission(0x58);
      Wire.write(d1);
      Wire.write(d2);
      Wire.endTransmission();
    }
