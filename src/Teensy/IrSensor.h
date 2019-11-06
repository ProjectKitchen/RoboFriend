/*
 * @file      Motor.cpp
 * @version   v10.0
 * @date      01.01.20xx
 * @changed   07.03.2019
 * @author    cveigl, mzahedi
 * @brief     DESCRIPTION
 */

#ifndef IRSENSOR_
#define IRSENSOR_

/*
#define ircam 0x58
#define ircam_W 0xB0
#define ircam_R 0xB1
*/
struct IrToken
{
  int x;
  int y;
  bool valid;
  int tokenSize;
};

class IrCam{

  public:
    IrCam();
    byte  read_ir();
    void init_ir ();
    IrToken token1;
  IrToken token2;
  IrToken token3;
  IrToken token4;
  
  private:
  void Write_2bytes(byte d1, byte d2);
  
};

#endif // ODOMETRY_H
