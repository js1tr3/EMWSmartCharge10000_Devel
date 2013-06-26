/*
uLCD_144 Arduino Library

Copyright (c) 2011 Valery Miftakhov.  All right reserved.
Version 1.0, April 8, 2011

See README.txt for usage & more info
*/


//==================================== SCREEN FUNCTION LIBRARY ========================
#ifndef uLCD_144_h
#define uLCD_144_h

// without this include, nothing works 
#include "WProgram.h"

class uLCD_144
{
  private:
    void waitAck();
  public:
    uLCD_144(int);
    byte getMSB(byte, byte, byte);
    byte getLSB(byte, byte, byte);
    void setContrast(int);
    void clrScreen();
    void setBgColor(byte, byte, byte);
    void setPenSize(int);
    void setFont(int);
    void setOpacity(int);
    void drawPixel(int, int, byte, byte, byte);
    void drawCircle(int, int, int, byte, byte, byte);
    void printStr(int, int, int, byte, byte, byte, const char*);
    void printGRStr(int, int, int, byte, byte, byte, int, int, const char*);
    void drawLine(int, int, int, int, byte, byte, byte);
    void drawRect(int, int, int, int, byte, byte, byte);
};
#endif
//================================ END SCREEN FUNCTION LIBRARY ========================
