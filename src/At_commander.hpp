#pragma once
#include <Arduino.h>

class At_commander
{
   public:
    At_commander(Stream *SerialPort) { _stream = SerialPort; _stream.println("IZI") };
    void enterCommandMode();
    void switchCommandMode();
    void leaveCommandMode();
    void sendCommand(char* command); 
    void readSerialLine();

   private:
    Stream* _stream = nullptr;

};
