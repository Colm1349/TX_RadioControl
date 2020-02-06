#include "At_commander.hpp"




//////////////////


void At_commander::switchCommandMode() { //переключиться из или в коммандный режим (+++)
  Port.print("+");
  delay(10);
  Port.print("+");
  delay(10);
  Port.print("+");
  delay(10);
}

void At_commander::enterCommandMode() { //переключиться в командный режим
  // check current mode
  if (CommandMode_Flag == 0 )
  {
    switchCommandMode();
    if(Serial2.available() > 0)
    {
      for (int i = 0; i < (sizeof(recieve_buff)/sizeof(recieve_buff[0]) ) ; i++)
      {
         recieve_buff[i]=Serial2.read();
      }
    }
  }
  if (Serial2.read() == 13){
    Serial.println("Entered command mode");
    return;
  } else {
    Serial.println("Left command mode, switching back in...");
    switchCommandMode();
    Serial.println("Entered command mode");
  }
}

void At_commander::leaveCommandMode() { //выйти, лучше следить за статусом командного режима и делать enterCommandMode и потом switchCommandMode
  switchCommandMode();
  if (Serial2.read() == -1){
    Serial.println("Left command mode");
    return;
  } else {
    Serial.println("Entered command mode, getting out...");
    switchCommandMode();
    Serial.println("Left command mode");
  }
}

void At_commander::sendCommand(char* command) {
  Serial.print("Sending command: ");
  Serial.println(command);
  delay(50);
  Serial2.println(command);
}

void At_commander::readSerialLine() {
  int incomingByte = 0;
  int i = 0;
  if (Serial2.available() > 0) {
       Serial.println("~I received: ");
       while (Serial2.available() > 0) {  //если есть доступные данные
          // считываем байт
          incomingByte = Serial2.read();
          // отсылаем то, что получили
          Serial.write(incomingByte);
          Serial.print(" i ==");
          Serial.print(i);
          Serial.print(" Byte ==");
          Serial.print(incomingByte);
          Serial.println();
          i++;
          // Serial.print(incomingByte);
          //Serial.write(10);
        }
        Serial.println();
        Serial.println("~");
  }
}
