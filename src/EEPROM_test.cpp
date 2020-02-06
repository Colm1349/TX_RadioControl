#include <Arduino.h>
#include <EEPROM.h>

#define INIT_ADDR 1023  // номер резервной ячейки
#define INIT_KEY  88    // ключ первого запуска. 0-254, на выбор
#define CarrierFrequency_addr 1
#define defaultIndexOf_CarrierFrequency 0
#define Button_ChangeCarrierFrequency 8 

bool at_settings = false;
const unsigned long CarrierFrequency_Array [] = {868000000, 870000000, 880000000, 890000000, 900000000, 910000000, 915000000, 918000000, 920000000, 922000000};
uint8_t Channel = 0;


void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Button_ChangeCarrierFrequency, INPUT);
  Serial.begin(115200);
  Serial.println("Start");

  // my calc
  Serial.println("eepron(0xff)=" + String(EEPROM.read(INIT_ADDR)) + " before check");
  String X = "Xx12345678";
  Serial.println("S1ze X = " + String(sizeof(X)*X.length()) + " -> " + X );
  Serial.println("S1ze Channel = " + String(sizeof(Channel)) );
  char f = 'f';
  Serial.println("S1ze f = " + String(sizeof(f)) );

  Serial.println("Big array CarrierFrequency_Array size = " + String(sizeof(CarrierFrequency_Array)/sizeof(*CarrierFrequency_Array) * sizeof(CarrierFrequency_Array) ) );

  // put default val to INIT_ADDR
  if(EEPROM.read(INIT_ADDR) != INIT_KEY)
  {
    Serial.println("change EEPROM value");
    EEPROM.write(INIT_ADDR,INIT_KEY);
    EEPROM.put(CarrierFrequency_addr, defaultIndexOf_CarrierFrequency);
  }
  
  //check EEPROM for read Actual Frequency value 
  uint8_t temp = EEPROM.read(CarrierFrequency_addr);
  if (temp >= 0 && temp < 10)
  {
    Channel = temp;
    Serial.println("Channel = " + String(Channel) );
  }
    
}

void loop() 
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(500);
  
  // my serpentarium
  bool button_press = true;
  bool wire_connect = true;
  // Serial.println("On the button A0 -> " + String(digitalRead(Button_ChangeCarrierFrequency)) );
  if (button_press == 1 && wire_connect ==1)
  {
    at_settings = true;
    Channel = Channel + 1;
    Serial.println("Size Big Array =" + String(sizeof(CarrierFrequency_Array)/sizeof(CarrierFrequency_Array[0])) );
    if (Channel >= (sizeof(CarrierFrequency_Array)/sizeof(CarrierFrequency_Array[0])) || Channel < 0 )
    {
      Channel = 0;
      Serial.println(F("Reset Channel to ZERO"));
    }
    unsigned long atFR = CarrierFrequency_Array[Channel];
    Serial.println("atFR = " + String(atFR) + "  Ch->" + String(Channel));
    // at_commander.setATFR(atFR);
  }
  if (at_settings == true && wire_connect == false)
  {
    EEPROM.update(CarrierFrequency_addr,Channel);
    at_settings = false;
  } 
}
