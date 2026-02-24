#include <SPI.h>
#include <mcp_can.h>

const int SPI_CS_PIN = 10;
const int CAN_INT_PIN = 2;
const int BUTTON_PIN = 3; 

MCP_CAN CAN0(SPI_CS_PIN);

bool sportMode = false;
bool lastButtonState = HIGH; 

void setup()
{
  Serial.begin(115200);
  while(!Serial);

  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) 
  {
    //CSV Header
    Serial.println("Time_ms,RPM,Sport_Mode,Fault_Code");
  }

  CAN0.setMode(MCP_NORMAL);
  pinMode(CAN_INT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); 
}

void loop()
{
  //1. BUTTON LOGIC (User Input)
  bool currentButtonState = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && currentButtonState == LOW) 
  {
    sportMode = !sportMode; 
    byte data[1] = {sportMode ? (byte)1 : (byte)0};
    CAN0.sendMsgBuf(0x200, 0, 1, data); 
    delay(50); 
  }
  lastButtonState = currentButtonState; 

  //2.CAN RECEIVE 
  if(!digitalRead(CAN_INT_PIN))
  {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    // Normal Rpm Operations
    if(rxId == 0x103)
    {
       uint16_t rpm = (rxBuf[0] << 8) | rxBuf[1];
       
       // Print CSV format
       Serial.print(millis());
       Serial.print(",");
       Serial.print(rpm);
       Serial.print(",");
       Serial.print(sportMode);
       Serial.print(",");
       Serial.println(0); // 0 means No Fault
    }
    // emergency fault (ID 0x001) ex. engine misfire
    else if(rxId == 0x001 && rxBuf[0] == 0xFF) 
    {
       //CSV format 
       Serial.print(millis());
       Serial.print(",");
       Serial.print(6500); // Peg the graph at redline
       Serial.print(",");
       Serial.print(sportMode);
       Serial.print(",");
       Serial.println(1); // 1 means FAULT TRIGGERED
    }
  }
}