#ifndef SPI_CAN_H
#define SPI_CAN_H

#include <SPI.h>
#include <mcp2515.h>

class SPI_CAN {
  private:
  MCP2515 *mcp;
  MCP2515 mcp2515;
  const int32_t can_id = 0x000;
  struct can_frame out_frame;
  struct can_frame in_frame;
  const uint8_t cs_pin = 10;
  public:
  void begin(unsigned long baud) 
  {
    mcp2515(10);
    mcp = &mcp2515;
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setLoopbackMode();

    //Out Frame
    out_frame.can_id = can_id;
    out_frame.can_dlc = 8;
    out_frame.data[0] = 0xFF;
    out_frame.data[1] = 0xFF;
    out_frame.data[2] = 0xFF;
    out_frame.data[3] = 0xFF;
    out_frame.data[4] = 0xFF;
    out_frame.data[5] = 0xFF;
    out_frame.data[6] = 0xFF;
    out_frame.data[7] = 0xFF;

    //In Frame
    in_frame.can_id = 0x0;
    in_frame.can_dlc = 8;
    in_frame.data[0] = 0xFF;
    in_frame.data[1] = 0xFF;
    in_frame.data[2] = 0xFF;
    in_frame.data[3] = 0xFF;
    in_frame.data[4] = 0xFF;
    in_frame.data[5] = 0xFF;
    in_frame.data[6] = 0xFF;
    in_frame.data[7] = 0xFF;
  }
  
  unsigned char send(unsigned long id, uchar ext, uchar rtrBit, uchar len, const uchar *buf)
  {
    out_frame.can_id = id;
    
    
    for(int i=0; i<len; i++)
    {
        out_frame.data[i] = buf[i];
    }
    
    
    mcp->sendMessage(&out_frame);
  }
  
  unsigned char recv(unsigned long *id, uchar *buf)
  {
    if (mcp->readMessage(&in_frame) == MCP2515::ERROR_OK) {
      Serial.print("Message from ID: ");
      Serial.println(in_frame.can_id);
      for(int i=0; i<4; i++) {
        Serial.println(in_frame.data[i]);
      }
    }
  }
};



#endif
