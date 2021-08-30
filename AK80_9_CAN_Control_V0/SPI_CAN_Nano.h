#ifndef SPI_CAN_H
#define SPI_CAN_H

#include <SPI.h>
#include <mcp2515.h>

class SPI_CAN {
  private:
  MCP2515 mcp;
  const int32_t can_id = 0x000;
  struct can_frame out_frame {
    .can_id = can_id;
    .can_dlc = 8;
    .data[0] = 0xFF;
    .data[1] = 0xFF;
    .data[2] = 0xFF;
    .data[3] = 0xFF;
    .data[4] = 0xFF;
    .data[5] = 0xFF;
    .data[6] = 0xFF;
    .data[7] = 0xFF;
  };
  struct can_frame in_frame {
    .can_id = 0x0;
    .can_dlc = 8;
    .data[0] = 0xFF;
    .data[1] = 0xFF;
    .data[2] = 0xFF;
    .data[3] = 0xFF;
    .data[4] = 0xFF;
    .data[5] = 0xFF;
    .data[6] = 0xFF;
    .data[7] = 0xFF;
  };
  public:
  void begin(unsigned long baud) 
  {
    mcp(10);
    mcp.reset();
    mcp.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp.setLoopbackMode();
  }
  
  unsigned char send(unsigned long id, uchar ext, uchar rtrBit, uchar len, const uchar *buf)
  {
    out_frame.can_id = id;
    
    
    for(int i=0; i<len; i++)
    {
        out_frame.data[i] = buf[i];
    }
    
    
    mcp.sendMessage(&frame);
  }
  
  unsigned char recv(unsigned long *id, uchar *buf)
  {
    if (mcp.readMessage(&frame) == MCP2515::ERROR_OK) {
      Serial.print("Message from ID: ");
      Serial.println(frame.can_id);
      for(int i=0; i<4; i++) {
        Serial.println(fram.data[i
      }
    }
  }
};



#endif
