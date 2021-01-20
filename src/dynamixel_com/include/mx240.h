#ifndef MX240_H
#define MX240_H

#include"uart.h"
#include"gpio.h"
#include<stdio.h>
#include<string.h>
#include <string>
#include<unistd.h>
#include<vector>

#define _ctrlPin 38
#define   BUFFER_SIZE 128

class MX240 {
   public:
     MX240() 
       //seting up gpio
        gpio_export(_ctrlPin);
	      gpio_set_dir(_ctrlPin,1); //set as output
        gpio_fd = gpio_fd_open(_ctrlPin);

        printf("constructor init\n");
      }

      ~MX240(){}
      std::vector<std::string> ping(uint8_t ID_pose); 
                 
  private:  
      int gpio_fd;
      uint16_t crc;
      char crc_[2];
      char tx_buffer[BUFFER_SIZE]; 
      uint16_t len;
      Uart uart;
      unsigned short update_crc(unsigned short crc_accum,  char *data_blk_ptr, unsigned short data_blk_size);
      void simpleWrite(char *buffer )
      { 
        std::string m_s;
        m_s.assign(buffer,sizeof(buffer));
        gpio_set_value(_ctrlPin,1);
      }



};

 const char header[] = {0xFF,0xFF,0xFD,0x00};
 const char servo_ID[]   = {0x01,0x02,0xFE}; //L,R,B
 const char inst[]    = {0x01,0x02,0x03}; //ping,read,write



#endif
