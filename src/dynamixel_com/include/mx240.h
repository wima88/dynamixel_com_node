#ifndef MX240_H
#define MX240_H

#include<stdio.h>
#include<string.h>
#include <string>
#include<unistd.h>
#include<vector>

#include"gpio.h"
#include <CppLinuxSerial/SerialPort.hpp>

#define _ctrlPin 38
#define BUFFER_SIZE 128

#define DEBUG 1

class MX240 {
   public:
     MX240();
     ~MX240();
     std::string ping(uint8_t ID_pose);
                 
  private:  
      int gpio_fd;
      char tx_buffer[BUFFER_SIZE]; 
      mn::CppLinuxSerial::SerialPort serialPort;
      static std::string data;
      int header_pattern_start_bit;



      unsigned short update_crc(unsigned short crc_accum,  char *data_blk_ptr, unsigned short data_blk_size);
      void simpleWrite(char * buffer, int bufferSize);
      std::string simpleRead();
      bool crc_check();
      uint16_t param_length_calc();


};

 const char header[] = {0xFF,0xFF,0xFD,0x00};
 const char servo_ID[]   = {0x01,0x02,0xFE}; //L,R,B
 const char inst[]    = {0x01,0x02,0x03}; //ping,read,write



#endif
