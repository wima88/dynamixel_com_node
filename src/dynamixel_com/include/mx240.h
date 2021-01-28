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
     void writeDataToAddress(uint8_t ID_pose ,
                             int tx_data,
                             const uint16_t *address);

     void writeDataToAddress(uint8_t ID_pose ,  /*reg write */
                             int tx_data,
                             const uint16_t *address,
                             uint8_t inst_pose);

     void ReadDataFromAddress(uint8_t ID_pose,
                              const uint16_t *address,
                              uint16_t data_length );

     std::vector<std::string>  syncRead(const uint16_t *address,
                                        const char *ID_array,
                                        int sizeofArray);

     bool readWithcrc_check();
     void set_speed(int left_speed,int right_speed );
     bool read_speed(int* left_speed,int* right_speed );
                 
  private:  
      int gpio_fd;
      mn::CppLinuxSerial::SerialPort serialPort;
      std::string data; //Rx_data
      int header_pattern_start_bit;



      unsigned short update_crc(unsigned short crc_accum,  char *data_blk_ptr, unsigned short data_blk_size);
      void simpleWrite(char * buffer, int bufferSize);
      std::string simpleRead();
      bool crc_check();
      uint16_t param_length_calc();


};

 const char header[] = {0xFF,0xFF,0xFD,0x00};
 const char servo_ID[]   = {0x01,0x02,0xFE}; //L,R,B
 const char inst[]    = {0x01,0x02,0x03,0x04,0x05,0x82}; //ping,read,write

//address list for dynamixel ML420
const uint16_t	TORQUE_ENABLE	 =	64	;
const uint16_t	LED	 =	65	;
const uint16_t	STATUS_RETURN_LEVEL	 =	68	;
const uint16_t	REGISTERED_INSTRUCTION	 =	69	;
const uint16_t	HARDWARE_ERROR_STATUS	 =	70	;
const uint16_t	VELOCITY_I_GAIN	 =	76	;
const uint16_t	VELOCITY_P_GAIN	 =	78	;
const uint16_t	POSITION_D_GAIN	 =	80	;
const uint16_t	POSITION_I_GAIN	 =	82	;
const uint16_t	POSITION_P_GAIN	 =	84	;
const uint16_t	FEEDFORWARD_2ND_GAIN	 =	88	;
const uint16_t	FEEDFORWARD_1ST_GAIN	 =	90	;
const uint16_t	BUS_WATCHDOG	 =	98	;
const uint16_t	GOAL_PWM	 =	100	;
const uint16_t	GOAL_VELOCITY	 =	104	;
const uint16_t	PROFILE_ACCELERATION	 =	108	;
const uint16_t	PROFILE_VELOCITY	 =	112	;
const uint16_t	GOAL_POSITION	 =	116	;
const uint16_t	REALTIME_TICK	 =	120	;
const uint16_t	MOVING	 =	122	;
const uint16_t	MOVING_STATUS	 =	123	;
const uint16_t	PRESENT_PWM	 =	124	;
const uint16_t	PRESENT_LOAD	 =	126	;
const uint16_t	PRESENT_VELOCITY	 =	128	;
const uint16_t	PRESENT_POSITION	 =	132	;
const uint16_t	VELOCITY_TRAJECTORY	 =	136	;
const uint16_t	POSITION_TRAJECTORY	 =	140	;
const uint16_t	PRESENT_INPUT_VOLTAGE	 =	144	;
const uint16_t	PRESENT_TEMPERATURE	 =	146	;
const uint16_t	INDIRECT_ADDRESS_1	 =	168	;
const uint16_t	INDIRECT_ADDRESS_2	 =	170	;
const uint16_t	INDIRECT_ADDRESS_3	 =	172	;
const uint16_t	INDIRECT_ADDRESS_26	 =	218	;
const uint16_t	INDIRECT_ADDRESS_27	 =	220	;
const uint16_t	INDIRECT_ADDRESS_28	 =	222	;
const uint16_t	INDIRECT_DATA_1	 =	224	;
const uint16_t	INDIRECT_DATA_2	 =	225	;
const uint16_t	INDIRECT_DATA_3	 =	226	;
const uint16_t	INDIRECT_DATA_26	 =	249	;
const uint16_t	INDIRECT_DATA_27	 =	250	;
const uint16_t	INDIRECT_DATA_28	 =	251	;
const uint16_t	INDIRECT_ADDRESS_29	 =	578	;
const uint16_t	INDIRECT_ADDRESS_30	 =	580	;
const uint16_t	INDIRECT_ADDRESS_31	 =	582	;
const uint16_t	INDIRECT_ADDRESS_54	 =	628	;
const uint16_t	INDIRECT_ADDRESS_55	 =	630	;
const uint16_t	INDIRECT_ADDRESS_56	 =	632	;
const uint16_t	INDIRECT_DATA_29	 =	634	;
const uint16_t	INDIRECT_DATA_30	 =	635	;
const uint16_t	INDIRECT_DATA_31	 =	636	;
const uint16_t	INDIRECT_DATA_54	 =	659	;
const uint16_t	INDIRECT_DATA_55	 =	660	;
const uint16_t	INDIRECT_DATA_56	 =	661	;

#endif


