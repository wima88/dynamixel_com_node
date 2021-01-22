#include "ros/ros.h"
#include "std_msgs/String.h"
//#include <CppLinuxSerial/SerialPort.hpp>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <unistd.h>
//#include "gpio.h"
#include <vector>
#include <iostream>
#include "mx240.h"

#define _ctrlPin 38
#define BUFFER_SIZE 128

using namespace mn::CppLinuxSerial;
using namespace std;

char test_buffer[] = {0x00,0x02,0x00,0x00,};
int dataT0Write =512;
uint16_t add= 0x0074;
uint16_t torq_address = 0x40;
uint16_t LED = 0x41;
int main(int argc, char *argv[]) {

 MX240 mx240;
//cout <<mx240.ping(0x00)<<endl;
cout <<mx240.ping(0x00)<<endl;
cout <<mx240.ping(0x01)<<endl;

//mx240.writeDataToAddress(0x00,test_buffer,sizeof(test_buffer),&add); 
mx240.writeDataToAddress(0x02,1,&torq_address);
usleep(2500);  
mx240.writeDataToAddress(0x02,1024,&add);
usleep(2500000); 
mx240.writeDataToAddress(0x02,0,&add);
usleep(2500000); 
//mx240.writeDataToAddress(0x00,200,&add); 
//if(mx240.ping(0x00)) cout <<"ping sucsessfull";

}


