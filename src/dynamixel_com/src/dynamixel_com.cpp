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
#include "mx240.h"

#define _ctrlPin 38
#define BUFFER_SIZE 128

using namespace mn::CppLinuxSerial;
using namespace std;



int main(int argc, char *argv[]) {

//string reply;
 MX240 mx240;
 mx240.ping(0x00);
//if(mx240.ping(0x00)) cout <<"ping sucsessfull";

}


