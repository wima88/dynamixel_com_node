/************************************/
/* @auteur Mathieu Bahin            */
/* @date_création mars 2020         */
/* @version 1.0                     */
/* @email bahin.mathieu@gmail.com   */
/************************************/

#include "uart.h"
#include <stdio.h>
#include <unistd.h>       // Used for UART
#include <sys/fcntl.h>    // Used for UART
#include <termios.h>      // Used for UART
#include <string.h>
#include <sys/errno.h>

using namespace std;

Uart :: Uart (){
  int ii, jj, kk;

  // SETUP SERIAL WORLD

  struct termios  port_options;   // Create the structure

  tcgetattr(fid, &port_options);	// Get the current attributes of the Serial port


  //------------------------------------------------
  //  OPEN THE UART
  //------------------------------------------------
  // The flags (defined in fcntl.h):
  //	Access modes (use 1 of these):
  //		O_RDONLY - Open for reading only.
  //		O_RDWR   - Open for reading and writing.
  //		O_WRONLY - Open for writing only.
  //	    O_NDELAY / O_NONBLOCK (same function)
  //               - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
  //                 if there is no input immediately available (instead of blocking). Likewise, write requests can also return
  //				   immediately with a failure status if the output can't be written immediately.
  //                 Caution: VMIN and VTIME flags are ignored if O_NONBLOCK flag is set.
  //	    O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.fid = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode

  fid = open(uart_target, O_RDWR | O_NOCTTY );

  tcflush(fid, TCIFLUSH);
  tcflush(fid, TCIOFLUSH);

  usleep(1000000);  // 1 sec delay

  if (fid == -1)
  {
    printf("Error %i from open: %s\n", errno, strerror(errno));
  }

  //------------------------------------------------
  // CONFIGURE THE UART
  //------------------------------------------------
  // flags defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html
  //	Baud rate:
  //         - B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200,
  //           B230400, B460800, B500000, B576000, B921600, B1000000, B1152000,
  //           B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
  //	CSIZE: - CS5, CS6, CS7, CS8
  //	CLOCAL - Ignore modem status lines
  //	CREAD  - Enable receiver
  //	IGNPAR = Ignore characters with parity errors
  //	ICRNL  - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
  //	PARENB - Parity enable
  //	PARODD - Odd parity (else even)

  port_options.c_cflag &= ~PARENB;            // Disables the Parity Enable bit(PARENB),So No Parity
  port_options.c_cflag &= ~CSTOPB;            // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
  port_options.c_cflag &= ~CSIZE;	            // Clears the mask for setting the data size
  port_options.c_cflag |=  CS8;               // Set the data bits = 8
  port_options.c_cflag &= ~CRTSCTS;           // No Hardware flow Control
  port_options.c_cflag |=  CREAD | CLOCAL;                  // Enable receiver,Ignore Modem Control lines
  port_options.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both input & output
  port_options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode
  port_options.c_oflag &= ~OPOST;                           // No Output Processing

  port_options.c_lflag = 0;               //  enable raw input instead of canonical,

  port_options.c_cc[VMIN]  = VMINX;       // Read at least 1 character
  port_options.c_cc[VTIME] = 0;           // Wait indefinetly

  cfsetispeed(&port_options,BAUDRATE);    // Set Read  Speed
  cfsetospeed(&port_options,BAUDRATE);    // Set Write Speed

  // Set the attributes to the termios structure
 if (tcsetattr(fid, TCSANOW, &port_options) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  } 

  // Flush Buffers
  tcflush(fid, TCIFLUSH);
  tcflush(fid, TCIOFLUSH);

  usleep(500000);   // 0.5 sec delay


}

void Uart :: sendUart(unsigned char *msg){
  //--------------------------------------------------------------
  // TRANSMITTING BYTES
  //--------------------------------------------------------------
  int tx = write(fid, msg, sizeof(msg));

  if(tx < 0)
  {
    printf("Error %i from sendUart: %s\n", errno, strerror(errno));
  }


}

int Uart :: readUart(char *read_buf)
{

  //--------------------------------------------------------------
  // RECEIVING BYTES - AND BUILD MESSAGE RECEIVED
  //--------------------------------------------------------------
  int rx = read(fid, read_buf, sizeof(read_buf));

  if(rx < 0)
  {
    printf("Error %i from readUart: %s\n", errno, strerror(errno));
    return errno;
  }
return rx;
}

void Uart :: closeUart(){
  //-------------------------------------------
  //  CLOSE THE SERIAL PORT
  //-------------------------------------------
  close(fid);
}
