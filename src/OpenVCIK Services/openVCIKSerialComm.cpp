/*                                   LICENSE 
*                                ----------------
*   Copyright (c) 2015,  Raman Research Institute, Bangalore, India.
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.

*   You should have received a copy of the GNU General Public License along
*   with this program; if not, write to the Free Software Foundation, Inc.,
*   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*   This work was supported by the Department of Science and Technology (DST), India. 
* 
*   RRI hereby disclaims all copyright interest in the project OpenVCIK.
*/


/*                                                DESCRIPTOPN
 *
 *  This program is for reading and writing to/from ports with most common settings.One can set the BAUD RATE
 *  for INPUT and OUTPUT. Also can set VMIN and VTIME parameters which are very important and crucial in case 
 *  of NONCANONICAL operation. 
 *
 *      Author : Junaid Ahmed Ansari.
 *      email: ansariahmedjunaid@gmail.com 
 *      github : junaidcs032
 *
 */


#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>


#ifndef SERIAL_COMM
#define SERIAL_COMM

namespace OpenVCIK{

using namespace std;

class SerialComm{
  private: 
    struct termios _options;
    int _fd;
    string _fileName;
  
  public:

    SerialComm(string fname): _fd(-1), _fileName(fname){cerr<<"constructor of SerialComm\n";}  
    ~SerialComm(){

      if(_fd != -1){

            close(_fd);
      }
    }

    /* opens the serial port */
    int openPort(){
      
      _fd = open(_fileName.c_str(), O_RDWR | O_NOCTTY /*| O_NONBLOCK*/);
      if(_fd == -1){
        return -1;    // file couldnt open
      }

    if(tcgetattr(_fd, &_options) == -1)
      return -2;   // file opend but cant get attr

    return 0;   // success
  }

    // function which configures things with default.
    int setupPortParameters(speed_t = B9600, speed_t = B9600, int = 0, int = 1);
    int readPort(string* buf, int size);                          // read into buff array of size specified
    int writePort(string buf);
    bool flushPortIO();

};


/*set parameters specific to our work and actually most of the USB serial can be used this way
/* returns -1 if tcgetattr() failed 
/* returns -2 if cfsetispeed() failed 
/* returns -3 if cfsetospeed() failed 
/* returns -4 if tcsetattr() failed */
int SerialComm::setupPortParameters(speed_t in, speed_t out, int vmin, int vtime){
    /* Get currently set options for the tty */
    if(tcgetattr(_fd, &_options) == -1) return -1;
     
    /* Set custom options */
     
    /* 9600 baud */
     if(cfsetispeed(&_options, in) == -1)  return -2;  
     if(cfsetospeed(&_options, out) == -1) return -3;
     /* 8 bits, no parity, no stop bits */
     _options.c_cflag &= ~PARENB;
     _options.c_cflag &= ~CSTOPB;
     _options.c_cflag &= ~CSIZE;
     _options.c_cflag |= CS8;
     /* no hardware flow control */
     _options.c_cflag &= ~CRTSCTS;
     /* enable receiver, ignore status lines */
     _options.c_cflag |= CREAD | CLOCAL;
     /* disable input/output flow control, disable restart chars */
     _options.c_iflag &= ~(IXON | IXOFF | IXANY);
     /* disable canonical input, disable echo,
     disable visually erase chars,
     disable terminal-generated signals */
     _options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
     /* disable output processing */
     _options.c_oflag &= ~OPOST;
     
    /* wait for 24 characters to come in before read returns */
     _options.c_cc[VMIN] = vmin;
     /* no minimum time to wait before read returns */
     _options.c_cc[VTIME] = vtime;
     
    /* commit the options */
     if(tcsetattr(_fd, TCSANOW, &_options) == -1) return -4;

}


//write to port
int SerialComm::writePort(string buf){

  // LOCK HERE

  return write(_fd, buf.c_str(), buf.length());          // here we donot need not worry about including null in size as we will not send null any way

  // UNLOCK HERE

}

// read into buff array of size specified
// return of -1 means either nothing read or pointer passed was NULL
int SerialComm::readPort(string *buf, int size=1){

  char data[size+1];         // initiallizing to avoid null termination issues
  int bytesRead=-1;

  // LOCK HERE
  if(buf != NULL){
      bytesRead = read(_fd, data, size);
      if(bytesRead>0){

        *buf = (data);   // if doesnt work trying casting it : (const char*)(data)  
      }
      else
        *buf = "";      // return a Null string . remember "\0" is same as ""
      // UNLOCK HERE

      return bytesRead; 
  }
  else{
      *buf = "";
      return -1;     
  }

}

// flushes the input and output buffer.
// can be changed with TCIFLUSH, TCOFLUSH for input only or output only
bool SerialComm::flushPortIO(){

  //LOCK HERE
  tcflush(_fd, TCIOFLUSH);
  //UNLOCK HERE
  
  return true;
}

} //end of namspace
#endif
