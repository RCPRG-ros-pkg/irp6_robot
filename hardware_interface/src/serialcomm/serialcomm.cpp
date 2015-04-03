/*
 *  Copyright  (c)  2014-2015,  Robot  Control  and  Pattern  Recognition  Group,  Warsaw  University  of  Technology.
 *  All  rights  reserved.
 *
 *  Redistribution  and  use  in  source  and  binary  forms,  with  or  without
 *  modification,  are  permitted  provided  that  the  following  conditions  are  met:
 *
 *  *  Redistributions  of  source  code  must  retain  the  above  copyright
 *  notice,  this  list  of  conditions  and  the  following  disclaimer.
 *  *  Redistributions  in  binary  form  must  reproduce  the  above  copyright
 *  notice,  this  list  of  conditions  and  the  following  disclaimer  in  the
 *  documentation  and/or  other  materials  provided  with  the  distribution.
 *  *  Neither  the  name  of  the  Robot  Control  and  Pattern  Recognition  Group,
 *  Warsaw  University  of  Technology  nor  the  names  of  its  contributors  may
 *  be  used  to  endorse  or  promote  products  derived  from  this  software
 *  without  specific  prior  written  permission.
 *
 *  THIS  SOFTWARE  IS  PROVIDED  BY  THE  COPYRIGHT  HOLDERS  AND  CONTRIBUTORS  "AS  IS"
 *  AND  ANY  EXPRESS  OR  IMPLIED  WARRANTIES,  INCLUDING,  BUT  NOT  LIMITED  TO,  THE
 *  IMPLIED  WARRANTIES  OF  MERCHANTABILITY  AND  FITNESS  FOR  A  PARTICULAR  PURPOSE
 *  ARE  DISCLAIMED.  IN  NO  EVENT  SHALL  THE  COPYRIGHT  OWNER  OR  CONTRIBUTORS  BE
 *  LIABLE  FOR  ANY  DIRECT,  INDIRECT,  INCIDENTAL,  SPECIAL,  EXEMPLARY,  OR
 *  CONSEQUENTIAL  DAMAGES  (INCLUDING,  BUT  NOT  LIMITED  TO,  PROCUREMENT  OF
 *  SUBSTITUTE  GOODS  OR  SERVICES;  LOSS  OF  USE,  DATA,  OR  PROFITS;  OR  BUSINESS
 *  INTERRUPTION)  HOWEVER  CAUSED  AND  ON  ANY  THEORY  OF  LIABILITY,  WHETHER  IN
 *  CONTRACT,  STRICT  LIABILITY,  OR  TORT  (INCLUDING  NEGLIGENCE  OR  OTHERWISE)
 *  ARISING  IN  ANY  WAY  OUT  OF  THE  USE  OF  THIS  SOFTWARE,  EVEN  IF  ADVISED  OF  THE
 *  POSSIBILITY  OF  SUCH  DAMAGE.
 */

#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <cstring>
#include <iostream> // NOLINT
#include <string>

#include "serialcomm.hpp"

SerialComm::SerialComm(const std::string& port, int baud) {
  connected = false;

  fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd >= 0) {
    tcgetattr(fd, &oldtio);

    // set up new settings
    struct termios newtio;
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag = CBAUD | CS8 | CLOCAL | CREAD | CSTOPB;
    newtio.c_iflag = INPCK | IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    // newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //####
    if (cfsetispeed(&newtio, baud) < 0 || cfsetospeed(&newtio, baud) < 0) {
      fprintf(stderr, "Failed to set serial baud rate: %d\n", baud);
      tcsetattr(fd, TCSANOW, &oldtio);
      close(fd);
      fd = -1;
      return;
    }
    // activate new settings
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);
    // fcntl(fd, F_SETFL, FNDELAY); //####
    connected = true;
  }
}

SerialComm::~SerialComm() {
  // restore old port settings
  if (fd > 0)
    tcsetattr(fd, TCSANOW, &oldtio);
  close(fd);
}

int SerialComm::write(uint8_t* buf, int len) {
  tcflush(fd, TCIFLUSH);
  return ::write(fd, buf, len);
}

int SerialComm::read(uint8_t* buf, int len) {
  return ::read(fd, reinterpret_cast<char*>(buf), len);
}

uint8_t SerialComm::readOneByte() {
  uint8_t ret;

  struct timeval tv;

  // Prepare mask for select - Watch fd to see when it has input
  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);
  do {
    int sel;
    printf("Read attempt.\n");
    // Prepare imeout struct for select - Wait up to five seconds
    tv.tv_sec = 5;
    tv.tv_usec = 0;

    sel = select(fd + 1, &rfds, NULL, NULL, &tv);
    // Don't rely on the value of tv now!

    if (sel == -1)
      perror("select()");
    else if (sel == 0)
      printf("No data within five seconds.\n");
  } while (::read(fd, reinterpret_cast<char*>(&ret), 1) < 1);
  return ret;
}

bool SerialComm::isConnected() {
  return connected ? true : false;
}

