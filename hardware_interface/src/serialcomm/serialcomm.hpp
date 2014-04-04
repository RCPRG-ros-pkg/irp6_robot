/*
 * serialcomm.hpp
 *
 *  Created on: June 05, 2012
 *      Author: mwalecki
 */

#ifndef ELEKTRON_HPP_
#define ELEKTRON_HPP_

#include <stdint.h>
#include <termios.h>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <fstream>
#include <inttypes.h>

// baudrate
#define DEFAULT_BAUD B57600
// port
#define DEFAULT_PORT = "/dev/ttyACM0"



class SerialComm {
public:
	SerialComm(const std::string& port, int baud = DEFAULT_BAUD);
	~SerialComm();
	
	int write(uint8_t* buf, int len);
	int read(uint8_t* buf, int len);
	uint8_t readOneByte();
	bool isConnected();
	

private:
	int fd;
	struct termios oldtio;
	bool connected;
	fd_set rfds;
	struct timeval tv;
	
};

#endif /* ELEKTRON_HPP_ */
