/*
 * serialcomm.cpp
 *
 *  Created on: June 05, 2012
 *      Author: mwalecki
 */

#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <cstring>
#include <iostream>

#include "serialcomm.hpp"

using namespace std;

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
		//newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //####
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
		//fcntl(fd, F_SETFL, FNDELAY);	//####
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
	return ::read(fd, (char*) buf, len);
}

uint8_t SerialComm::readOneByte() {
	uint8_t ret;
	int sel;
	struct timeval tv;

	// Prepare mask for select - Watch fd to see when it has input
	FD_ZERO(&rfds);
	FD_SET(fd, &rfds);
	do {
		printf("Read attempt.\n");
		// Prepare imeout struct for select - Wait up to five seconds
		tv.tv_sec = 5;
		tv.tv_usec = 0;
	
		sel = select(fd+1, &rfds, NULL, NULL, &tv);
		// Don't rely on the value of tv now!

		if (sel == -1)
			perror("select()");
		else if (sel == 0)
			printf("No data within five seconds.\n");
	} while( ::read(fd, (char*)(&ret), 1) < 1);
	return ret;
}

bool SerialComm::isConnected() {
	return connected?true:false;
}

