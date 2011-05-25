/* TODO:
 *
 * inicjalizacja struktur servo_data w konstruktorze hi_moxa
 * przekazanie do konstruktora hi_moxa danych o ilosci i numerach portow
 */

#ifndef __HI_MOXA_H
#define __HI_MOXA_H

#define USLEEP_US 500000

#include "hi_moxa_combuf.h"

#include <stdint.h>
#include <termios.h>
//#include <ctime>

#include <string>
#include <vector>

namespace hi_moxa {

const std::size_t WRITE_BYTES = 10;
const std::size_t READ_BYTES = 8;
const std::size_t MOXA_SERVOS_NR = 8;
const int MAX_PARAM_SET_ATTEMPTS = 3;
const int MAX_COMM_TIMEOUTS = 3;
const int FIRST_HARDWARE_READS_WITH_ZERO_INCREMENT = 4;

const int MAX_CURRENT_0 = 15000;
const int MAX_CURRENT_1 = 18000;
const int MAX_CURRENT_2 = 10000;
const int MAX_CURRENT_3 = 10000;
const int MAX_CURRENT_4 = 10000;
const int MAX_CURRENT_5 = 10000;

/*!
 * @brief IRp6 postument max encoder increment
 * @ingroup irp6p_m
 */
const double ridiculous_increment[] = { 150, 200, 150, 150, 150, 200 };

// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------


class HI_moxa {

public:

	HI_moxa(unsigned int numberOfDrivers); // Konstruktor
	~HI_moxa();

	virtual void init(std::vector<std::string> ports);
	virtual void insertSetValue(int drive_offset, double set_value);
	virtual int getCurrent(int drive_offset);
	virtual double getIncrement(int drive_offset);
	virtual long int getPosition(int drive_offset);
	virtual uint64_t readWriteHardware(void); // Obsluga sprzetu
	virtual void resetCounters(void); // Zerowanie licznikow polozenia
	virtual void startSynchro(int drive_offset);
	virtual void finishSynchro(int drive_offset);
	virtual bool isInSynchroArea(int drive);

	virtual bool isImpulseZero(int drive_offset);
	virtual void resetPosition(int drive_offset);

	bool isRobotSynchronized();
	virtual void set_command_param(int drive_offset, uint8_t param);
	virtual int set_parameter(int drive_number, const int parameter,
			uint32_t new_value);

protected:
private:
#if defined(B921600)
	static const speed_t BAUD = B921600;
#else
	static const speed_t BAUD = 921600;
#endif

	std::size_t last_drive_number;
	//std::vector<double> ridiculous_increment;
	std::vector<std::string> port_names;
	int fd[MOXA_SERVOS_NR], fd_max;
	struct servo_St servo_data[MOXA_SERVOS_NR];
	struct termios oldtio[MOXA_SERVOS_NR];
	bool hardware_panic;
}; // koniec: class hardware_interface

} // namespace hi_moxa

#endif // __HI_MOXA_H
