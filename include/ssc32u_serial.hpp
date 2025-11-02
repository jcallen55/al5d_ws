/*
 * Author: Jackson Allen
 * Filename: ssc32u_serial.hpp
 *
 * */

#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <cstring>
#include <iostream>
#include <thread>
#include <csignal>

// Coverts int baud to portable baud
static speed_t to_baud(unsigned rate);

// --- CLASS Serial - for serial/usb connection ---
class Serial
{
public:
    ~Serial();

    bool open(const std::string &dev, unsigned baud);

    void close();

    bool writeLine(const std::string &s);

    std::string readFor(int ms = 100);

    bool ok() const;
};