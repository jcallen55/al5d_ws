/*
 * Filename: Ssc32uSerial.hpp
 *
 * */

#ifndef SSC32U_SERIAL_HPP
#define SSC32U_SERIAL_HPP

#include <string>
#include <termios.h>
#include <vector>
#include "utils.hpp"

class Ssc32uSerial
{
public:
    // Constructor and Destructor
    Ssc32uSerial();
    ~Ssc32uSerial();

    // Prevent copying
    Ssc32uSerial(const Ssc32uSerial &) = delete;
    Ssc32uSerial &operator=(const Ssc32uSerial &) = delete;

    // Connection management
    bool open(const std::string &port = "/dev/ttyUSB0", int baudRate = 115200);
    void close();
    bool isOpen() const;

    // Communication methods
    bool sendUsbCommandWriteOnly(const std::string &command);
    std::string sendUsbCommandWriteRead(const std::string &command, int timeoutMs);
    std::string readResponse(int timeoutMs = 1000);

    // Query methods for SSC-32U
    bool moveServoDeg(ServoChNum ch, unsigned int pw, unsigned int speed = 0);
    bool moveServoPwm(ServoChNum ch, unsigned int pw, unsigned int speed = 0);
    bool moveServoTimed(ServoChNum ch, unsigned int pw, unsigned int timeMs);
    bool commandGroup();
    bool servoPositionOffset();
    bool cancelOutput();
    bool discreteOutput();
    bool byteOutput();
    bool queryMovementStatus();
    bool queryPulseWidth(ServoChNum ch, std::string &resp);
    bool readDigitalInput();
    bool readAnalogInput();
    bool readBaudR4();
    bool stopServo();
    bool ssDisplay();
    bool ssDeleteCharacters();
    bool ssConcatenate();
    std::string queryFirmwareVersion();
    void printR0(const std::string &response);

    bool moveServoGroup(const std::vector<ServoMove> &moves, unsigned int speed = 0, unsigned int timeMs = 0);

    // Utility
    void flush();
    std::string getLastError() const;

    // AL5D
    bool assumeResetPos();
    bool assumeResetPos(unsigned int timeMs);

private:
    int fd_;                // File descriptor for serial port
    struct termios oldTio_; // Original terminal settings
    bool isOpen_;
    std::string lastError_;

    // Helper methods
    bool configure(int baudRate);
    int waitForData(int timeoutMs);
};

#endif // SSC32U_SERIAL_HPP