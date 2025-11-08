#ifndef SSC32U_SERIAL_HPP
#define SSC32U_SERIAL_HPP

#include <string>
#include <termios.h>

class Ssc32uSerial {
public:
    // Constructor and Destructor
    Ssc32uSerial();
    ~Ssc32uSerial();

    // Prevent copying
    Ssc32uSerial(const Ssc32uSerial&) = delete;
    Ssc32uSerial& operator=(const Ssc32uSerial&) = delete;

    // Connection management
    bool open(const std::string& port = "/dev/ttyUSB0", int baudRate = 115200);
    void close();
    bool isOpen() const;

    // Communication methods
    bool writeCommand(const std::string& command);
    std::string readResponse(int timeoutMs = 1000);
    std::string readLine(int timeoutMs = 1000);
    
    // Query methods for SSC-32U
    bool queryMovementComplete();
    std::string queryVersion();

    // Utility
    void flush();
    std::string getLastError() const;

private:
    int fd_;                    // File descriptor for serial port
    struct termios oldTio_;     // Original terminal settings
    bool isOpen_;
    std::string lastError_;

    // Helper methods
    bool configure(int baudRate);
    int waitForData(int timeoutMs);
};

#endif // SSC32U_SERIAL_HPP