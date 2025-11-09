/*
 * /===--------------------------------------------------------------------===/
 * Filename: Ssc32uSerial.cpp
 * Description: C++ class definition for the SSC-32U USB connection
 * /===--------------------------------------------------------------------===/
 **/

#include "Ssc32uSerial.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <sys/select.h>
#include <errno.h>
#include <sstream>
#include <vector>

const std::map<ServoChNum, unsigned int> jointResetPoss = 
    {
        {ServoChNum::BASE,1500}, {ServoChNum::SHOULDER,1500}, 
        {ServoChNum::ELBOW,1500}, {ServoChNum::WRIST,2355}, 
        {ServoChNum::WRIST_ROTATE,1500}
    };

Ssc32uSerial::Ssc32uSerial() : fd_(-1), isOpen_(false) {}

Ssc32uSerial::~Ssc32uSerial()
{
    close();
}

bool Ssc32uSerial::open(const std::string &port, int baudRate)
{
    if (isOpen_)
    {
        lastError_ = "Port already open";
        return false;
    }

    // Open the serial port
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0)
    {
        lastError_ = "Failed to open port: " + port + " - " + std::string(strerror(errno));
        return false;
    }

    // Save current terminal settings
    if (tcgetattr(fd_, &oldTio_) < 0)
    {
        lastError_ = "Failed to get terminal attributes: " + std::string(strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Configure the port
    if (!configure(baudRate))
    {
        lastError_ = "Failed to configure baud rate: " + std::string(strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    isOpen_ = true;
    return true;
}

bool Ssc32uSerial::configure(int baudRate)
{
    struct termios tio;
    memset(&tio, 0, sizeof(tio));

    // Set baud rate
    speed_t speed;
    switch (baudRate)
    {
    case 9600:
        speed = B9600;
        break;
    case 38400:
        speed = B38400;
        break;
    case 115200:
        speed = B115200;
        break;
    default:
        lastError_ = "Unsupported baud rate";
        return false;
    }

    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    // Configure 8N1 (8 data bits, no parity, 1 stop bit)
    tio.c_cflag = speed | CS8 | CLOCAL | CREAD;
    tio.c_iflag = IGNPAR; // Ignore parity errors
    tio.c_oflag = 0;      // Raw output
    tio.c_lflag = 0;      // Raw input (non-canonical mode)

    // Set read timeout (deciseconds)
    tio.c_cc[VMIN] = 0;  // Non-blocking read
    tio.c_cc[VTIME] = 1; // 0.1 second timeout

    // Flush the port and apply settings
    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &tio) < 0)
    {
        lastError_ = "Failed to set terminal attributes: " + std::string(strerror(errno));
        return false;
    }

    return true;
}

void Ssc32uSerial::close()
{
    if (isOpen_ && fd_ >= 0)
    {
        // Restore original terminal settings
        tcsetattr(fd_, TCSANOW, &oldTio_);
        ::close(fd_);
        fd_ = -1;
        isOpen_ = false;
    }
}

bool Ssc32uSerial::isOpen() const
{
    return isOpen_;
}

bool Ssc32uSerial::writeCommand(const std::string &command)
{
    if (!isOpen_)
    {
        lastError_ = "Port not open";
        return false;
    }

    // SSC-32U commands should end with carriage return
    std::string cmd = command;
    if (cmd.empty() || cmd.back() != '\r')
    {
        cmd += '\r';
    }

    ssize_t written = write(fd_, cmd.c_str(), cmd.length());
    if (written < 0)
    {
        lastError_ = "Write failed: " + std::string(strerror(errno));
        return false;
    }

    if (static_cast<size_t>(written) != cmd.length())
    {
        lastError_ = "Incomplete write";
        return false;
    }

    return true;
}

int Ssc32uSerial::waitForData(int timeoutMs)
{
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(fd_, &readSet);

    struct timeval timeout;
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_usec = (timeoutMs % 1000) * 1000;

    int result = select(fd_ + 1, &readSet, nullptr, nullptr, &timeout);
    if (result < 0)
    {
        lastError_ = "Select failed: " + std::string(strerror(errno));
    }
    return result;
}

std::string Ssc32uSerial::readResponse(int timeoutMs)
{
    if (!isOpen_)
    {
        lastError_ = "Port not open";
        return "";
    }

    std::string response;
    char buffer[256];
    int totalTime = 0;
    const int pollInterval = 10;

    while (totalTime < timeoutMs)
    {
        int ready = waitForData(pollInterval);
        if (ready > 0)
        {
            ssize_t bytesRead = read(fd_, buffer, sizeof(buffer) - 1);
            if (bytesRead > 0)
            {
                buffer[bytesRead] = '\0';
                response += buffer;

                // Check if we've received a complete response (ends with \r)
                if (response.find('\r') != std::string::npos)
                {
                    break;
                }
            }
            else if (bytesRead < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
            {
                lastError_ = "Read failed: " + std::string(strerror(errno));
                break;
            }
        }
        else if (ready < 0)
        {
            break;
        }
        totalTime += pollInterval;
    }

    return response;
}

std::string Ssc32uSerial::readLine(int timeoutMs)
{
    std::string response = readResponse(timeoutMs);

    // Remove trailing carriage return and newline
    while (!response.empty() && (response.back() == '\r' || response.back() == '\n'))
    {
        response.pop_back();
    }

    return response;
}

// Sends a single servo command to the SSC-32U with #PS parameters
bool Ssc32uSerial::moveServo(ServoChNum ch, unsigned int thetadeg, unsigned int speed)
{
    double slope = 1.0 * (2500 - 500) / (90 - (-90));
    unsigned int pulseWidth = 500 + round(slope * (thetadeg - (-90)));

    std::stringstream ss;
    ss << '#' << static_cast<char>(ch) << " P" << pulseWidth;
    
    if (speed > 0) {
        ss << " S" << speed;
    }

    std::string cm = ss.str();
    std::cout << "Servo command: " << cm << "\n";
    
    ss << '\r';
    std::string cmd = ss.str();
    
    return writeCommand(cmd);
}

bool Ssc32uSerial::moveServoTimed(ServoChNum ch, unsigned int pulseWidth, unsigned int timeMs)
{
    std::stringstream ss;
    ss << '#' << static_cast<char>(ch) << " P" << pulseWidth << " T" << timeMs << '\r';
    std::string cmd = ss.str();
    std::cout << "Servo command: " << cmd;
    
    return writeCommand(cmd);
}

bool Ssc32uSerial::queryPulseWidth(ServoChNum ch, std::string& response)
{

    if (!writeCommand("QP" + static_cast<char>(ch)))
    {
        return false;
    }

    std::string resp = readLine(100);
    if (!resp.empty())
    {
        std::cerr << "Response to command 'QUERY PULSEWIDTH' empty.\n";
        return false;
    }
    else
    {
        response = resp;
        std::cout << response << std::endl;
        return true;
    }
}

bool Ssc32uSerial::queryMovementStatus()
{
    if (!writeCommand("Q"))
    {
        return false;
    }

    std::string response = readResponse(100);
    return (!response.empty() && response[0] == '.');
}

std::string Ssc32uSerial::queryFirmwareVersion()
{
    if (!writeCommand("VER"))
    {
        return "";
    }

    return readLine(1000);
}

void Ssc32uSerial::flush()
{
    if (isOpen_)
    {
        tcflush(fd_, TCIOFLUSH);
    }
}

std::string Ssc32uSerial::getLastError() const
{
    return lastError_;
}

bool Ssc32uSerial::assumeResetPos()
{
    std::map<ServoChNum,unsigned int> curJointPoss;
    std::map<ServoChNum,unsigned int>::iterator it;
    for (it = curJointPoss.begin(); it != curJointPoss.end(); it++)
    {
        std::string resp;
        this->queryPulseWidth(it->first,resp);
        it->second = std::stoul(resp,nullptr,10);
    }
    
    // TODO:
    // Calculate trajectory using reset map

    // Execute movement

    return true;
}

bool Ssc32uSerial::ssDisplay()
{
    if (!writeCommand("SS\r"))
    {
        return false;
    }

    std::string response = readLine(3000);
    if (!response.empty())
    {
        std::cerr << "Response to command 'DISPLAY STARTUP STRING' empty.\n";
        return false;
    }
    else
    {
        std::cout << response << std::endl;
        return true;
    }
}
