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
#include <stdexcept>
#include <cstdint>
#include <iostream>

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

// int Ssc32uSerial::waitForData(int timeoutMs)
// {
//     fd_set readSet;
//     FD_ZERO(&readSet);
//     FD_SET(fd_, &readSet);

//     struct timeval timeout;
//     timeout.tv_sec = timeoutMs / 1000;
//     timeout.tv_usec = (timeoutMs % 1000) * 1000;

//     int result = select(fd_ + 1, &readSet, nullptr, nullptr, &timeout);
//     if (result < 0)
//     {
//         lastError_ = "Select failed: " + std::string(strerror(errno));
//     }
//     return result;
// }

// std::string Ssc32uSerial::readResponse(int timeoutMs)
// {
//     if (!isOpen_)
//     {
//         lastError_ = "Port not open";
//         return "";
//     }

//     std::string response;
//     char buffer[256];
//     int totalTime = 0;
//     const int pollInterval = 10;

//     while (totalTime < timeoutMs)
//     {
//         int ready = waitForData(pollInterval);
//         if (ready > 0)
//         {
//             ssize_t bytesRead = read(fd_, buffer, sizeof(buffer) - 1);
//             if (bytesRead > 0)
//             {
//                 buffer[bytesRead] = '\0';
//                 response += buffer;

//                 // Check if we've received a complete response (ends with \r)
//                 if (response.find('\r') != std::string::npos)
//                 {
//                     break;
//                 }
//             }
//             else if (bytesRead < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
//             {
//                 lastError_ = "Read failed: " + std::string(strerror(errno));
//                 break;
//             }
//         }
//         else if (ready < 0)
//         {
//             break;
//         }
//         totalTime += pollInterval;
//     }

//     // Remove trailing carriage return and newline
//     while (!response.empty() && (response.back() == '\r' || response.back() == '\n'))
//     {
//         response.pop_back();
//     }

//     return response;
// }

/* Sends command over USB and does not wait for a response */
bool Ssc32uSerial::sendUsbCommandWriteOnly(const std::string &command)
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

/* Sends command over USB and waits for response */
std::string Ssc32uSerial::sendUsbCommandWriteRead(const std::string &command, int timeoutMs)
{
    if (!isOpen_)
    {
        lastError_ = "Port not open";
        return lastError_;
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
        return lastError_;
    }

    if (static_cast<size_t>(written) != cmd.length())
    {
        lastError_ = "Incomplete write";
        return lastError_;
    }

    std::string response;
    char buffer[256];
    int totalTime = 0;
    const int pollInterval = 10;

    while (totalTime < timeoutMs)
    {
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(fd_, &readSet);

        struct timeval timeout;
        timeout.tv_sec = pollInterval / 1000;
        timeout.tv_usec = (pollInterval % 1000) * 1000;

        int ready = select(fd_ + 1, &readSet, nullptr, nullptr, &timeout);

        if (ready < 0)
        {
            lastError_ = "Select failed: " + std::string(strerror(errno));
            return lastError_;
        }
        else if (ready > 0)
        {
            ssize_t bytesRead = read(fd_, buffer, sizeof(buffer) - 1);
            if (bytesRead < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
            {
                lastError_ = "Read failed: " + std::string(strerror(errno));
                return lastError_;
            }
            else if (bytesRead > 0)
            {
                buffer[bytesRead] = '\0';
                response += buffer;

                // Check if we've received a complete response (ends with \r)
                if (response.find('\r') != std::string::npos)
                {
                    break;
                }
            }
        }
        totalTime += pollInterval;
    }

    // Remove trailing carriage return and newline
    while (!response.empty() && (response.back() == '\r' || response.back() == '\n'))
    {
        response.pop_back();
    }

    return response;
}

// Sends a single servo command to the SSC-32U with #PS parameters
bool Ssc32uSerial::moveServoDeg(ServoChNum ch, unsigned int thetadeg, unsigned int speed)
{
    double slope = 1.0 * (2500 - 500) / (90 - (-90));
    unsigned int pulseWidth = 500 + round(slope * (thetadeg - (-90)));

    std::stringstream ss;
    ss << '#' << static_cast<char>(ch) << " P" << pulseWidth;

    if (speed > 0)
    {
        ss << " S" << speed;
    }

    std::string cm = ss.str();
    std::cout << "Servo command: " << cm << "\n";

    ss << '\r';
    std::string cmd = ss.str();

    return sendUsbCommandWriteOnly(cmd);
}

// Sends a single servo command to the SSC-32U with #PS parameters
bool Ssc32uSerial::moveServoPwm(ServoChNum ch, unsigned int pwm, unsigned int speed)
{
    std::stringstream ss;
    ss << '#' << static_cast<char>(ch) << " P" << pwm;

    if (speed > 0)
    {
        ss << " S" << speed;
    }

    std::string cm = ss.str();
    std::cout << "Servo command: " << cm << "\n";

    ss << '\r';
    std::string cmd = ss.str();

    return sendUsbCommandWriteOnly(cmd);
}

bool Ssc32uSerial::moveServoTimed(ServoChNum ch, unsigned int pulseWidth, unsigned int timeMs)
{
    std::stringstream ss;
    ss << '#' << static_cast<char>(ch) << " P" << pulseWidth << " T" << timeMs << '\r';
    std::string cmd = ss.str();
    std::cout << "Servo command: " << cmd;

    return sendUsbCommandWriteOnly(cmd);
}

bool Ssc32uSerial::moveServoGroup(const std::vector<ServoMove> &moves, unsigned int speed, unsigned int timeMs)
{
    if (moves.empty())
    {
        lastError_ = "No servo moves specified";
        return false;
    }

    std::stringstream ss;

    // Add each servo position
    for (const auto &move : moves)
    {
        ss << '#' << static_cast<char>(move.channel) << " P" << move.pulseWidth << ' ';
    }

    // Add optional speed parameter
    if (speed > 0)
    {
        ss << 'S' << speed << ' ';
    }

    // Add optional time parameter
    if (timeMs > 0)
    {
        ss << 'T' << timeMs << ' ';
    }

    ss << '\r';
    std::string cmd = ss.str();
    std::cout << "Group servo command: " << cmd;

    return sendUsbCommandWriteOnly(cmd);
}

// bool Ssc32uSerial::queryPulseWidth(ServoChNum ch, std::string &response)
// {

//     if (!sendUsbCommandWriteOnly("QP" + static_cast<char>(ch)))
//     {
//         return false;
//     }

//     std::string resp = readLine(100);
//     if (!resp.empty())
//     {
//         std::cerr << "Response to command 'QUERY PULSEWIDTH' empty.\n";
//         return false;
//     }
//     else
//     {
//         response = resp;
//         std::cout << response << std::endl;
//         return true;
//     }
// }

// bool Ssc32uSerial::queryMovementStatus()
// {
//     if (!sendUsbCommandWriteOnly("Q"))
//     {
//         return false;
//     }

//     std::string response = readResponse(100);
//     return (!response.empty() && response[0] == '.');
// }

// std::string Ssc32uSerial::queryFirmwareVersion()
// {
//     if (!sendUsbCommandWriteOnly("VER"))
//     {
//         return "";
//     }

//     return readLine(1000);
// }

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
    /*
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
    */
    moveServoPwm(ServoChNum::BASE, jointResetPoss.at(ServoChNum::BASE));
    moveServoPwm(ServoChNum::SHOULDER, jointResetPoss.at(ServoChNum::SHOULDER));
    moveServoPwm(ServoChNum::ELBOW, jointResetPoss.at(ServoChNum::ELBOW));
    moveServoPwm(ServoChNum::WRIST, jointResetPoss.at(ServoChNum::WRIST));
    moveServoPwm(ServoChNum::WRIST_ROTATE, jointResetPoss.at(ServoChNum::WRIST_ROTATE));

    return true;
}

bool Ssc32uSerial::assumeResetPos(unsigned int timeMs)
{
    /*
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
    */
    moveServoTimed(ServoChNum::BASE, jointResetPoss.at(ServoChNum::BASE), timeMs);
    moveServoTimed(ServoChNum::SHOULDER, jointResetPoss.at(ServoChNum::SHOULDER), timeMs);
    moveServoTimed(ServoChNum::ELBOW, jointResetPoss.at(ServoChNum::ELBOW), timeMs);
    moveServoTimed(ServoChNum::WRIST, jointResetPoss.at(ServoChNum::WRIST), timeMs);
    moveServoTimed(ServoChNum::WRIST_ROTATE, jointResetPoss.at(ServoChNum::WRIST_ROTATE), timeMs);

    return true;
}

// bool Ssc32uSerial::ssDisplay()
// {
//     if (!sendUsbCommandWriteOnly("SS\r"))
//     {
//         return false;
//     }

//     std::string response = readLine(3000);
//     if (!response.empty())
//     {
//         std::cerr << "Response to command 'DISPLAY STARTUP STRING' empty.\n";
//         return false;
//     }
//     else
//     {
//         std::cout << response << std::endl;
//         return true;
//     }
// }

/**
 * Parses the response from reading SSC-32U Register R0 and prints
 * a detailed configuration summary based on the Enable Register bit definitions.
 *
 * Register R0 bit definitions (from SSC-32U documentation):
 * - Bit 15 (MSB): Global Disable - If '1', disables all features controlled by Enable register
 * - Bits 14-4: Reserved
 * - Bit 3: Initial Pulse Width Enable - If '1', enables Initial Pulse Width register values at startup
 * - Bit 2: Initial Pulse Offset Enable - If '1', enables Initial Pulse Offset register values at startup
 * - Bit 1: TX Delay/Pacing Enable - If '1', enables Transmit Delay and Transmit Pacing values
 * - Bit 0 (LSB): Startup String Enable - If '1', enables execution of startup string on power-up
 *
 * @param response The string response from sendUsbCommandWriteRead("R0\r", timeout)
 *                 Expected format: ASCII decimal number (e.g., "1023" or "12")
 */
void Ssc32uSerial::printR0(const std::string &response)
{
    // Trim whitespace from response
    std::string trimmed = response;
    size_t start = trimmed.find_first_not_of(" \t\r\n");
    size_t end = trimmed.find_last_not_of(" \t\r\n");

    if (start == std::string::npos)
    {
        std::cerr << "Error: Empty or whitespace-only response received.\n";
        return;
    }

    trimmed = trimmed.substr(start, end - start + 1);

    // Check for error responses (the function returns error messages as strings)
    if (trimmed.find("failed") != std::string::npos ||
        trimmed.find("error") != std::string::npos ||
        trimmed.find("not open") != std::string::npos)
    {
        std::cerr << "Error: Received error response: " << trimmed << "\n";
        return;
    }

    // Parse the decimal value
    uint16_t registerValue;
    try
    {
        long parsed = std::stol(trimmed);
        if (parsed < 0 || parsed > 65535)
        {
            std::cerr << "Error: Register value out of valid range (0-65535): " << parsed << "\n";
            return;
        }
        registerValue = static_cast<uint16_t>(parsed);
    }
    catch (const std::invalid_argument &)
    {
        std::cerr << "Error: Could not parse response as number: \"" << trimmed << "\"\n";
        return;
    }
    catch (const std::out_of_range &)
    {
        std::cerr << "Error: Number out of range: \"" << trimmed << "\"\n";
        return;
    }

    // Extract individual bits
    bool globalDisable = (registerValue >> 15) & 0x01;       // Bit 15
    bool initialPulseWidthEn = (registerValue >> 3) & 0x01;  // Bit 3
    bool initialPulseOffsetEn = (registerValue >> 2) & 0x01; // Bit 2
    bool txDelayPacingEn = (registerValue >> 1) & 0x01;      // Bit 1
    bool startupStringEn = (registerValue >> 0) & 0x01;      // Bit 0

    // Check for any reserved bits being set (bits 4-14)
    uint16_t reservedBits = (registerValue >> 4) & 0x07FF; // Bits 14-4 (11 bits)

    // Print the configuration
    std::cout << "========================================\n";
    std::cout << "SSC-32U Register R0 (Enable Register)\n";
    std::cout << "========================================\n";
    std::cout << "Raw Value: " << registerValue << " (0x" << std::hex << registerValue << std::dec << ")\n";
    std::cout << "Binary:    ";
    for (int i = 15; i >= 0; --i)
    {
        std::cout << ((registerValue >> i) & 1);
        if (i == 12 || i == 8 || i == 4)
            std::cout << " "; // Group for readability
    }
    std::cout << "\n";
    std::cout << "----------------------------------------\n\n";

    // Global Disable (Bit 15)
    std::cout << "[Bit 15] Global Disable: " << (globalDisable ? "ENABLED" : "DISABLED") << "\n";
    if (globalDisable)
    {
        std::cout << "         *** ALL features controlled by Enable register are DISABLED ***\n";
        std::cout << "         (Individual bit settings below are overridden)\n";
    }
    else
    {
        std::cout << "         Individual bit values are used to control features.\n";
    }
    std::cout << "\n";

    // Reserved bits warning
    if (reservedBits != 0)
    {
        std::cout << "[Bits 14-4] Reserved bits have non-zero value: " << reservedBits << "\n";
        std::cout << "            (These bits should normally be 0)\n\n";
    }

    // Initial Pulse Width Enable (Bit 3)
    std::cout << "[Bit 3]  Initial Pulse Width Enable: " << (initialPulseWidthEn ? "ENABLED" : "DISABLED") << "\n";
    if (initialPulseWidthEn && !globalDisable)
    {
        std::cout << "         Servo initial pulse widths from registers R64-R95 will be\n";
        std::cout << "         applied at startup.\n";
    }
    else if (!initialPulseWidthEn || globalDisable)
    {
        std::cout << "         Default initial pulse width of 1500us will be used.\n";
    }
    std::cout << "\n";

    // Initial Pulse Offset Enable (Bit 2)
    std::cout << "[Bit 2]  Initial Pulse Offset Enable: " << (initialPulseOffsetEn ? "ENABLED" : "DISABLED") << "\n";
    if (initialPulseOffsetEn && !globalDisable)
    {
        std::cout << "         Servo pulse offsets from registers R32-R63 will be applied\n";
        std::cout << "         at startup (range: -100us to +100us per servo).\n";
    }
    else if (!initialPulseOffsetEn || globalDisable)
    {
        std::cout << "         Default pulse offset of 0 will be used for all servos.\n";
    }
    std::cout << "\n";

    // TX Delay/Pacing Enable (Bit 1)
    std::cout << "[Bit 1]  TX Delay/Pacing Enable: " << (txDelayPacingEn ? "ENABLED" : "DISABLED") << "\n";
    if (txDelayPacingEn && !globalDisable)
    {
        std::cout << "         Custom transmit delay (R1) and pacing (R2) values will be used\n";
        std::cout << "         for serial responses.\n";
    }
    else if (!txDelayPacingEn || globalDisable)
    {
        std::cout << "         Default TX delay of 600us and pacing of 70us will be used.\n";
    }
    std::cout << "\n";

    // Startup String Enable (Bit 0)
    std::cout << "[Bit 0]  Startup String Enable: " << (startupStringEn ? "ENABLED" : "DISABLED") << "\n";
    if (startupStringEn && !globalDisable)
    {
        std::cout << "         The programmed startup string will be executed when power\n";
        std::cout << "         is applied to the SSC-32U.\n";
    }
    else if (!startupStringEn || globalDisable)
    {
        std::cout << "         No startup string will be executed at power-on.\n";
    }
    std::cout << "\n";

    // Summary
    std::cout << "----------------------------------------\n";
    std::cout << "CONFIGURATION SUMMARY:\n";
    std::cout << "----------------------------------------\n";

    if (globalDisable)
    {
        std::cout << "  * Global Disable is ON - all configurable features are disabled.\n";
    }
    else
    {
        int enabledCount = 0;
        if (startupStringEn)
        {
            std::cout << "  * Startup string execution: ON\n";
            enabledCount++;
        }
        if (txDelayPacingEn)
        {
            std::cout << "  * Custom TX timing: ON\n";
            enabledCount++;
        }
        if (initialPulseOffsetEn)
        {
            std::cout << "  * Initial pulse offsets: ON\n";
            enabledCount++;
        }
        if (initialPulseWidthEn)
        {
            std::cout << "  * Initial pulse widths: ON\n";
            enabledCount++;
        }

        if (enabledCount == 0)
        {
            std::cout << "  * All optional features are DISABLED (using defaults).\n";
        }
    }
    std::cout << "========================================\n";
}
