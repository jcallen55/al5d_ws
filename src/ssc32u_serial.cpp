/*
 * Author: Jackson Allen
 * Filename: ssc32u_serial.cpp
 * 
 * */

#include <ssc32u_serial.hpp>

static speed_t to_baud(unsigned rate)
{
  switch (rate)
  {
  case 9600:
    return B9600;
  case 19200:
    return B19200;
  case 38400:
    return B38400;
  case 57600:
    return B57600;
  case 115200:
    return B115200;
  default:
    return B9600;
  }
}

int fd_ = -1;

Serial::~Serial() 
{ 
    close(); 
}

bool Serial::open(const std::string &dev, unsigned baud)
{
    fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
    {
        std::cerr << "open failed: " << strerror(errno) << "\n";
        return false;
    }
    termios tio{};
    if (tcgetattr(fd_, &tio) != 0)
    {
        std::cerr << "tcgetattr failed: " << strerror(errno) << "\n";
        return false;
    }
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

    speed_t sp = to_baud(baud);
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);

    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tio) != 0)
    {
        std::cerr << "tcsetattr failed: " << strerror(errno) << "\n";
        return false;
    }

    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, (flags & ~O_NONBLOCK));
    return true;
}

void Serial::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

bool Serial::writeLine(const std::string &s)
{
    if (fd_ < 0)
        return false;
    std::string out = s;
    if (out.empty() || out.back() != '\r')
        out.push_back('\r');
    ssize_t n = ::write(fd_, out.data(), out.size());
    return n == (ssize_t)out.size();
}

std::string Serial::readFor(int ms = 100)
{
    if (fd_ < 0)
        return {};
    std::string buf;
    auto start = std::chrono::steady_clock::now();
    char tmp[256];
    while (true)
    {
        int avail = 0;
        ioctl(fd_, FIONREAD, &avail);
        if (avail > 0)
        {
        ssize_t n = ::read(fd_, tmp, std::min(avail, (int)sizeof(tmp)));
        if (n > 0)
            buf.append(tmp, tmp + n);
        }
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() >= ms)
        break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return buf;
}

bool Serial::ok() const
{ 
    return fd_ >= 0;
}