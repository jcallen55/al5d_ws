/*
 * Author: Jackson Allen
 * Filename: ssc32u.cpp
 * 
 * */

#include <ssc32u.hpp>

static std::string trim(std::string s)
{
  auto issp = [](unsigned char c)
  { return std::isspace(c); };
  while (!s.empty() && issp(s.back()))
    s.pop_back();
  size_t i = 0;
  while (i < s.size() && issp(s[i]))
    ++i;
  return s.substr(i);
}

bool SSC32U::move(uint8_t ch, int pw, std::optional<int> spd = std::nullopt, std::optional<int> time_ms = std::nullopt)
{
    std::ostringstream oss;
    oss << "#" << int(ch) << " P" << pw;
    if (spd)
        oss << " S" << *spd;
    if (time_ms)
        oss << " T" << *time_ms;
    return ser_.writeLine(oss.str());
}

bool SSC32U::groupMove(const std::vector<std::tuple<uint8_t, int, std::optional<int>>> &items, std::optional<int> time_ms = std::nullopt)
{
    std::ostringstream oss;
    for (auto &t : items)
    {
        auto ch = std::get<0>(t);
        auto pw = std::get<1>(t);
        auto sp = std::get<2>(t);
        oss << "#" << int(ch) << " P" << pw;
        if (sp)
        oss << " S" << *sp;
        oss << " ";
    }
    if (time_ms)
        oss << "T" << *time_ms;
    return ser_.writeLine(oss.str());
}

std::optional<char> SSC32U::queryStatus()
{
    if (!ser_.writeLine("Q"))
        return std::nullopt;
    auto r = ser_.readFor(50);
    if (r.find('.') != std::string::npos)
        return '.';
    if (r.find('+') != std::string::npos)
        return '+';
    return std::nullopt;
}

std::vector<int> SSC32U::queryPulse(const std::vector<uint8_t> &channels)
{
    std::ostringstream oss;
    oss << "QP";
    for (auto ch : channels)
        oss << " " << int(ch);
    ser_.writeLine(oss.str());
    auto r = ser_.readFor(50);
    std::vector<int> vals;
    vals.reserve(channels.size());
    for (unsigned i = 0; i < r.size() && i < channels.size(); ++i)
    {
        vals.push_back((unsigned char)r[i] * 10);
    }
    return vals;
}

std::optional<std::string> SSC32U::readDigital(bool latched = false)
{
    std::string cmd = latched ? "AL BL CL DL EL FL" : "A B C D E F";
    if (!ser_.writeLine(cmd))
        return std::nullopt;
    auto r = ser_.readFor(50);
    return trim(r);
}

std::optional<int> SSC32U::readInputA()
{
    if (!ser_.writeLine("A"))
        return std::nullopt;
    auto r = ser_.readFor(50);
    for (unsigned char c : r)
    {
        if (c == '0')
        return 0;
        if (c == '1')
        return 1;
    }
    return std::nullopt;
}

std::vector<int> SSC32U::readAnalog()
{
    ser_.writeLine("VA VB VC VD VE VF VG VH");
    auto r = ser_.readFor(50);
    std::vector<int> vals;
    vals.reserve(r.size());
    for (unsigned i = 0; i < r.size(); ++i)
        vals.push_back((unsigned char)r[i]);
    return vals;
}

std::optional<int> SSC32U::getBaudRegister()
{
    if (!ser_.writeLine("R4"))
        return std::nullopt;
    auto r = ser_.readFor(50);
    r = trim(r);
    if (r.empty())
        return std::nullopt;
    return std::stoi(r);
}

bool SSC32U::setBaudRegister(int baud)
{
    std::ostringstream oss;
    oss << "R4=" << (baud / 10);
    return ser_.writeLine(oss.str());
}