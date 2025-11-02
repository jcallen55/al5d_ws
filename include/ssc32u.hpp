/*
 * Author: Jackson Allen
 * Filename: ssc32u.hpp
 *
 * */

#include <vector>
#include <sstream>
#include <optional>
#include <ssc32u_serial.hpp>

#ifndef SSC32U_HPP
#define SSC32U_HPP

static std::string trim(std::string s);

// --- CLASS SSC32U - virtual board object, responsible for formatting and initiating comms with SSC-32U ---
class SSC32U
{
    Serial &ser_;

public:
    explicit SSC32U(Serial &s) : ser_(s) {}

    bool move(uint8_t ch, int pw, std::optional<int> spd = std::nullopt, std::optional<int> time_ms = std::nullopt);

    bool groupMove(const std::vector<std::tuple<uint8_t, int, std::optional<int>>> &items, std::optional<int> time_ms = std::nullopt);

    std::optional<char> queryStatus();

    std::vector<int> queryPulse(const std::vector<uint8_t> &channels);

    std::optional<std::string> readDigital(bool latched = false);

    // read only input A (returns 0 or 1)
    std::optional<int> readInputA();

    std::vector<int> readAnalog();

    std::optional<int> getBaudRegister();

    bool setBaudRegister(int baud);
};

#endif // SSC32U_HPP