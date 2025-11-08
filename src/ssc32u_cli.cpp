/*
 * Author: Jackson Allen
 * Filename: ssc32u_cli.cpp
 * 
 * */

#include <algorithm>
#include <vector>
#include "Ssc32uSerial.hpp"
#include "utils.hpp"

// MAIN
int main(int argc, char **argv)
{
  std::string port = "/dev/ttyUSB0";
  unsigned baud = 115200;
  const std::vector<unsigned> valid_bauds = {9600, 38400, 115200, 0};

  for (int i = 1; i < argc; i++)
  {
    std::string a = argv[i];
    if (a == "--port" && i + 1 < argc)
    {
      port = argv[++i];
    }
    else if (a == "--baud" && i + 1 < argc)
    {
      baud = (unsigned)std::stoul(argv[++i]);
      if (std::find(valid_bauds.begin(), valid_bauds.end(), baud) == valid_bauds.end())
      {
          std::cout << "Non-standard baud rate specified: " << baud << "\n";
      }
    }
    else if (a == "--help" || a == "-h")
    {
      std::cout << "Usage: " << argv[0] << " --port /dev/ttyUSB0 [--baud 9600]\n";
      return 0;
    }
  }

  std::cout << "USB connection configuration specified: " << baud << " on port " << port << " --> Attempting to establish USB connection...\n";

  Ssc32uSerial serial;
  if (!serial.open(port, baud))
  {
    std::cerr << "USB connection failed. (" << port << ")\n";
    return 1;
  }
  else
  {
    std::cout << "USB connection established.\n";
    return 0;
  }
}

// SSC32U ssc(ser);

//   std::cout << "Connected to " << port << " @ " << baud << " baud.\n";
//   printHelp();

//   std::string line;
//   while (true)
//   {
//     std::cout << "> " << std::flush;
//     if (!std::getline(std::cin, line))
//       break;
//     line = trim(line);
//     if (line.empty())
//       continue;

//     std::istringstream iss(line);
//     std::string cmd;
//     iss >> cmd;

//     if (cmd == "quit" || cmd == "exit")
//       break;
//     if (cmd == "help")
//     {
//       printHelp();
//       continue;
//     }

//     if (cmd == "move")
//     {
//       std::string chs;
//       int pw;
//       std::string spdstr;
//       std::optional<int> spd, tms;
//       iss >> chs >> pw;
//       if (iss >> spdstr)
//       {
//         if (spdstr.rfind("T", 0) == 0)
//         {
//           tms = std::stoi(spdstr.substr(1));
//         }
//         else
//         {
//           spd = std::stoi(spdstr);
//           std::string t;
//           if (iss >> t && t.rfind("T", 0) == 0)
//             tms = std::stoi(t.substr(1));
//         }
//       }
//       int ch = -1;
//       if (AL5D.count(chs))
//         ch = AL5D.at(chs);
//       else
//         ch = std::stoi(chs);
//       if (!ssc.move((uint8_t)ch, pw, spd, tms))
//         std::cerr << "write failed\n";
//       continue;
//     }

//     if (cmd == "group")
//     {
//       std::vector<std::tuple<uint8_t, int, std::optional<int>>> items;
//       std::optional<int> tms;
//       std::string tok;
//       while (iss >> tok)
//       {
//         if (tok.size() > 1 && tok[0] == 'T')
//         {
//           tms = std::stoi(tok.substr(1));
//           continue;
//         }
//         auto a = tok.find(':');
//         if (a == std::string::npos)
//         {
//           std::cerr << "Bad token: " << tok << "\n";
//           continue;
//         }
//         auto b = tok.find(':', a + 1);
//         std::string chs = tok.substr(0, a);
//         int ch = AL5D.count(chs) ? AL5D.at(chs) : std::stoi(chs);
//         int pw = std::stoi(tok.substr(a + 1, b == std::string::npos ? std::string::npos : b - (a + 1)));
//         std::optional<int> spd;
//         if (b != std::string::npos)
//           spd = std::stoi(tok.substr(b + 1));
//         items.emplace_back((uint8_t)ch, pw, spd);
//       }
//       if (!ssc.groupMove(items, tms))
//         std::cerr << "write failed\n";
//       continue;
//     }

//     if (cmd == "home")
//     {
//       std::vector<std::tuple<uint8_t, int, std::optional<int>>> items;
//       for (auto &kv : AL5D)
//         items.emplace_back((uint8_t)kv.second, 1500, std::nullopt);
//       ssc.groupMove(items, 1000);
//       continue;
//     }

//     if (cmd == "status")
//     {
//       auto st = ssc.queryStatus();
//       if (!st)
//         std::cout << "(no response)\n";
//       else
//         std::cout << *st << "\n";
//       continue;
//     }

//     if (cmd == "qp")
//     {
//       std::vector<uint8_t> chs;
//       std::string tok;
//       while (iss >> tok)
//       {
//         int ch = AL5D.count(tok) ? AL5D.at(tok) : std::stoi(tok);
//         chs.push_back((uint8_t)ch);
//       }
//       if (chs.empty())
//       {
//         std::cerr << "usage: qp <ch> [<ch>...]\n";
//         continue;
//       }
//       auto vals = ssc.queryPulse(chs);
//       for (size_t i = 0; i < vals.size(); ++i)
//       {
//         std::cout << int(chs[i]) << ":" << vals[i] << "us ";
//       }
//       std::cout << "\n";
//       continue;
//     }

//     if (cmd == "din")
//     {
//       std::string opt;
//       iss >> opt;
//       bool latched = (opt == "latched");
//       auto r = ssc.readDigital(latched);
//       if (!r)
//         std::cout << "(no response)\n";
//       else
//         std::cout << *r << "\n";
//       continue;
//     }

//     if (cmd == "ain")
//     {
//       auto v = ssc.readAnalog();
//       if (v.empty())
//       {
//         std::cout << "(no response)\n";
//         continue;
//       }
//       std::cout << "A..H:";
//       for (auto x : v)
//         std::cout << " " << x;
//       std::cout << "\n";
//       continue;
//     }

//     if (cmd == "shutdown")
//     {
//       std::vector<uint8_t> chs = {0, 1, 2, 3, 4, 5};

//       // read current pulses and send a normal "#ch P<pw>" group
//       auto cur = ssc.queryPulse(chs);
//       if (!cur.empty() && cur.size() == chs.size())
//       {
//         std::vector<std::tuple<uint8_t, int, std::optional<int>>> prime;
//         for (size_t i = 0; i < chs.size(); ++i)
//           prime.emplace_back(chs[i], cur[i], std::nullopt);
//         ssc.groupMove(prime, std::nullopt);
//       }

//       // send reset position "#ch P1500 S20" group (all servos to middle position at 1500us)
//       const int SPEED_US_PER_S = 100; // 100 µs/s
//       std::vector<std::tuple<uint8_t, int, std::optional<int>>> home;
//       for (auto ch : chs)
//         home.emplace_back(ch, 1500, SPEED_US_PER_S);
//       if (!ssc.groupMove(home, std::nullopt))
//       {
//         std::cerr << "Failed to write shutdown move.\n";
//       }
//       else
//       {
//         std::cout << "Shutdown: returning to home at 100 µs/s...\n";
//       }

//       // Wait until motion completes (Q -> .)
//       for (;;)
//       {
//         auto st = ssc.queryStatus();
//         if (st && *st == '.')
//           break;
//         std::this_thread::sleep_for(std::chrono::milliseconds(250));
//       }
//       std::cout << "Shutdown: home reached. Closing serial and exiting.\n";

//       // Disable pulses to remove holding torque ("#<ch> P0" on all channels)
//       for (auto ch : chs)
//       {
//         std::ostringstream oss;
//         oss << "#" << int(ch) << " P0";
//         ser.writeLine(oss.str());
//         std::this_thread::sleep_for(std::chrono::milliseconds(5)); // tiny pacing
//       }
//       std::cout << "Shutdown: home reached. Servo pulses disabled (signal low).\n";

//       // Close port and exit program
//       ser.close();
//       return 0;
//     }

//     // Automated gripping sequence for testing tactile sensing mods
//     if (cmd == "grab")
//     {
//       // --- Assume standard position for grab demo ---
//       std::vector<uint8_t> chs = {0, 1, 2, 3, 4, 5};

//       // send "#ch P1500 S20" group (assume mid-positions again)
//       const int SPEED_US_PER_S = 100; // 100 µs/s
//       std::vector<std::tuple<uint8_t, int, std::optional<int>>> home;
//       for (auto ch : chs)
//         home.emplace_back(ch, 1500, SPEED_US_PER_S);
//       if (!ssc.groupMove(home, std::nullopt))
//       {
//         std::cerr << "Failed to write grab homing move.\n";
//       }
//       else
//       {
//         std::cout << "Returning to home at 100 µs/s...\n";
//       }

//       // Wait until motion completes (Q -> .)
//       for (;;) // FORFUTREF: infinite loop shorthand
//       {
//         auto st = ssc.queryStatus();
//         if (st && *st == '.')
//           break;
//         std::this_thread::sleep_for(std::chrono::milliseconds(250));
//       }

//       // --- send move commands for individual servos for grab demo ---
//       // Base
//       if (!ssc.move(0, 1500, SPEED_US_PER_S, std::nullopt))
//       {
//         std::cerr << "Failed to write Servo 0 'move' to grab demo position.";
//       }
//       else
//       {
//         std::cout << "Servo 0 positioned. Ready." << "\n";
//       }
//       // Wait until motion completes
//       for (;;)
//       {
//         auto st = ssc.queryStatus();
//         if (st && *st == '.')
//           break;
//         std::this_thread::sleep_for(std::chrono::milliseconds(250));
//       }
//       // Shoulder
//       if (!ssc.move(1, 1600, SPEED_US_PER_S, std::nullopt))
//       {
//         std::cerr << "Failed to write Servo 1 'move' to grab demo position.";
//       }
//       else
//       {
//         std::cout << "Servo 1 positioned. Ready." << "\n";
//       }
//       for (;;)
//       {
//         auto st = ssc.queryStatus();
//         if (st && *st == '.')
//           break;
//         std::this_thread::sleep_for(std::chrono::milliseconds(250));
//       }
//       // Elbow
//       if (!ssc.move(2, 1650, SPEED_US_PER_S, std::nullopt))
//       {
//         std::cerr << "Failed to write Servo 2 'move' to grab demo position.";
//       }
//       else
//       {
//         std::cout << "Servo 2 positioned. Ready." << "\n";
//       }
//       for (;;)
//       {
//         auto st = ssc.queryStatus();
//         if (st && *st == '.')
//           break;
//         std::this_thread::sleep_for(std::chrono::milliseconds(250));
//       }
//       // Wrist
//       if (!ssc.move(3, 700, SPEED_US_PER_S, std::nullopt))
//       {
//         std::cerr << "Failed to write Servo 3 'move' to grab demo position.";
//       }
//       else
//       {
//         std::cout << "Servo 3 positioned. Ready." << "\n";
//       }
//       for (;;)
//       {
//         auto st = ssc.queryStatus();
//         if (st && *st == '.')
//           break;
//         std::this_thread::sleep_for(std::chrono::milliseconds(250));
//       }
//       // Write Rotate
//       if (!ssc.move(4, 1500, SPEED_US_PER_S, std::nullopt))
//       {
//         std::cerr << "Failed to write Servo 4 'move' to grab demo position.";
//       }
//       else
//       {
//         std::cout << "Servo 4 positioned. Ready." << "\n";
//       }
//       for (;;)
//       {
//         auto st = ssc.queryStatus();
//         if (st && *st == '.')
//           break;
//         std::this_thread::sleep_for(std::chrono::milliseconds(250));
//       }
//       // Gripper
//       if (!ssc.move(5, 1100, SPEED_US_PER_S, std::nullopt))
//       {
//         std::cerr << "Failed to write Servo 5 'move' to grab demo position.";
//       }
//       else
//       {
//         std::cout << "Servo 5 positioned. Ready." << "\n";
//       }
//       for (;;)
//       {
//         auto st = ssc.queryStatus();
//         if (st && *st == '.')
//           break;
//         std::this_thread::sleep_for(std::chrono::milliseconds(250));
//       }

//       // --- begin Grab Demo ---
//       // parameters
//       int max_pw = 2500;
//       if (iss.good())
//       {
//         int tmp;
//         if (iss >> tmp)
//           max_pw = std::clamp(tmp, 500, 2500);
//       }
//       const int step_us = 4;      // 1 µs per step
//       const int interval_ms = 50; // 50 ms per step  ->  4us/50ms = ~??? µs/s

//       // Check current switch position
//       auto sw = ssc.readInputA();
//       if (!sw)
//       {
//         std::cout << "(no response reading input A)\n";
//         continue;
//       }
//       if (*sw == 0)
//       {
//         std::cout << "Switch is already CLOSED; aborting grab to avoid crushing.\n";
//         continue;
//       }

//       // Get current gripper PWM (use 1500 if for some reason not available)
//       auto curv = ssc.queryPulse({GRIPPER_CH});
//       int pw = (!curv.empty() ? curv[0] : 1500);
//       pw = std::clamp(pw, 500, max_pw);
//       std::cout << "Starting grab on ch " << int(GRIPPER_CH)
//                 << " at " << pw << "us, target <= " << max_pw
//                 << " at ~20us/s; monitoring switch A (1=open, 0=closed)…\n";

//       // Start monitoring switch position
//       int stopped_at = pw;
//       bool tripped = false;
//       while (pw < max_pw)
//       {
//         // check the switch
//         auto s = ssc.readInputA();
//         if (s && *s == 0)
//         { // transition detected
//           tripped = true;
//           stopped_at = pw;
//           // Hold current position
//           std::ostringstream hold;
//           hold << "#" << int(GRIPPER_CH) << " P" << pw;
//           if (!ser.writeLine(hold.str()))
//             std::cerr << "Warn: write hold failed\n";
//           break;
//         }

//         // Advance 1 µs toward fully closed
//         pw += step_us;
//         std::ostringstream oss;
//         oss << "#" << int(GRIPPER_CH) << " P" << pw;
//         if (!ser.writeLine(oss.str()))
//         {
//           std::cerr << "Error: write failed\n";
//           break;
//         }
//         stopped_at = pw;

//         std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
//       }

//       // Print results of the demo and gripper pwm if switch triggered
//       if (tripped)
//       {
//         std::cout << "Switch CLOSED detected. Stopped gripper at " << stopped_at << "us.\n";
//       }
//       else if (pw >= max_pw)
//       {
//         std::cout << "Reached max_pw (" << max_pw << "us) without switch closure.\n";
//       }
//       else
//       {
//         std::cout << "Grab aborted due to write/IO issue. Last PWM: " << stopped_at << "us.\n";
//       }
//       continue;
//     }

//     if (cmd == "watch_switch")
//     {
//       int interval_ms = 25;
//       if (iss.good())
//       {
//         int tmp;
//         if (iss >> tmp)
//           interval_ms = std::max(5, tmp);
//       }
//       std::cout << "Monitoring input A (1=open, 0=closed). Press Ctrl-C to stop.\n";
//       g_stop_watch = 0;
//       auto prev_handler = std::signal(SIGINT, sigint_handler);

//       auto st = ssc.readInputA();
//       if (!st)
//       {
//         std::cout << "(no response)\n";
//         std::signal(SIGINT, prev_handler);
//         continue;
//       }
//       int last = *st;
//       std::cout << "Initial state: " << (last ? "OPEN" : "CLOSED") << "\n";

//       while (!g_stop_watch)
//       {
//         auto s = ssc.readInputA();
//         if (s)
//         {
//           if (*s != last)
//           {
//             if (*s == 1)
//               std::cout << "Switch OPENED\n";
//             else
//               std::cout << "Switch CLOSED\n";
//             last = *s;
//           }
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
//       }
//       std::cout << "\n(stop) returning to prompt.\n";
//       std::signal(SIGINT, prev_handler);
//       continue;
//     }

//     if (cmd == "baud")
//     {
//       std::string sub;
//       iss >> sub;
//       if (sub == "get")
//       {
//         auto r4 = ssc.getBaudRegister();
//         if (!r4)
//           std::cout << "(no response)\n";
//         else
//           std::cout << "R4=" << *r4 << " -> approx " << (*r4) * 10 << " baud\n";
//       }
//       else if (sub == "set")
//       {
//         int rate;
//         iss >> rate;
//         if (!iss)
//         {
//           std::cerr << "baud set <rate>\n";
//           continue;
//         }
//         if (!ssc.setBaudRegister(rate))
//           std::cerr << "write failed\n";
//         else
//           std::cout << "Wrote R4=" << (rate / 10) << " (power-cycle or reconnect at new rate)\n";
//       }
//       else
//       {
//         std::cout << "baud get | set <rate>\n";
//       }
//       continue;
//     }

//     if (cmd == "stop")
//     {
//       ser.writeLine("T0");
//       continue;
//     }

//     std::cout << "Unknown command. Type 'help'.\n";
//   }
//   return 0;
// }
