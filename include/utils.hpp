/*
 * Author: Jackson Allen
 * Filename: utils.hpp
 * 
 * */

 #include <iostream>
 #include <map>
 #include <cstdint>
 #include <csignal>

// Associates board channel number to function (for convenience when typing commands)
static const std::map<std::string, int> AL5D = {
    {"base", 0}, {"shoulder", 1}, {"elbow", 2}, {"wrist", 3}, {"wristRotate", 4}, {"gripper", 5}};

static constexpr uint8_t GRIPPER_CH = 5;

static void printHelp()
{
  std::cout <<
      R"(Commands:
  move <ch|name> <pw> [spd] [T<ms>]    # single-servo move, pw in microseconds (e.g., 500..2500)
  group <ch:pw[:spd]> ... [T<ms>]      # multiple channels, synchronized. example: group 0:1500 1:1600:500 2:750 T2000
  home                                 # center AL5D joints at 1500 microseconds
  status                               # returns '.' (idle) or '+' (moving)
  qp <ch> [<ch> ...]                   # query pulse widths (returns µs per channel)
  din [latched]                        # read digital inputs A-F (or AL-FL)
  ain                                  # read analog inputs VA-VH (0-255)
  grab [max_pw]                        # slowly close gripper on ch 5 at ~80us/s until switch closes, report PWM
  shutdown                             # home, disable pulses (P0) on ch 0-5, then close port & exit
  watch_switch [interval_ms]           # continuously monitor input A, print when it opens or closes
  baud get | set <rate>                # read/write baud via R4 register
  stop                                 # send T0 to stop a timed move
  help | quit
Named channels: base shoulder elbow wrist gripper wristRotate
)";
}

// --- Ctrl-C for the switch monitoring loop ---
static volatile std::sig_atomic_t g_stop_watch = 0;
static void sigint_handler(int) { g_stop_watch = 1; }