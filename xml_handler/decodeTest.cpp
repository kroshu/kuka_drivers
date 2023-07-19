#include "rsi_command_handler.hpp"

#include <cstring>

int main(int argc, char const * argv[])
{
  kuka_rsi_hw_interface::RSICommandHandler handler;
  char xml_state[1024] =
  {
    "<Rob TYPE=\"KUKA\"><Out 01=\"0\" 02=\"1\" 03=\"0\" 04=\"0\" 05=\"1\"/><Override>55</Override></Rob>"};
  if (!handler.Decode(xml_state, 1024)) {
    std::cout << "decode failed" << std::endl;
    return -1;
  }


  std::cout << "decode successful" << std::endl;

  handler.state_data_structure_.operator<<(std::cout) << std::endl;

  return 0;
}
