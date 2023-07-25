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

  handler.GetState().operator<<(std::cout) << std::endl;
  bool param;
  handler.GetState().GetElement("Out").GetParam<bool>("01", param);

  std::cout << "encode data fill" << std::endl;
  std::string msg = "KROSHU";
  xml::XMLString xmlStr(msg);
  handler.SetCommandParam<xml::XMLString>("Sen", "Type", xmlStr);
  double x = 5.2;
  double y = 6;
  double z = 3.2;
  double a = 12;
  double b = 7;
  double c = 4.35;
  long DiO = 12012141212323123;
  handler.SetCommandParam<double>("RKorr", "X", x);
  handler.SetCommandParam<double>("RKorr", "Y", y);
  handler.SetCommandParam<double>("RKorr", "Z", z);
  handler.SetCommandParam<double>("RKorr", "A", a);
  handler.SetCommandParam<double>("RKorr", "B", b);
  handler.SetCommandParam<double>("RKorr", "C", c);
  handler.SetCommandParam<long>("DiO", "DiO", DiO);

  handler.GetCommand().operator<<(std::cout) << std::endl;

  return 0;
}
