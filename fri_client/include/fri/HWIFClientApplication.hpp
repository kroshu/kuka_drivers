#ifndef FRI__HWIFCLIENTAPPLICATION_HPP_
#define FRI__HWIFCLIENTAPPLICATION_HPP_

#include <string>

#include <fri/friClientApplication.h>
#include <fri/friConnectionIf.h>
#include <fri/friClientIf.h>
#include <fri/friTransformationClient.h>
#include <friClientData.h>


namespace KUKA
{
namespace FRI
{

class HWIFClientApplication : public ClientApplication
{
public:
  HWIFClientApplication(IConnection & connection, IClient & client);

  bool client_app_read();
  void client_app_update();
  bool client_app_write();

private:
  int size_;
};

}
}  // namespace KUKA::FRI

#endif  // FRI__HWIFCLIENTAPPLICATION_HPP_
