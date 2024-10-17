#ifndef FRI__HWIFCLIENTAPPLICATION_HPP_
#define FRI__HWIFCLIENTAPPLICATION_HPP_

#include <string>

#include <fri_client_sdk/friClientApplication.h>
#include <fri_client_sdk/friConnectionIf.h>
#include <fri_client_sdk/friClientIf.h>
#include <fri_client_sdk/friTransformationClient.h>
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
