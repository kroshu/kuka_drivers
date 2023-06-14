// This material is the exclusive property of KUKA Deutschland GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Deutschland GmbH for
// internal development purposes of KUKA Deutschland GmbH.
//
// Copyright (C)
// KUKA Deutschland GmbH, Germany. All Rights Reserved.

#ifndef UDP_SecureReplier_H
#define UDP_SecureReplier_H

#include <chrono>

#include "os-core-udp-communication/replier.h"

namespace os::core::udp::communication
{

class SecureReplier : public Replier
{
public:
  //<ctor>
  SecureReplier(
    const std::string & certificate_path, const std::string & private_key_path,
    const SocketAddress & local_address);
  virtual ~SecureReplier() = default;
};
}  // namespace os::core::udp::communication

#endif  // UDP_SecureReplier_H
