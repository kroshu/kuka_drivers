// This material is the exclusive property of KUKA Deutschland GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Deutschland GmbH for
// internal development purposes of KUKA Deutschland GmbH.
//
// Copyright (C)
// KUKA Deutschland GmbH, Germany. All Rights Reserved.

#include "os-core-udp-communication/secure_replier.h"

namespace os::core::udp::communication
{

SecureReplier::SecureReplier(const std::string& certificate_path, const std::string& private_key_path, const SocketAddress & local_address)
: Replier(local_address) {}


}  // namespace os::core::udp::communication
