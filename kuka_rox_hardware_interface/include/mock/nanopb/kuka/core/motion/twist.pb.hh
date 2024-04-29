#ifndef PB_KUKA_CORE_MOTION_TWIST_PB_HH_INCLUDED
#define PB_KUKA_CORE_MOTION_TWIST_PB_HH_INCLUDED

#include <functional>
#include "nanopb/encoding.hh"
#include "nanopb/kuka/core/motion/twist.pb.h"

namespace nanopb::kuka::core::motion {

using Twist = kuka_core_motion_Twist;

constexpr Twist Twist_init_default kuka_core_motion_Twist_init_default;

constexpr const pb_msgdesc_t* Twist_fields = kuka_core_motion_Twist_fields;

} // namespace nanopb::kuka::core::motion


namespace nanopb {
} // namespace nanopb


namespace nanopb::encoding {

template<> inline constexpr const char* GetTypeUrl<nanopb::kuka::core::motion::Twist>() {
 return "/kuka.core.motion.Twist";
}

template<> inline constexpr const pb_msgdesc_t* GetMessageDescription<nanopb::kuka::core::motion::Twist>() {
 return kuka_core_motion_Twist_fields;
}
} // namespace nanopb::encoding


namespace std {
} // namespace std

#endif //  PB_KUKA_CORE_MOTION_TWIST_PB_HH_INCLUDED
