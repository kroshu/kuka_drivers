#ifndef PB_KUKA_CORE_MOTION_JOINT_PB_HH_INCLUDED
#define PB_KUKA_CORE_MOTION_JOINT_PB_HH_INCLUDED

#include <functional>
#include "nanopb/kuka/core/motion/joint.pb.h"

namespace nanopb::kuka::core::motion {

using JointPositions = kuka_core_motion_JointPositions;


constexpr JointPositions JointPositions_init_default kuka_core_motion_JointPositions_init_default;

} // namespace nanopb::kuka::core::motion

#endif //  PB_KUKA_CORE_MOTION_JOINT_PB_HH_INCLUDED

