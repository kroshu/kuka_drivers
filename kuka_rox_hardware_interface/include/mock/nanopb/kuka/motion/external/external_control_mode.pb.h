#ifndef PB_KUKA_MOTION_EXTERNAL_KUKA_MOTION_EXTERNAL_EXTERNAL_CONTROL_MODE_PB_H_INCLUDED
#define PB_KUKA_MOTION_EXTERNAL_KUKA_MOTION_EXTERNAL_EXTERNAL_CONTROL_MODE_PB_H_INCLUDED
#include <pb.h>


#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

typedef enum _kuka_motion_external_ExternalControlMode
{
  kuka_motion_external_ExternalControlMode_EXTERNAL_CONTROL_MODE_UNSPECIFIED = 0,
  kuka_motion_external_ExternalControlMode_POSITION_CONTROL = 1,
  kuka_motion_external_ExternalControlMode_CARTESIAN_OVERLAY = 2,
  kuka_motion_external_ExternalControlMode_JOINT_IMPEDANCE_CONTROL = 3,
  kuka_motion_external_ExternalControlMode_CARTESIAN_IMPEDANCE_CONTROL = 4,
  kuka_motion_external_ExternalControlMode_TORQUE_CONTROL = 5,
  kuka_motion_external_ExternalControlMode_MR_VELOCITY_CONTROL = 6
} kuka_motion_external_ExternalControlMode;

/* Helper constants for enums */
#define _kuka_motion_external_ExternalControlMode_MIN \
  kuka_motion_external_ExternalControlMode_EXTERNAL_CONTROL_MODE_UNSPECIFIED
#define _kuka_motion_external_ExternalControlMode_MAX \
  kuka_motion_external_ExternalControlMode_MR_VELOCITY_CONTROL
#define _kuka_motion_external_ExternalControlMode_ARRAYSIZE (( \
    kuka_motion_external_ExternalControlMode)( \
    kuka_motion_external_ExternalControlMode_MR_VELOCITY_CONTROL + 1))


#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
