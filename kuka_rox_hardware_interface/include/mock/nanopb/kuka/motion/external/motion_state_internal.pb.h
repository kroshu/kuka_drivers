#ifndef PB_KUKA_MOTION_EXTERNAL_KUKA_MOTION_EXTERNAL_MOTION_STATE_INTERNAL_PB_H_INCLUDED
#define PB_KUKA_MOTION_EXTERNAL_KUKA_MOTION_EXTERNAL_MOTION_STATE_INTERNAL_PB_H_INCLUDED
#include <pb.h>
#include <nanopb/kuka/core/motion/joint.pb.h>
#include <nanopb/kuka/motion/external/external_control_mode.pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

typedef struct _kuka_motion_external_MotionStateInternal { 
  bool ipo_stopped;

  kuka_motion_external_ExternalControlMode control_mode;

  bool has_measured_positions;
  kuka_core_motion_JointPositions measured_positions;

    bool has_measured_velocities;
    kuka_core_motion_JointPositions measured_velocities; 

    bool has_measured_torques;
    kuka_core_motion_JointPositions measured_torques; 
} kuka_motion_external_MotionStateInternal;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define kuka_motion_external_MotionStateInternal_init_default {0, _kuka_motion_external_ExternalControlMode_MIN, false, kuka_core_motion_JointPositions_init_default, false, kuka_core_motion_JointPositions_init_default, false, kuka_core_motion_JointPositions_init_default}
#define kuka_motion_external_MotionStateInternal_init_zero {0, _kuka_motion_external_ExternalControlMode_MIN, false, kuka_core_motion_JointPositions_init_zero, false, kuka_core_motion_JointPositions_init_zero, false, kuka_core_motion_JointPositions_init_zero}

/* Maximum encoded size of messages (where known) */
#if defined(kuka_core_motion_JointPositions_size) && defined(kuka_core_motion_JointVelocities_size) && defined(kuka_core_motion_JointTorques_size)
#define kuka_motion_external_MotionStateInternal_size (22 + kuka_core_motion_JointPositions_size + kuka_core_motion_JointVelocities_size + kuka_core_motion_JointTorques_size)
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
