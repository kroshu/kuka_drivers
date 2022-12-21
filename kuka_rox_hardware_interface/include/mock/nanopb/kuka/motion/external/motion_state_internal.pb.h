#ifndef PB_KUKA_MOTION_EXTERNAL_KUKA_MOTION_EXTERNAL_MOTION_STATE_INTERNAL_PB_H_INCLUDED
#define PB_KUKA_MOTION_EXTERNAL_KUKA_MOTION_EXTERNAL_MOTION_STATE_INTERNAL_PB_H_INCLUDED
#include <pb.h>
#include <nanopb/kuka/core/motion/joint.pb.h>
#include <nanopb/kuka/motion/external/external_control_mode.pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
/* Internal message for SharedMem Communication between ecs and external sampler */
typedef struct _kuka_motion_external_MotionStateInternal { 
    /* Flag indicating whether the command handling has stopped */
    bool ipo_stopped; 
    /* Currently used control mode */
    kuka_motion_external_ExternalControlMode control_mode; 
    /* The measured joint position of the current cycle */
    bool has_measured_positions;
    kuka_core_motion_JointPositions measured_positions; 
    /* The measured joint torques of the current cycle */
    bool has_measured_torques;
    kuka_core_motion_JointPositions measured_torques; 
} kuka_motion_external_MotionStateInternal;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define kuka_motion_external_MotionStateInternal_init_default {0, _kuka_motion_external_ExternalControlMode_MIN, false, kuka_core_motion_JointPositions_init_default, false, kuka_core_motion_JointPositions_init_default}
#define kuka_motion_external_MotionStateInternal_init_zero {0, _kuka_motion_external_ExternalControlMode_MIN, false, kuka_core_motion_JointPositions_init_zero, false, kuka_core_motion_JointPositions_init_zero}

/* Maximum encoded size of messages (where known) */
#if defined(kuka_core_motion_JointPositions_size) && defined(kuka_core_motion_JointTorques_size)
#define kuka_motion_external_MotionStateInternal_size (16 + kuka_core_motion_JointPositions_size + kuka_core_motion_JointTorques_size)
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
