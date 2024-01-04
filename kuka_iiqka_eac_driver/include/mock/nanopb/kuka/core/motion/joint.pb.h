#ifndef PB_KUKA_CORE_MOTION_KUKA_CORE_MOTION_JOINT_PB_H_INCLUDED
#define PB_KUKA_CORE_MOTION_KUKA_CORE_MOTION_JOINT_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* An array of joint positions for a given kinematic structure. */
typedef struct _kuka_core_motion_JointPositions {
    pb_size_t values_count;
    double values[24];
} kuka_core_motion_JointPositions;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define kuka_core_motion_JointPositions_init_default {0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
#define kuka_core_motion_JointPositions_init_zero {0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}

/* Maximum encoded size of messages (where known) */
#define kuka_core_motion_JointPositions_size     216

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
