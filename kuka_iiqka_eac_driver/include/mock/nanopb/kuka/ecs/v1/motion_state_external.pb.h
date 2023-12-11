#ifndef PB_KUKA_ECS_V1_KUKA_ECS_V1_MOTION_STATE_EXTERNAL_PB_H_INCLUDED
#define PB_KUKA_ECS_V1_KUKA_ECS_V1_MOTION_STATE_EXTERNAL_PB_H_INCLUDED
#include <pb.h>
#include <nanopb/kuka/ecs/v1/external_header.pb.h>
#include <nanopb/kuka/motion/external/motion_state_internal.pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

typedef struct _kuka_ecs_v1_MotionStateExternal {
    bool has_header;
    kuka_ecs_v1_ExternalHeader header;

    bool has_motion_state;
    kuka_motion_external_MotionStateInternal motion_state;
} kuka_ecs_v1_MotionStateExternal;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define kuka_ecs_v1_MotionStateExternal_init_default {false, kuka_ecs_v1_ExternalHeader_init_default, false, kuka_motion_external_MotionStateInternal_init_default}
#define kuka_ecs_v1_MotionStateExternal_init_zero {false, kuka_ecs_v1_ExternalHeader_init_zero, false, kuka_motion_external_MotionStateInternal_init_zero}

/* Maximum encoded size of messages (where known) */
#if defined(kuka_motion_external_MotionStateInternal_size)
#define kuka_ecs_v1_MotionStateExternal_size     (20 + kuka_motion_external_MotionStateInternal_size)
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
