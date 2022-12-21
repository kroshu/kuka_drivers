#ifndef PB_KUKA_ECS_V1_KUKA_ECS_V1_CONTROL_SIGNAL_EXTERNAL_PB_H_INCLUDED
#define PB_KUKA_ECS_V1_KUKA_ECS_V1_CONTROL_SIGNAL_EXTERNAL_PB_H_INCLUDED
#include <pb.h>
#include <nanopb/kuka/ecs/v1/external_header.pb.h>
#include <nanopb/kuka/motion/external/control_signal_internal.pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
/* Proto definition for external RT commands */
typedef struct _kuka_ecs_v1_ControlSignalExternal { 
    /* Message Header */
    bool has_header;
    kuka_ecs_v1_ExternalHeader header; 
    /* Internal control signal */
    bool has_control_signal;
    kuka_motion_external_ControlSignalInternal control_signal;
} kuka_ecs_v1_ControlSignalExternal;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define kuka_ecs_v1_ControlSignalExternal_init_default {false, kuka_ecs_v1_ExternalHeader_init_default, false, kuka_motion_external_ControlSignalInternal_init_default}
#define kuka_ecs_v1_ControlSignalExternal_init_zero {false, kuka_ecs_v1_ExternalHeader_init_zero, false, kuka_motion_external_ControlSignalInternal_init_zero}


/* Maximum encoded size of messages (where known) */
#if defined(kuka_motion_external_ControlSignalInternal_size)
#define kuka_ecs_v1_ControlSignalExternal_size   (20 + kuka_motion_external_ControlSignalInternal_size)
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
