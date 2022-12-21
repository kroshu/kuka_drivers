#ifndef PB_KUKA_ECS_V1_KUKA_ECS_V1_MOTION_SERVICES_ECS_PB_H_INCLUDED
#define PB_KUKA_ECS_V1_KUKA_ECS_V1_MOTION_SERVICES_ECS_PB_H_INCLUDED
#include <pb.h>
#include <nanopb/kuka/ecs/v1/qos_profile_settings.pb.h>
#include <nanopb/kuka/motion/external/external_control_mode.pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
/* Command Event */
typedef enum _kuka_ecs_v1_CommandEvent { 
    kuka_ecs_v1_CommandEvent_CONTROL_EVENT_UNSPECIFIED = 0, 
    /* Idle */
    kuka_ecs_v1_CommandEvent_IDLE = 1, 
    /* Preparation finishes for motion command
 (= MotionCommand ExtendedStateValue transitions to kReady) */
    kuka_ecs_v1_CommandEvent_COMMAND_READY = 2, 
    /* Sampler starts sampling
 (= MotionCommand ExtendedStateValue transitions to kExecuting) */
    kuka_ecs_v1_CommandEvent_SAMPLING = 3, 
    /* Motion command execution finishes successfully
 (= MotionCommand ExtendedStateValue transitions to kSamplingFinished) */
    kuka_ecs_v1_CommandEvent_STOPPED = 4, 
    /* Error occurs during external control, possible reasons:
   - motion command is rejected or recalled
   - motion command preparation fails
   - unhandled error during execution (ExtendedStateValue = kExecutionFailed)
   - cancelAll() is called for session
   - session is suspended or terminated
   - connection quality drops below the used QoS profile specification */
    kuka_ecs_v1_CommandEvent_ERROR = 6 
} kuka_ecs_v1_CommandEvent;


/* Helper constants for enums */
#define _kuka_ecs_v1_CommandEvent_MIN kuka_ecs_v1_CommandEvent_CONTROL_EVENT_UNSPECIFIED
#define _kuka_ecs_v1_CommandEvent_MAX kuka_ecs_v1_CommandEvent_ERROR
#define _kuka_ecs_v1_CommandEvent_ARRAYSIZE ((kuka_ecs_v1_CommandEvent)(kuka_ecs_v1_CommandEvent_ERROR+1))


#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
