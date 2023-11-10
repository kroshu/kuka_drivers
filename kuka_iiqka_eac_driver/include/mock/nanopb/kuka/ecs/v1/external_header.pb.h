#ifndef PB_KUKA_ECS_V1_KUKA_ECS_V1_EXTERNAL_HEADER_PB_H_INCLUDED
#define PB_KUKA_ECS_V1_KUKA_ECS_V1_EXTERNAL_HEADER_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

typedef struct _kuka_ecs_v1_ExternalHeader {
    uint32_t message_id; 
    uint32_t ipoc; 
} kuka_ecs_v1_ExternalHeader;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define kuka_ecs_v1_ExternalHeader_init_default  {0, 0}
#define kuka_ecs_v1_ExternalHeader_init_zero     {0, 0}

/* Maximum encoded size of messages (where known) */
#define kuka_ecs_v1_ExternalHeader_size          12

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
