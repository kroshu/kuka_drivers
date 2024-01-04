#ifndef PB_KUKA_MOTION_EXTERNAL_KUKA_MOTION_EXTERNAL_CONTROL_ATTRIBUTES_PB_H_INCLUDED
#define PB_KUKA_MOTION_EXTERNAL_KUKA_MOTION_EXTERNAL_CONTROL_ATTRIBUTES_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

typedef struct _kuka_motion_external_CartesianImpedanceControlAttributes {
    pb_size_t stiffness_count;
    double stiffness[24];

    pb_size_t damping_count;
    double damping[24];

    pb_size_t nullspace_stiffness_count;
    double nullspace_stiffness[24];

    pb_size_t nullspace_damping_count;
    double nullspace_damping[24];
} kuka_motion_external_CartesianImpedanceControlAttributes;

typedef struct _kuka_motion_external_JointImpedanceControlAttributes {
    pb_size_t stiffness_count;
    double stiffness[24];

    pb_size_t damping_count;
    double damping[24];
} kuka_motion_external_JointImpedanceControlAttributes;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define kuka_motion_external_JointImpedanceControlAttributes_init_default {0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
#define kuka_motion_external_CartesianImpedanceControlAttributes_init_default {0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
#define kuka_motion_external_JointImpedanceControlAttributes_init_zero {0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
#define kuka_motion_external_CartesianImpedanceControlAttributes_init_zero {0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}

/* Maximum encoded size of messages (where known) */
#define kuka_motion_external_CartesianImpedanceControlAttributes_size 864
#define kuka_motion_external_JointImpedanceControlAttributes_size 432

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
