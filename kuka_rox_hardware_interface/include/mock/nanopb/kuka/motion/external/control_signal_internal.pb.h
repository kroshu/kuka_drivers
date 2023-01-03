#ifndef PB_KUKA_MOTION_EXTERNAL_KUKA_MOTION_EXTERNAL_CONTROL_SIGNAL_INTERNAL_PB_H_INCLUDED
#define PB_KUKA_MOTION_EXTERNAL_KUKA_MOTION_EXTERNAL_CONTROL_SIGNAL_INTERNAL_PB_H_INCLUDED
#include <pb.h>
#include <nanopb/kuka/core/motion/joint.pb.h>
#include <nanopb/kuka/motion/external/control_attributes.pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

typedef struct _kuka_motion_external_ControlSignalInternal {
    bool stop_ipo; 

    bool has_joint_command;
    kuka_core_motion_JointPositions joint_command; 

    bool has_cartesian_command;
    kuka_core_motion_JointPositions cartesian_command; 

    bool has_joint_torque_command;
    kuka_core_motion_JointPositions joint_torque_command; 

    bool has_wrench_command;
    kuka_core_motion_JointPositions wrench_command; 

    bool has_joint_attributes;
    kuka_motion_external_JointImpedanceControlAttributes joint_attributes; 

    bool has_cartesian_attributes;
    kuka_motion_external_CartesianImpedanceControlAttributes cartesian_attributes; 

    bool update_attributes; 
} kuka_motion_external_ControlSignalInternal;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define kuka_motion_external_ControlSignalInternal_init_default {0, false, kuka_core_motion_JointPositions_init_default, false, kuka_core_motion_JointPositions_init_default, false, kuka_core_motion_JointPositions_init_default, false, kuka_core_motion_JointPositions_init_default, false, kuka_motion_external_JointImpedanceControlAttributes_init_default, false, kuka_motion_external_CartesianImpedanceControlAttributes_init_default, 0}
#define kuka_motion_external_ControlSignalInternal_init_zero {0, false, kuka_core_motion_JointPositions_init_zero, false, kuka_core_motion_JointPositions_init_zero, false, kuka_core_motion_JointPositions_init_zero, false, kuka_core_motion_JointPositions_init_zero, false, kuka_motion_external_JointImpedanceControlAttributes_init_zero, false, kuka_motion_external_CartesianImpedanceControlAttributes_init_zero, 0}

/* Maximum encoded size of messages (where known) */
#if defined(kuka_core_motion_JointPositions_size) && defined(kuka_core_motion_CartesianTargetPose_size) && defined(kuka_core_motion_JointTorques_size)
#define kuka_motion_external_ControlSignalInternal_size (1384 + kuka_core_motion_JointPositions_size + kuka_core_motion_CartesianTargetPose_size + kuka_core_motion_JointTorques_size)
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
