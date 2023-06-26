#ifndef PB_KUKA_MOTION_EXTERNAL_KUKA_MOTION_EXTERNAL_CONTROL_SIGNAL_INTERNAL_PB_H_INCLUDED
#define PB_KUKA_MOTION_EXTERNAL_KUKA_MOTION_EXTERNAL_CONTROL_SIGNAL_INTERNAL_PB_H_INCLUDED
#include <pb.h>
#include <nanopb/kuka/core/motion/joint.pb.h>
#include <nanopb/kuka/motion/external/control_attributes.pb.h>
#include <nanopb/kuka/motion/external/external_control_mode.pb.h>


#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

typedef struct _kuka_core_geometry_Vector {
    double x; 
    double y; 
    double z; 
} kuka_core_geometry_Vector;

#define kuka_core_geometry_Vector_init_default   {0, 0, 0}
#define kuka_core_geometry_Vector_init_zero      {0, 0, 0}
#define kuka_core_geometry_Vector_size           27

typedef struct _kuka_core_geometry_Quaternion {
    double qx; 
    double qy; 
    double qz; 
    double qw; 
} kuka_core_geometry_Quaternion;

#define kuka_core_geometry_Quaternion_init_default {0, 0, 0, 0}
#define kuka_core_geometry_Quaternion_init_zero  {0, 0, 0, 0}
#define kuka_core_geometry_Quaternion_size       36

typedef struct _kuka_core_geometry_Transform { 
    bool has_translation;
    kuka_core_geometry_Vector translation; 
    bool has_rotation;
    kuka_core_geometry_Quaternion rotation; 
} kuka_core_geometry_Transform;

#define kuka_core_geometry_Transform_init_default {false, kuka_core_geometry_Vector_init_default, false, kuka_core_geometry_Quaternion_init_default}
#define kuka_core_geometry_Transform_init_zero   {false, kuka_core_geometry_Vector_init_zero, false, kuka_core_geometry_Quaternion_init_zero}

typedef struct _kuka_core_motion_Twist { 
    bool has_linear;
    kuka_core_geometry_Vector linear; 
    bool has_angular;
    kuka_core_geometry_Vector angular; 
} kuka_core_motion_Twist;

#define kuka_core_motion_Twist_init_default      {false, kuka_core_geometry_Vector_init_default, false, kuka_core_geometry_Vector_init_default}
#define kuka_core_motion_Twist_init_zero         {false, kuka_core_geometry_Vector_init_zero, false, kuka_core_geometry_Vector_init_zero}
#define kuka_core_motion_Twist_size              58


typedef struct _kuka_motion_external_ControlSignalInternal { 
    bool stop_ipo; 

    bool has_joint_command;
    kuka_core_motion_JointPositions joint_command; 

    bool has_cartesian_command;
    kuka_core_geometry_Transform cartesian_command; 

    bool has_joint_velocity_command;
    kuka_core_motion_JointPositions joint_velocity_command; 
  
    bool has_twist_command;
    kuka_core_motion_Twist twist_command; 

    bool has_joint_torque_command;
    kuka_core_motion_JointPositions joint_torque_command; 

    bool has_wrench_command;
    kuka_core_motion_JointPositions wrench_command; 

    bool has_joint_attributes;
    kuka_motion_external_JointImpedanceControlAttributes joint_attributes; 

    bool has_cartesian_attributes;
    kuka_motion_external_CartesianImpedanceControlAttributes cartesian_attributes; 

    kuka_motion_external_ExternalControlMode control_mode; 
} kuka_motion_external_ControlSignalInternal;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define kuka_motion_external_ControlSignalInternal_init_default {0, false, kuka_core_motion_JointPositions_init_default, false, kuka_core_geometry_Transform_init_default, false, kuka_core_motion_JointPositions_init_default, false, kuka_core_motion_Twist_init_default, false, kuka_core_motion_JointPositions_init_default, false, kuka_core_motion_JointPositions_init_default, false, kuka_motion_external_JointImpedanceControlAttributes_init_default, false, kuka_motion_external_CartesianImpedanceControlAttributes_init_default, _kuka_motion_external_ExternalControlMode_MIN}
#define kuka_motion_external_ControlSignalInternal_init_zero {0, false, kuka_core_motion_JointPositions_init_zero, false, kuka_core_geometry_Transform_init_zero, false, kuka_core_motion_JointPositions_init_zero, false, kuka_core_motion_Twist_init_zero, false, kuka_core_motion_JointPositions_init_zero, false, kuka_core_motion_JointPositions_init_zero, false, kuka_motion_external_JointImpedanceControlAttributes_init_zero, false, kuka_motion_external_CartesianImpedanceControlAttributes_init_zero, _kuka_motion_external_ExternalControlMode_MIN}


/* Maximum encoded size of messages (where known) */
#if defined(kuka_core_motion_JointPositions_size) && defined(kuka_core_motion_JointVelocities_size) && defined(kuka_core_motion_JointTorques_size)
#define kuka_motion_external_ControlSignalInternal_size (1513 + kuka_core_motion_JointPositions_size + kuka_core_motion_JointVelocities_size + kuka_core_motion_JointTorques_size)
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
