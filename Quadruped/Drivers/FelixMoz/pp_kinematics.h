#ifndef PP_KINEMATICS_H
#define PP_KINEMATICS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stddef.h>  // for NULL

/* Joint angles for 2-DOF planar leg (femur/thigh pitch, tibia/shin pitch) */
typedef struct {
    float femur;  /* First pitch joint (shoulder/hip pitch) */
    float tibia;  /* Second pitch joint (elbow/knee pitch) */
} PP_JointAngles;

/* 2D coordinate in leg plane (horizontal distance from hip, vertical depth) */
typedef struct {
    float x;      /* Horizontal reach from hip joint (meters) */
    float z;      /* Vertical position, positive = downward (meters) */
} PP_Point2D;

/* Unit normal vector representing suction pad orientation in 2D plane */
typedef struct {
    float x;      /* Horizontal component of normal vector */
    float z;      /* Vertical component of normal vector */
} PP_Normal2D;

/* Kinematic parameters for 2-link planar manipulator */
typedef struct {
    float L1;     /* Femur (upper leg) length in meters */
    float L2;     /* Tibia (lower leg) length in meters */
    float theta1_min, theta1_max;  /* Femur joint limits in radians */
    float theta2_min, theta2_max;  /* Tibia joint limits in radians */
} PP_LegParams;

/* Initialize with default leg dimensions and joint limits */
void PP_InitParams(PP_LegParams* p);

/* Forward kinematics: joint angles -> foot position + pad normal vector */
void PP_Forward(const PP_LegParams* p, const PP_JointAngles* ang, 
                PP_Point2D* foot_pos, PP_Normal2D* pad_normal);

/* Inverse kinematics - position only: solves for joint angles given target position.
   elbow_up selects between elbow-forward (false) or elbow-backward (true) solution */
bool PP_Inverse_Pos(const PP_LegParams* p, const PP_Point2D* target,
                    bool elbow_up, PP_JointAngles* sol);

/* Constrained inverse kinematics for suction cup climbing: solves for position
   and checks if resulting pad orientation matches target normal within tolerance.
   Returns error angle in degrees via error_deg pointer if solution found */
bool PP_Inverse_Pose(const PP_LegParams* p, const PP_Point2D* target,
                     const PP_Normal2D* target_normal, 
                     PP_JointAngles* sol, float* error_deg);

/* Convert radians to HT-85H servo value (0-1000 range).
   offset_rad accounts for mechanical zero-position calibration */
uint16_t PP_AngleToServo(float rad, float offset_rad);

#endif /* PP_KINEMATICS_H */
					 