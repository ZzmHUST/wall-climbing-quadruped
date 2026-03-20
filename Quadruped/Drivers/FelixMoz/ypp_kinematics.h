#ifndef YPP_KINEMATICS_H
#define YPP_KINEMATICS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stddef.h>

typedef struct {
    float coxa;
    float femur;
    float tibia;
} YPP_JointAngles;

typedef struct {
    float x;
    float y;
    float z;
} YPP_Vec3;

typedef struct {
    float L_coxa;
    float L_femur;
    float L_tibia;
    float coxa_min, coxa_max;
    float femur_min, femur_max;
    float tibia_min, tibia_max;
} YPP_LegParams;

void YPP_InitParams(YPP_LegParams* params);
void YPP_Forward(const YPP_LegParams* p, const YPP_JointAngles* ang, 
                 YPP_Vec3* foot_pos, YPP_Vec3* foot_normal);
bool YPP_Inverse_Pos(const YPP_LegParams* p, const YPP_Vec3* target_pos,
                     bool knee_bend_down, YPP_JointAngles* result);
bool YPP_Inverse_Pose(const YPP_LegParams* p, const YPP_Vec3* target_pos,
                      const YPP_Vec3* target_normal, YPP_JointAngles* result,
                      float* out_error_deg);
bool YPP_CheckLimits(const YPP_LegParams* p, const YPP_JointAngles* ang);
uint16_t YPP_AngleToServoValue(float angle_rad, float offset_rad);

#endif

