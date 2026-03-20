#include "ypp_kinematics.h"

#define PI 3.14159265359f
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)
#define SERVO_RESOLUTION 0.24f
#define SERVO_CENTER 500

void YPP_InitParams(YPP_LegParams* params) {
    params->L_coxa = 0.05f;
    params->L_femur = 0.12f;
    params->L_tibia = 0.12f;
    params->coxa_min = -60.0f * DEG2RAD;
    params->coxa_max = 60.0f * DEG2RAD;
    params->femur_min = -90.0f * DEG2RAD;
    params->femur_max = 45.0f * DEG2RAD;
    params->tibia_min = 0.0f * DEG2RAD;
    params->tibia_max = 120.0f * DEG2RAD;
}

void YPP_Forward(const YPP_LegParams* p, const YPP_JointAngles* ang,
                 YPP_Vec3* foot_pos, YPP_Vec3* foot_normal) {
    float s1 = sinf(ang->coxa);
    float c1 = cosf(ang->coxa);
    float s2 = sinf(ang->femur);
    float c2 = cosf(ang->femur);
    float alpha = ang->femur + ang->tibia;
    float sa = sinf(alpha);
    float ca = cosf(alpha);
    
    float x1 = p->L_coxa * c1;
    float y1 = p->L_coxa * s1;
    float dx = p->L_femur * c2 + p->L_tibia * ca;
    float dz = p->L_femur * s2 + p->L_tibia * sa;
    
    foot_pos->x = x1 + dx * c1;
    foot_pos->y = y1 + dx * s1;
    foot_pos->z = -dz;
    
    if (foot_normal != NULL) {
        foot_normal->x = ca * c1;
        foot_normal->y = ca * s1;
        foot_normal->z = -sa;
    }
}

bool YPP_Inverse_Pos(const YPP_LegParams* p, const YPP_Vec3* target_pos,
                     bool knee_bend_down, YPP_JointAngles* result) {
    float x = target_pos->x;
    float y = target_pos->y;
    float z = target_pos->z;
    result->coxa = atan2f(y, x);
    
    if (result->coxa < p->coxa_min || result->coxa > p->coxa_max) return false;
    
    float x1 = p->L_coxa * cosf(result->coxa);
    float y1 = p->L_coxa * sinf(result->coxa);
    float dx = x - x1;
    float dy = y - y1;
    float D = sqrtf(dx*dx + dy*dy);
    float H = -z;
    
    float max_reach = p->L_femur + p->L_tibia;
    float min_reach = fabsf(p->L_femur - p->L_tibia);
    float dist = sqrtf(D*D + H*H);
    if (dist > max_reach || dist < min_reach) return false;
    
    float cos_theta3 = (dist*dist - p->L_femur*p->L_femur - p->L_tibia*p->L_tibia) 
                       / (2.0f * p->L_femur * p->L_tibia);
    if (cos_theta3 > 1.0f) cos_theta3 = 1.0f;
    if (cos_theta3 < -1.0f) cos_theta3 = -1.0f;
    
    if (knee_bend_down) result->tibia = -acosf(cos_theta3);
    else result->tibia = acosf(cos_theta3);
    
    float alpha = atan2f(H, D);
    float r = p->L_tibia * sinf(-result->tibia);
    float d = p->L_femur + p->L_tibia * cosf(-result->tibia);
    float beta = atan2f(r, d);
    result->femur = alpha - beta;
    
    return YPP_CheckLimits(p, result);
}

bool YPP_Inverse_Pose(const YPP_LegParams* p, const YPP_Vec3* target_pos,
                      const YPP_Vec3* target_normal, YPP_JointAngles* result,
                      float* out_error_deg) {
    float nx = target_normal->x;
    float ny = target_normal->y;
    float nz = target_normal->z;
    float norm = sqrtf(nx*nx + ny*ny + nz*nz);
    if (norm < 0.001f) return false;
    nx /= norm; ny /= norm; nz /= norm;
    
    float alpha = atan2f(-nz, sqrtf(nx*nx + ny*ny));
    result->coxa = atan2f(ny, nx);
    
    float x1 = p->L_coxa * cosf(result->coxa);
    float y1 = p->L_coxa * sinf(result->coxa);
    float dx_world = target_pos->x - x1;
    float dy_world = target_pos->y - y1;
    float D_target = dx_world * cosf(result->coxa) + dy_world * sinf(result->coxa);
    float H_target = -target_pos->z;
    
    float D_req = D_target - p->L_tibia * cosf(alpha);
    float H_req = H_target - p->L_tibia * sinf(alpha);
    float reach_error = sqrtf(D_req*D_req + H_req*H_req) - p->L_femur;
    if (fabsf(reach_error) > 0.001f) return false;
    
    result->femur = atan2f(H_req, D_req);
    result->tibia = alpha - result->femur;
    
    if (!YPP_CheckLimits(p, result)) return false;
    
    YPP_Vec3 actual_pos, actual_n;
    YPP_Forward(p, result, &actual_pos, &actual_n);
    float dot = nx*actual_n.x + ny*actual_n.y + nz*actual_n.z;
    if (dot > 1.0f) dot = 1.0f;
    if (dot < -1.0f) dot = -1.0f;
    float error_rad = acosf(dot);
    if (out_error_deg != NULL) *out_error_deg = error_rad * RAD2DEG;
    
    return (error_rad < 5.0f * DEG2RAD);
}

bool YPP_CheckLimits(const YPP_LegParams* p, const YPP_JointAngles* ang) {
    return (ang->coxa >= p->coxa_min && ang->coxa <= p->coxa_max &&
            ang->femur >= p->femur_min && ang->femur <= p->femur_max &&
            ang->tibia >= p->tibia_min && ang->tibia <= p->tibia_max);
}

uint16_t YPP_AngleToServoValue(float angle_rad, float offset_rad) {
    float total_deg = (angle_rad + offset_rad) * RAD2DEG;
    if (total_deg < 0.0f) total_deg = 0.0f;
    if (total_deg > 240.0f) total_deg = 240.0f;
    return (uint16_t)(total_deg / SERVO_RESOLUTION + 0.5f);
}
