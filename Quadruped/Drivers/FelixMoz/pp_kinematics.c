#include "pp_kinematics.h"

#define PI 3.14159265359f
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)
#define SERVO_RES 0.24f  /* HT-85H resolution: 0.24 degrees per unit */

void PP_InitParams(PP_LegParams* p) {
    /* Default leg dimensions - modify to match your mechanical design */
    p->L1 = 0.12f;  /* 120mm femur length */
    p->L2 = 0.12f;  /* 120mm tibia length */
    
    /* Joint limits in radians */
    p->theta1_min = -90.0f * DEG2RAD;  /* Femur backward limit */
    p->theta1_max = 90.0f * DEG2RAD;   /* Femur forward limit */
    p->theta2_min = 0.0f * DEG2RAD;    /* Tibia straight (fully extended) */
    p->theta2_max = 150.0f * DEG2RAD;  /* Tibia forward bend limit */
}

/* Forward kinematics: calculate foot position and pad normal from joint angles.
   Standard 2-link planar manipulator equations:
   x = L1*cos(femur) + L2*cos(femur+tibia)
   z = L1*sin(femur) + L2*sin(femur+tibia)
   normal = [cos(femur+tibia), sin(femur+tibia)] */
void PP_Forward(const PP_LegParams* p, const PP_JointAngles* ang,
                PP_Point2D* foot_pos, PP_Normal2D* pad_normal) {
    float s1 = sinf(ang->femur);
    float c1 = cosf(ang->femur);
    float sum_ang = ang->femur + ang->tibia;
    float s12 = sinf(sum_ang);
    float c12 = cosf(sum_ang);
    
    /* Calculate foot position */
    foot_pos->x = p->L1 * c1 + p->L2 * c12;
    foot_pos->z = p->L1 * s1 + p->L2 * s12;
    
    /* Pad normal points along tibia axis (outward from leg) */
    if (pad_normal) {
        pad_normal->x = c12;
        pad_normal->z = s12;
    }
}

/* Standard 2-link planar inverse kinematics (position only).
   Uses cosine law to solve for elbow angle, then geometric decomposition for shoulder.
   elbow_up = false: knee bends forward (typical for climbing)
   elbow_up = true: knee bends backward */
bool PP_Inverse_Pos(const PP_LegParams* p, const PP_Point2D* target,
                    bool elbow_up, PP_JointAngles* sol) {
    float x = target->x;
    float z = target->z;
    float d_sq = x*x + z*z;
    float d = sqrtf(d_sq);
    
    /* Check reachability against mechanical limits */
    float max_reach = p->L1 + p->L2;
    float min_reach = fabsf(p->L1 - p->L2);
    if (d > max_reach || d < min_reach) return false;
    
    /* Solve tibia angle using cosine law */
    float cos_t2 = (d_sq - p->L1*p->L1 - p->L2*p->L2) / (2.0f * p->L1 * p->L2);
    cos_t2 = fmaxf(-1.0f, fminf(1.0f, cos_t2));  /* Clamp for numerical safety */
    
    if (elbow_up) {
        sol->tibia = -acosf(cos_t2);  /* Elbow bends backward */
    } else {
        sol->tibia = acosf(cos_t2);   /* Elbow bends forward */
    }
    
    /* Solve femur angle using geometric decomposition:
       alpha = angle to target, beta = offset from tibia geometry */
    float alpha = atan2f(z, x);
    float beta = atan2f(p->L2 * sinf(sol->tibia), 
                       p->L1 + p->L2 * cosf(sol->tibia));
    sol->femur = alpha - beta;
    
    /* Validate against joint limits */
    return (sol->femur >= p->theta1_min && sol->femur <= p->theta1_max &&
            sol->tibia >= p->theta2_min && sol->tibia <= p->theta2_max);
}

/* Pose-constrained inverse kinematics for suction cup climbing.
   Since 2-DOF cannot independently control (position, angle), this tries both
   elbow configurations and returns the one with <5 degree normal error.
   target_normal must be unit vector or near-unit vector.
   Returns actual angular error in degrees via error_deg pointer. */
bool PP_Inverse_Pose(const PP_LegParams* p, const PP_Point2D* target,
                     const PP_Normal2D* target_n, 
                     PP_JointAngles* sol, float* error_deg) {
    /* Normalize target normal vector */
    float nx = target_n->x;
    float nz = target_n->z;
    float norm = sqrtf(nx*nx + nz*nz);
    if (norm < 0.001f) return false;  /* Invalid zero vector */
    nx /= norm; 
    nz /= norm;
    
    /* Try elbow-down (knee forward) first - usually more stable for climbing */
    if (PP_Inverse_Pos(p, target, false, sol)) {
        PP_Normal2D actual;
        PP_Point2D temp_pos;
        PP_Forward(p, sol, &temp_pos, &actual);
        
        /* Calculate angular error via dot product */
        float dot = nx*actual.x + nz*actual.z;
        dot = fmaxf(-1.0f, fminf(1.0f, dot));
        float err = acosf(dot) * RAD2DEG;
        
        if (err < 5.0f) {  /* Within 5 degree tolerance */
            if (error_deg) *error_deg = err;
            return true;
        }
    }
    
    /* Fallback: try elbow-up (knee backward) configuration */
    if (PP_Inverse_Pos(p, target, true, sol)) {
        PP_Normal2D actual;
        PP_Point2D temp_pos;
        PP_Forward(p, sol, &temp_pos, &actual);
        
        float dot = nx*actual.x + nz*actual.z;
        dot = fmaxf(-1.0f, fminf(1.0f, dot));
        float err = acosf(dot) * RAD2DEG;
        
        if (err < 5.0f) {
            if (error_deg) *error_deg = err;
            return true;
        }
    }
    
    return false;  /* Neither configuration satisfies constraint */
}

/* Convert radians to HT-85H servo protocol value (0-1000).
   offset_rad compensates for mechanical mounting offsets.
   Result clamped to valid range [0, 240] degrees. */
uint16_t PP_AngleToServo(float rad, float offset_rad) {
    float total_deg = (rad + offset_rad) * RAD2DEG;
    if (total_deg < 0.0f) total_deg = 0.0f;
    if (total_deg > 240.0f) total_deg = 240.0f;
    return (uint16_t)(total_deg / SERVO_RES + 0.5f);
}

