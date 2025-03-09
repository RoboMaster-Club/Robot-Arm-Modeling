#include <stdio.h>
#include <math.h>

typedef struct {
    float a;
    float alpha;
    float d;
    float theta;
} dh_joint_t;

typedef struct {
    dh_joint_t joints[6];  
} dh_arm_t;

typedef struct {
    float theta;
    float velocity;
    float x_pos;
    float y_pos;
    float z_pos;
} joint_t;

typedef struct {
    joint_t joints[6];
} Arm_State_t;

dh_arm_t g_dh_arm;
Arm_State_t g_arm_state;