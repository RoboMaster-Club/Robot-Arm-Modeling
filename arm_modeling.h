#include <stdio.h>
#include <math.h>

#include "user_math.h"

typedef struct {
    DH_Params dh_params;
    float theta;
    float velocity;
    float x_pos;
    float y_pos;
    float z_pos;
} joint_t;

typedef struct {
    joint_t joints[6];
} Arm_State_t;

Arm_State_t g_arm_state;