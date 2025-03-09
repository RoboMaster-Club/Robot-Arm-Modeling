#include <stdio.h>
#include <math.h>

#include "user_math.h"
#include "arm_modeling.h"

dh_arm_t g_dh_arm = {{{0, M_PI / 2, 0.1, 0},
                 {0.3, 0, 0, M_PI / 2},
                 {0.3, 0, 0, 0},
                 {0, M_PI /2, 0.1, 0},
                 {0, -1 * M_PI /2, 0.1, 0},
                 {0, 0, 0.1, 0}}
};

Arm_State_t g_arm_state = { 0 };

void dh_update_arm() {
     g_dh_arm.joints[0].theta = g_arm_state.joints[0].theta;
     g_dh_arm.joints[1].theta = g_arm_state.joints[1].theta;
     g_dh_arm.joints[2].theta = g_arm_state.joints[2].theta;
     g_dh_arm.joints[3].theta = g_arm_state.joints[3].theta;
     g_dh_arm.joints[4].theta = g_arm_state.joints[4].theta;
     g_dh_arm.joints[5].theta = g_arm_state.joints[5].theta;
}

//Denavit-Hartenberg matrix calculations
Mat* dh_transform(float a, float alpha, float d, float theta) {
    Mat* out_mat = new_mat(4, 4);
    // first row
    out_mat->data[0] = cosf(theta);
    out_mat->data[1] = -1 * sinf(theta) * cosf(alpha);
    out_mat->data[2] = sinf(theta) * sinf(alpha);
    out_mat->data[3] =  a * cosf(theta);
    // second row
    out_mat->data[0 + 4] = sinf(theta);
    out_mat->data[1 + 4] = cosf(theta) * cosf(alpha);
    out_mat->data[2 + 4] = cosf(theta) * sinf(alpha);
    out_mat->data[3 + 4] =  a * sinf(theta);
    // third row
    out_mat->data[0 + 8] = 0;
    out_mat->data[1 + 8] = sinf(alpha);
    out_mat->data[2 + 8] = cosf(alpha);
    out_mat->data[3 + 8] =  d;
    //fourth row
    out_mat->data[0 + 12] = 0;
    out_mat->data[1 + 12] = 0;
    out_mat->data[2 + 12] = 0;
    out_mat->data[3 + 12] =  1;
    
    return out_mat;
}

// void forward_kinematics() {
//     dh_update_arm();
//     Mat
    



