#include <stdio.h>
#include <math.h>

#include "user_math.h"
#include "arm_modeling.h"



// [a, alpha, d]
Arm_State_t g_arm_state = { .joints[0].dh_params = {0, PI / 2, 0.1},
                            .joints[1].dh_params = {0.3, 0, 0, PI / 2},
                            .joints[2].dh_params = {0.3, 0, 0, 0},
                            .joints[3].dh_params = {0, PI /2, 0.1},
                            .joints[4].dh_params = {0, -1 * PI /2, 0.1},
                            .joints[5].dh_params = {0, 0, 0.1}
}; // could be #defined somewhere these are all constants

//Denavit-Hartenberg matrix calculations
Mat* joint_transform(joint_t joint) {
    return dh_transform(joint.dh_params);
}

void forward_kinematics() {
    dh_update_arm();
    Mat* transform = mat_identity(4);
    for (int i = 0; i < 6; i++) {
        transform = mat_mult(transform, dh_transform(g_arm_state->joints[i]));
        g_arm_state.joints[i].x_pos = transform->data[3];       // index [0, 3]
        g_arm_state.joints[i].y_pos = transform->data[3 + 4];   // index [1, 3]
        g_arm_state.joints[i].z_pos = transform->data[3 + 8];   // index [2, 3]
    }
} // can make it return final pos of arm and roll, pitch, yaw if needed