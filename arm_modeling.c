#include <stdio.h>
#include <math.h>

#include "user_math.h"
#include "arm_modeling.h"

// [a, alpha, d, theta]
Arm_State_t g_arm_state = { .joints[0].dh_params = {0, PI/2, 0.1, 0.0f},
                            .joints[1].dh_params = {0.3, 0, 0, PI/2},
                            .joints[2].dh_params = {0.3, 0, 0, 0},
                            .joints[3].dh_params = {0, PI/2, 0.1, 0.0f},
                            .joints[4].dh_params = {0, -1 * PI/2, 0.1, 0.0f},
                            .joints[5].dh_params = {0, 0, 0.1, 0.0f}
};

//Denavit-Hartenberg matrix calculations
Mat* joint_transform(joint_t joint) {
    return dh_transform(joint.dh_params);
}

void forward_kinematics() {
    dh_update_arm();
    Mat* transform = mat_identity(4);
    for (int i = 0; i < 6; i++) {
        transform = mat_mult(transform, joint_transform(g_arm_state.joints[i]));
        g_arm_state.joints[i].x_pos = MAT_IDX(transform, 0,3);       // index [0, 3]
        g_arm_state.joints[i].y_pos = MAT_IDX(transform, 1,3);   // index [1, 3]
        g_arm_state.joints[i].z_pos = MAT_IDX(transform, 2,3);   // index [2, 3]
    }
} // can make it return final pos of arm and roll, pitch, yaw if needed

void update_angles()

void calculate_jacobian() {
    
}