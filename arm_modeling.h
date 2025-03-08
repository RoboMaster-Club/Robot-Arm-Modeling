#include <stdio.h>
#include <math.h>

#define PI (3.141592f)

float g_theta0 = 0;
float g_theta1 = 0;
float g_theta2 = 0;
float g_theta3 = 0;
float g_theta4 = 0;
float g_theta5 = 0;

typedef struct joint {
    float a;
    float alpha;
    float d;
    float theta;
} joint_t;

typedef struct arm {
    joint_t joints[6];  
} arm_t;

// [a, alpha, d, theta]
arm_t g_arm = {{{0, M_PI / 2, 0.1, 0},
                 {0.3, 0, 0, M_PI / 2},
                 {0.3, 0, 0, 0},
                 {0, M_PI /2, 0.1, 0},
                 {0, -1 * M_PI /2, 0.1, 0},
                 {0, 0, 0.1, 0}}
};
