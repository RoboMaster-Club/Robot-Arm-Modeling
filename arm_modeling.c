#include <stdio.h>
#include <math.h>

#include "user_math.h"
#include "arm_modeling.h"

Mat* dh_transform(float a, float alpha, float d, float theta) {
    Mat* out_mat = new_mat(4, 4);
    
} 


def dh_transform(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])