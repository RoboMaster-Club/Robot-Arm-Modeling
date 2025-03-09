#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "user_math.h"

int test_count = 0;

void test_determinant_1x1() {
    Mat* m = new_mat(1, 1);
    m->data[0] = 5.0f;
    assert(mat_determinant(m) == 5.0f);
    free_mat(m);
    test_count++;
}

void test_determinant_2x2() {
    Mat* m = new_mat(2, 2);
    m->data[0] = 1; m->data[1] = 2;
    m->data[2] = 3; m->data[3] = 4;
    assert(mat_determinant(m) == (1 * 4 - 2 * 3)); // -2
    free_mat(m);
    test_count++;
}

void test_determinant_3x3() {
    Mat* m = new_mat(3, 3);
    m->data[0] = 1; m->data[1] = 2; m->data[2] = 3;
    m->data[3] = 4; m->data[4] = 5; m->data[5] = 6;
    m->data[6] = 7; m->data[7] = 8; m->data[8] = 9;
    assert(mat_determinant(m) == 0); // Singular matrix
    free_mat(m);
    test_count++;
}

void test_determinant_4x4() {
    Mat* m = new_mat(4, 4);
    float values[16] = {
         6,  1,  1,  3,
         4, -2,  5,  1,
         2,  8,  7,  6,
         3,  1,  9,  7
    };
    for (int i = 0; i < 16; i++) m->data[i] = values[i];
    
    assert(mat_determinant(m) == -418); // Known determinant
    free_mat(m);
    test_count++;
}

void test_mat_determinant_recurs() {
    Mat* m = new_mat(4, 4);
    float values[16] = {
         1,  2,  3,  4,
         5,  6,  7,  8,
         9, -1, -2, -3,
        -4, -5, -6, -7
    };
    for (int i = 0; i < 16; i++) m->data[i] = values[i];

    // Extract the submatrix:
    // | 6  7  8 |
    // |-1 -2 -3 |
    // |-5 -6 -7 |
    float det = mat_determinant_recurs(m, 3, 1, 3, 1, 3, 0, 0);
    
    assert(det == 0); // This particular submatrix is singular
    free_mat(m);
    test_count++;
}

int main() {
    test_determinant_1x1();
    test_determinant_2x2();
    test_determinant_3x3();
    test_determinant_4x4();
    test_mat_determinant_recurs();

    printf("All determinant tests passed!\n");

    printf("Total tests passed: %d\n", test_count);
    return 0;
}
