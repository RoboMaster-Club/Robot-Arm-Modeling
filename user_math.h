#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

// DEFINE CONSTANTS
#define PI 3.14159265358979323846

// MACROS
#define MAT_IDX(m, i, j) ((m)->data[(i) * (m)->cols + (j)])


// NxN Matrix struct

typedef struct Mat {
    int rows;
    int cols;
    float* data;
} Mat;

Mat* new_mat(int rows, int cols);
Mat* new_eye(int size);
Mat* new_mat_buffer(int rows, int cols, float* buffer);
void free_mat(Mat* m);
Mat* mat_mult(Mat* m1, Mat* m2);
void mat_mult_buffer(Mat* m1, Mat* m2, Mat* product);
Mat* mat_scalar_mult(Mat* m, float scalar);
void mat_scalar_mult_buffer(Mat* m, float scalar, Mat* product);
Mat* mat_add(Mat* m1, Mat* m2);
void mat_add_buffer(Mat* m1, Mat* m2, Mat* sum);
Mat* mat_sub(Mat* m1, Mat* m2);
void mat_sub_buffer(Mat* m1, Mat* m2, Mat* diff);
float mat_determinant_recurs(Mat* m, int size, int r_start, int r_end, int c_start, int c_end, int i, int j);
float mat_determinant(Mat* m);
Mat* mat_adjoint(Mat* m);
Mat* mat_adjoint_buffer(Mat* m, Mat* buffer);
float mat_cofactor(Mat *m, int i, int j);
Mat* mat_cofactor_matrix(Mat* m);
Mat* mat_cofactor_matrix_buffer(Mat* m, Mat* buffer);
Mat* mat_inverse(Mat* m);
Mat* mat_inverse_buffer(Mat* m, Mat* buffer);
Mat* mat_transpose(Mat *m);
Mat* mat_transpose_buffer(Mat *m);


// ADD TRANSPOSE, PSEUDO INVERSE, COFACTOR from determinant and adjoint as transpose

