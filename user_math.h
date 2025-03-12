#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include <float.h>

// DEFINE CONSTANTS
#define PI 3.14159265358979323846
#define bool int
#define true 1
#define false 0

// MACROS
#define MAT_IDX(m, i, j) ((m)->data[(i) * (m)->cols + (j)])
#define VEC_IDX(m, i) ((m)->data[(i)])

/*
-------------------------------------------------------------
SECTION:	MATRICES
-------------------------------------------------------------
*/

// NxN Matrix struct
typedef struct Mat {
    int rows;
    int cols;
    float* data;
} Mat;

// matrix creation functions
Mat* new_mat(int rows, int cols);
Mat* new_eye(int size);
Mat* new_mat_buffer(int rows, int cols, float* buffer);
void free_mat(Mat* m);

// matrix helpers
char* mat_to_string(Mat* m);
void print_mat(Mat* m);
bool mat_equal(Mat* m1, Mat* m2, float tol);

// general matrix operations
Mat* mat_mult(Mat* m1, Mat* m2);
Mat* mat_mult_buffer(Mat* m1, Mat* m2, Mat* product);
Mat* mat_scalar_mult(Mat* m, float scalar);
Mat* mat_scalar_mult_buffer(Mat* m, float scalar, Mat* product);
Mat* mat_add(Mat* m1, Mat* m2);
Mat* mat_add_buffer(Mat* m1, Mat* m2, Mat* sum);
Mat* mat_sub(Mat* m1, Mat* m2);
Mat* mat_sub_buffer(Mat* m1, Mat* m2, Mat* diff);

// advanced matrix operations
float mat_determinant(Mat* m);
Mat* mat_adjoint(Mat* m);
Mat* mat_adjoint_buffer(Mat* m, Mat* buffer);
float mat_cofactor(Mat *m, int i, int j);
Mat* mat_cofactor_matrix(Mat* m);
Mat* mat_cofactor_matrix_buffer(Mat* m, Mat* buffer);
Mat* mat_inverse(Mat* m);
Mat* mat_inverse_buffer(Mat* m, Mat* buffer);
Mat* mat_transpose(Mat *m);
Mat* mat_transpose_buffer(Mat *m, Mat* buffer);
Mat* mat_transpose_overwrite(Mat *m);
Mat* mat_pseudo_inverse(Mat *m);
Mat* mat_damped_pseudo_inverse(Mat* m, float rho);

// TODO: ADD PSEUDO INVERSE

// TODO: If I feel like it: LU DECOMPOSITION, QR DECOMPOSITION, SVD DECOMPOSITION, EIGEN DECOMPOSITION, SOLVE LINEAR SYSTEM, SOLVE EIGENVALUE PROBLEM, SOLVE EIGENVECTOR PROBLEM, SOLVE SVD PROBLEM

/*
-------------------------------------------------------------
SECTION:	VECTORS
-------------------------------------------------------------
*/

// To maintain compatibility we use 1D matrix for vectors. You can simply use the matrix functions for vectors.
#define Vec Mat

// vector creation functions
Vec* new_vec(int size);
Vec* new_vec_buffer(int size, float* buffer);

// vector helpers
bool assert_vec(Vec* v);

// general vector operations
float vec_dot(Vec* v1, Vec* v2);
Vec* vec_cross(Vec* v1, Vec* v2);
Vec* vec_cross_buffer(Vec* v1, Vec* v2, Vec* buffer);
float vec_magnitude(Vec* v);
Vec* vec_normalize(Vec* v);
Vec* vec_normalize_overwrite(Vec* v);


/*
-------------------------------------------------------------
SECTION:	DH TRANSFORMATIONS
-------------------------------------------------------------
*/

typedef struct DH_Params {
    float a;
    float alpha;
    float d;
    float theta;
} DH_Params;

// Denavit-Hartenberg matrix calculations
DH_Params* new_dh_params(float a, float alpha, float d, float theta);
void free_dh_params(DH_Params* dh);
Mat* dh_transform(DH_Params dh);
Mat* dh_transform_buffer(DH_Params dh, Mat* buffer);
