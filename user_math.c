#include "user_math.h"

// #define assert(cond) if (!(cond)) { printf("Assertion failed: %s\n", #cond); exit(1); }

Mat* new_mat(int rows, int cols) {
    Mat* mat = (Mat*)malloc(sizeof(Mat));
    mat->rows = rows;
    mat->cols = cols;
    mat->data = (float*)malloc(rows * cols * sizeof(float));
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            MAT_IDX(mat, i, j) = 0;
        }
    }
    return mat;
}

Mat* new_eye(int size) {
    Mat* mat = new_mat(size, size);
    for (int i = 0; i < size; i++) {
        MAT_IDX(mat, i, i) = 1;
    }
    return mat;
}

Mat* new_mat_buffer(int rows, int cols, float* buffer) {
    Mat* mat = (Mat*)malloc(sizeof(Mat));
    mat->rows = rows;
    mat->cols = cols;
    mat->data = (float*)malloc(rows * cols * sizeof(float));
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            MAT_IDX(mat, i, j) = buffer[i * cols + j];
        }
    }
    return mat;
}

void free_mat(Mat* m) {
    free(m->data);
    free(m);
}

Mat* mat_mult(Mat* m1, Mat* m2) {
    assert(m1->cols == m2->rows);
    Mat* product = (Mat*)malloc(m1->rows * m2->cols);
    product->rows = m1->rows;
    product->cols = m2->cols;
    for (int i = 0; i < m1->rows; i++) {
        for (int j = 0; j < m2->cols; j++) {
            MAT_IDX(product, i, j) = 0;
            for (int k = 0; k < m1->cols; k++) {
                MAT_IDX(product, i, j) += MAT_IDX(m1, i, k) * MAT_IDX(m2, k, j);
            }
        }
    }
}

void mat_mult_buffer(Mat* m1, Mat* m2, Mat* product) {
    assert(m1->cols == m2->rows &&
              product->rows == m1->rows &&
              product->cols == m2->cols);
    for (int i = 0; i < m1->rows; i++) {
        for (int j = 0; j < m2->cols; j++) {
            MAT_IDX(product, i, j) = 0;
            for (int k = 0; k < m1->cols; k++) {
                MAT_IDX(product, i, j) += MAT_IDX(m1, i, k) * MAT_IDX(m2, k, j);
            }
        }
    }
}

Mat* mat_scalar_mult(Mat* m, float scalar) {
    Mat* product = (Mat*)malloc(m->rows * m->cols);
    product->rows = m->rows;
    product->cols = m->cols;
    for (int i = 0; i < m->rows; i++) {
        for (int j = 0; j < m->cols; j++) {
            MAT_IDX(product, i, j) = MAT_IDX(m, i, j) * scalar;
        }
    }
    return product;
}

void mat_scalar_mult_buffer(Mat* m, float scalar, Mat* product) {
    assert (product->rows == m->rows && product->cols == m->cols);
    for (int i = 0; i < m->rows; i++) {
        for (int j = 0; j < m->cols; j++) {
            MAT_IDX(product, i, j) = MAT_IDX(m, i, j) * scalar;
        }
    }
}

Mat* mat_add(Mat* m1, Mat* m2) {
    assert(m1->rows == m2->rows && m1->cols == m2->cols);
    Mat* sum = (Mat*)malloc(m1->rows * m1->cols);
    sum->rows = m1->rows;
    sum->cols = m1->cols;
    for (int i = 0; i < m1->rows; i++) {
        for (int j = 0; j < m1->cols; j++) {
            MAT_IDX(sum, i, j) = MAT_IDX(m1, i, j) + MAT_IDX(m2, i, j);
        }
    }
    return sum;
}

void mat_add_buffer(Mat* m1, Mat* m2, Mat* sum) {
    assert(m1->rows == m2->rows && m1->cols == m2->cols && sum->rows == m1->rows && sum->cols == m1->cols);
    for (int i = 0; i < m1->rows; i++) {
        for (int j = 0; j < m1->cols; j++) {
            MAT_IDX(sum, i, j) = MAT_IDX(m1, i, j) + MAT_IDX(m2, i, j);
        }
    }
}

Mat* mat_sub(Mat* m1, Mat* m2) {
    assert(m1->rows == m2->rows && m1->cols == m2->cols);
    Mat* diff = (Mat*)malloc(m1->rows * m1->cols);
    diff->rows = m1->rows;
    diff->cols = m1->cols;
    for (int i = 0; i < m1->rows; i++) {
        for (int j = 0; j < m1->cols; j++) {
            MAT_IDX(diff, i, j) = MAT_IDX(m1, i, j) - MAT_IDX(m2, i, j);
        }
    }
    return diff;
}


void mat_sub_buffer(Mat* m1, Mat* m2, Mat* diff) {
    assert(m1->rows == m2->rows && m1->cols == m2->cols && diff->rows == m1->rows && diff->cols == m1->cols);
    for (int i = 0; i < m1->rows; i++) {
        for (int j = 0; j < m1->cols; j++) {
            MAT_IDX(diff, i, j) = MAT_IDX(m1, i, j) - MAT_IDX(m2, i, j);
        }
    }
}

/**
 * Calculate the determinant of the matrix recursively. DO NOT CALL THIS FUNCTION DIRECTLY.
 * 
 * @param m The matrix to calculate the determinant of.
 * @param size The size of the matrix determinant. Must be at least 2.
 * @param r_start The starting row index of the submatrix.
 * @param r_end The ending row index of the submatrix.
 * @param c_start The starting column index of the submatrix.
 * @param c_end The ending column index of the submatrix.
 * @param i The row index of the element in the submatrix to calculate the determinant of.
 * @param j The column index of the element int the submatrix to calculate the determinant of.
 * 
 * @return The determinant of the submatrix.
 * 
 * Example:
 * 
 * if m = |  1  2  3  4 |
 *        |  5  6  7  8 |
 *        |  9 -1 -2 -3 |
 *        | -4 -5 -6 -7 |
 * 
 * And you want to take the determinant of the 3x3 submatrix from i=1, j=1 to i=3, j=3 for element 6 (so i=0, j=0 relative to the submatrix):
 * you would call: mat_determinant_recurs(m, 3, 1, 3, 1, 3, 0, 0)
 * 
 * Here, the submatrix would be:
 * |  6  7  8 |
 * | -1 -2 -3 |
 * | -5 -6 -7 |
 * 
 * So the size is 2, r_start = 1, r_end = 3, c_start = 1, c_end = 3, i = 0, j = 0
 */
float mat_determinant_recurs(Mat* m, int size, int r_start, int r_end, int c_start, int c_end, int i, int j) {
    assert(m->rows == m->cols && size >= 2);
    assert(r_start >= 0 && r_end < m->rows && c_start >= 0 && c_end < m->cols);
    assert(size == r_end - r_start && size == c_end - c_start);
    assert(i >= 0 && i <= size && j >= 0 && j <= size);
    // TODO Add assertions
    float det = 0;
    switch(size) {
        case 2:
            return MAT_IDX(m, r_start + (i?0:1), c_start + (j?0:1)) * MAT_IDX(m, r_end - (size-i?0:1), c_end - (size-j?0:1))
            - MAT_IDX(m, r_start + (i?0:1), c_end - (size-j?0:1)) * MAT_IDX(m, r_end - (size-i?0:1), c_start + (j?0:1));
        default:
            for (int k = 0; k < size; k++) {
                det += (1 - 2 * ((i + j) % 2)) * MAT_IDX(m, r_start + i, c_start + j) * mat_determinant_recurs(m, size - 1, r_start, r_end, c_start, c_end, i, j);
            }
            return det;
    }
}

/**
 * Calculate the determinant of the matrix.
 * 
 * @param m The matrix to calculate the determinant of.
 * 
 * @return The determinant of the matrix.
 * 
 * Example:
 * 
 * if m = |1 2 3|
 *        |4 5 6|
 *        |7 8 9|
 * 
 * You would call: mat_determinant(m)
 */
float mat_determinant(Mat* m) {
    assert (m->rows == m->cols);
    float det = 0;
    switch (m->rows) {
        case 1:
            return m->data[0];
        case 2:
            return m->data[0] * m->data[3] - m->data[1] * m->data[2];
        case 3:
            for (int i = 0; i < 3; i++) {
                det += m->data[i] * m->data[4 + (i + 1) % 3] * m->data[8 + (i + 2) % 3];
                det -= m->data[2 + i] * m->data[4 + (i + 1) % 3] * m->data[6 + (i + 2) % 3];
            }
            return det;
        default:
            for (int i = 0; i < m->rows; i++) {
                det += (1 - 2 * (i % 2)) * m->data[i] * mat_determinant_recurs(m, m->rows - 1, 0, m->rows - 1, 0, m->rows - 1, i, 0);
            }
            return det;
    }
}

Mat* mat_inverse(Mat* m) {
    assert(m->rows == m->cols);
    Mat* inverse = (Mat*)malloc(m->rows * m->cols);
    inverse->rows = m->rows;
    inverse->cols = m->cols;
    float det = mat_determinant(m);
    assert(det != 0);
    Mat* adj = mat_adjoint(m);
    mat_scalar_mult_buffer(adj, 1 / det, inverse);
    free_mat(adj);
    return inverse;
}

Mat* mat_adjoint(Mat* m) {
    assert(m->rows == m->cols);
    Mat* adj = (Mat*)malloc(m->rows * m->cols);
    adj->rows = m->rows;
    adj->cols = m->cols;
    for (int i = 0; i < m->rows; i++) {
        for (int j = 0; j < m->cols; j++) {
            adj->data[i * adj->cols + j] = mat_cofactor(m, i, j);
        }
    }
    return adj;
}

float mat_cofactor(Mat *m, int i, int j) {
    assert(m->rows == m->cols);
    assert(i < m->rows && j < m->cols);
    float cofac = (1 - 2 * ((i * j) % 2)) * mat_determinant_recurs(m, m->rows, 0, m->rows, 0, m->cols, i, j);
}