#include "user_math.h"
#include <assert.h>

// #define assert(cond) if (!(cond)) { printf("Assertion failed: %s\n", #cond); exit(1); }
#define MAT_IDX(m, i, j) ((m)->data[(i) * (m)->cols + (j)])

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
    assert(m1->rows == m2->rows && m1->cols == m2->cols && sum->rows == m1->rows && sum->cols == m1->cols);
    for (int i = 0; i < m1->rows; i++) {
        for (int j = 0; j < m1->cols; j++) {
            MAT_IDX(diff, i, j) = MAT_IDX(m1, i, j) - MAT_IDX(m2, i, j);
        }
    }
}

float mat_determinant_recurs(Mat* m, int size, int r_start, int r_end, int c_start, int c_end, int i, int j) {
    assert(m->rows == m->cols && size >= 2);
    // TODO Add assertions
    float det = 0;
    switch(size) {
        case 2:
            return m->data[r_start * m->cols + c_start] * m->data[r_end * m->cols + c_end] 
            - m->data[r_start * m->cols + c_end] * m->data[r_end * m->cols + c_start];
        case 3:
            return
    }
}

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
            for (int i = 0; i < m-> rows; i++) {
                for (int j = 0; j < m->cols; j++) {
                    
                }
            }
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
    assert(m->rows == m->cols && i < m->rows && j < m->cols)
    float cofac = (1 - 2 * ((i * j) % 2)) * mat_determinant_recurs(m, i, j)
}