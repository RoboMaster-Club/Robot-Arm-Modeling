#include "user_math.h"

// #define assert(cond) if (!(cond)) { printf("Assertion failed: %s\n", #cond); exit(1); }

/*
-------------------------------------------------------------
SECTION:	Matrix Creation Functions
-------------------------------------------------------------
*/

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

/*
-------------------------------------------------------------
SECTION:	Matrix Helpers
-------------------------------------------------------------
*/

char* mat_to_string(Mat* m) {
    // esdtimate size
    int buffer_size = 1000;
    char* str = (char*)malloc(buffer_size * sizeof(char));
    if (!str) return NULL;
    
    char temp[50]; // Temporary buffer for formatting each number
    snprintf(str, buffer_size, "Matrix %dx%d\n", m->rows, m->cols);

    // check largest and smallest -> see if need sci notation decision
    float max_val = 0.0, min_val = __FLT_MAX__;
    for (int i = 0; i < m->rows * m->cols; i++) {
        if (fabsf(m->data[i]) > max_val) max_val = fabsf(m->data[i]);
        if (fabsf(m->data[i]) < min_val && m->data[i] != 0) min_val = fabsf(m->data[i]);
    }

    int use_sci = (max_val > 1e4 || min_val < 1e-3);  // if large diff, use sci notation

    for (int i = 0; i < m->rows; i++) {
        strcat(str, "|");
        for (int j = 0; j < m->cols; j++) {
            float val = MAT_IDX(m, i, j);
            if (use_sci)
                snprintf(temp, sizeof(temp), " % .2e", val);  // scientific notation
            else
                snprintf(temp, sizeof(temp), " %6.2f", val);  // default

            strcat(str, temp);
        }
        strcat(str, " |\n");
    }
    
    return str;
}


void print_mat(Mat* m) {
    char* str = mat_to_string(m);
    printf("\n%s\n", str);
    free(str);
}

bool mat_equal(Mat* m1, Mat* m2, float tol) {
    if (m1->rows != m2->rows || m1->cols != m2->cols) {
        return false;
    }
    for (int i = 0; i < m1->rows * m1->cols; i++) {
        if (fabsf(m1->data[i] - m2->data[i]) > tol) {
            return false;
        }
    }
    return true;
}

/*
-------------------------------------------------------------
SECTION:	General Matrix Operations
-------------------------------------------------------------
*/

Mat* mat_mult(Mat* m1, Mat* m2) {
    assert(m1->cols == m2->rows);
    Mat* product = new_mat(m1->rows, m2->cols);
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
    return product;
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
    Mat* product = new_mat(m->rows, m->cols);
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
    Mat* sum = new_mat(m1->rows, m1->cols);
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
    Mat* diff = new_mat(m1->rows, m1->cols);
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

/*
-------------------------------------------------------------
SECTION:	Advanced Matrix Operations
-------------------------------------------------------------
*/

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
    float det = 0.0f;
    switch (m->rows) {
        case 1:
            return m->data[0];
        case 2:
            return m->data[0] * m->data[3] - m->data[1] * m->data[2];
        case 3:
            for (int i = 0; i < 3; i++) {
                det += MAT_IDX(m, 0, i) * MAT_IDX(m, 1, (i + 1) % 3) * MAT_IDX(m, 2, (i + 2) % 3);
                det -= MAT_IDX(m, 0, i) * MAT_IDX(m, 1, (i + 2) % 3) * MAT_IDX(m, 2, (i + 1) % 3);
            }
            return det;
        default:
            Mat* temp = new_mat(m->rows-1, m->cols-1); // Submatrix for cofactor
            for (int j = 0; j < m->cols; j++) {
                float sign = (j % 2 == 0) ? 1.0f : -1.0f;
                
                int subi = 0;
                for (int i = 1; i < 4; i++) {
                    int subj = 0;
                    for (int k = 0; k < 4; k++) {
                        if (k == j) continue;
                        MAT_IDX(temp, subi, subj) = MAT_IDX(m, i, k);
                        subj++;
                    }
                    subi++;
                }
                det += sign * MAT_IDX(m, 0, j) * mat_determinant(temp);
            }
            free_mat(temp);
            return det;
    }
}

// Mat* mat_inverse(Mat* m) {
//     assert(m->rows == m->cols);
//     Mat* inverse = (Mat*)malloc(m->rows * m->cols);
//     inverse->rows = m->rows;
//     inverse->cols = m->cols;
//     float det = mat_determinant(m);
//     assert(det != 0);
//     Mat* adj = mat_adjoint(m);
//     mat_scalar_mult_buffer(adj, 1 / det, inverse);
//     free_mat(adj);
//     return inverse;
// }

// Mat* mat_adjoint(Mat* m) {
//     assert(m->rows == m->cols);
//     Mat* adj = (Mat*)malloc(m->rows * m->cols);
//     adj->rows = m->rows;
//     adj->cols = m->cols;
//     for (int i = 0; i < m->rows; i++) {
//         for (int j = 0; j < m->cols; j++) {
//             adj->data[i * adj->cols + j] = mat_cofactor(m, i, j);
//         }
//     }
//     return adj;
// }

// float mat_cofactor(Mat *m, int i, int j) {
//     assert(m->rows == m->cols);
//     assert(i < m->rows && j < m->cols);
//     float cofac = (1 - 2 * ((i * j) % 2)) * mat_determinant_recurs(m, m->rows, 0, m->rows, 0, m->cols, i, j);
// }
