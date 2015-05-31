/**
 * @file la_funcs.c
 * @author Danylo Malyuta <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief Lienar Algebra functions source file.
 *
 * This file contains necessary linear algebra functions such as matrix initialization,
 * multiplication, addition, subtraction, etc. These functions are used primarily in the Kalman_filter()
 * function (found in imu_funcs.c)
 */

# include <stdint.h>
# include <stdlib.h>
# include <stdio.h>
# include "la_header.h"

/**
 * @fn struct MATRIX initMatrix(size_t rows, size_t cols)
 *
 * This function allocates space for a [rows x cols] size matrix.
 * Attention : returned matrix A is just pointers that don't point to any default
 * value!
 *
 * @param rows Number of matrix rows.
 * @param cols Number of matrix columns.
 */
struct MATRIX initMatrix(size_t rows, size_t cols) {
	struct MATRIX A;
	A.rows = rows;
	A.cols = cols;
	A.matrix = (float**)malloc(A.cols*sizeof(float*));
	int ii;
	for (ii=0;ii<A.rows;ii++) {
		A.matrix[ii] = (float*)malloc(A.rows*sizeof(float));
	}
	return A;
}

/**
 * @fn struct MATRIX mmultiply(struct MATRIX A, struct MATRIX B)
 *
 * This fuction returns C=A*B where C is returned from the function and A,B are previously
 * defined matrices
 *
 * @param A The pre-multiplied matrix
 * @param B The post-multiplied matrix
 */
struct MATRIX mmultiply(struct MATRIX A, struct MATRIX B) {
	int ii; int jj; int kk;
	struct MATRIX C=initMatrix(A.rows,B.cols);
	for (ii=0;ii<A.rows;ii++) {
		for (jj=0;jj<B.cols;jj++) {
			C.matrix[ii][jj] = 0.0; // Initialize
			for (kk=0;kk<A.cols;kk++) {
				C.matrix[ii][jj] = C.matrix[ii][jj] + A.matrix[ii][kk]*B.matrix[kk][jj];
			}
		}
	}
	return C;
}

/**
 * @fn struct MATRIX madd(struct MATRIX A,struct MATRIX B)
 *
 * This function return C=A+B (matrix addition).
 *
 * @param A The first matrix.
 * @param B The second matrix.
 */
struct MATRIX madd(struct MATRIX A,struct MATRIX B) {
	struct MATRIX C=initMatrix(A.rows,A.cols);
	int ii; int jj;
	for (ii=0;ii<A.rows;ii++) {
		for (jj=0;jj<A.cols;jj++) {
			C.matrix[ii][jj]=A.matrix[ii][jj]+B.matrix[ii][jj];
		}
	}
	return C;
}

/**
 * @fn struct MATRIX msubtract(struct MATRIX A,struct MATRIX B)
 *
 * This function returns C=A-B (matrix subtraction)
 *
 * @param A The first matrix.
 * @param B The second matrix.
 */
struct MATRIX msubtract(struct MATRIX A,struct MATRIX B) {
	struct MATRIX C=initMatrix(A.rows,A.cols);
	int ii; int jj;
	for (ii=0;ii<A.rows;ii++) {
		for (jj=0;jj<A.cols;jj++) {
			C.matrix[ii][jj]=A.matrix[ii][jj]-B.matrix[ii][jj];
		}
	}
	return C;
}

/**
 * @fn struct MATRIX minverse_1x1 (struct MATRIX A)
 *
 * This function inverses a 1 by 1 matrix - i.e. a scalar, but in MATRIX format (as struct MATRIX)
 * that is usable in a matrix expression, i.e. doing ([1x2]*[2x1])+[1x1]^(-1).
 *
 * @param A [1x1] matrix.
 */
struct MATRIX minverse_1x1 (struct MATRIX A) {
	struct MATRIX B=initMatrix(1,1);
	B.matrix[0][0]=1/(A.matrix[0][0]);
	return B;
}

/**
 * @fn struct MATRIX transpose(struct MATRIX A)
 *
 * This function transposes a matrix, returning B=A' (real transpose of A).
 *
 * @param A A matrix.
 */
struct MATRIX transpose(struct MATRIX A) {
	struct MATRIX A_T=initMatrix(A.cols,A.rows);
	int ii; int jj;
	for (ii=0;ii<A_T.rows;ii++) {
		for (jj=0;jj<A_T.cols;jj++) {
			A_T.matrix[ii][jj]=A.matrix[jj][ii];
		}
	}
	return A_T;
}
