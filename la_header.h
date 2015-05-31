/**
 * @file la_header.h
 * @author Danylo Malyuta <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief Linear Algebra header file.
 *
 * This is the header to la_funcs.c containing necessary definitions and
 * initializations.
 */

#ifndef LA_HEADER_H_
#define LA_HEADER_H_

/**
 * @struct MATRIX
 * This is the structure for a Matrix.
 */
struct MATRIX {
	size_t rows; ///< Number of rows
	size_t cols; ///< Number of columns
	float **matrix; ///< The dynamic 2D array
};

/** @cond INCLUDE_WITH_DOXYGEN */
struct MATRIX initMatrix(size_t rows, size_t cols);
struct MATRIX mmultiply(struct MATRIX A, struct MATRIX B);
struct MATRIX minverse_1x1 (struct MATRIX A);
struct MATRIX transpose(struct MATRIX A);
struct MATRIX madd(struct MATRIX A,struct MATRIX B);
struct MATRIX msubtract(struct MATRIX A,struct MATRIX B);
/** @endcond */

#endif /* LA_HEADER_H_ */
