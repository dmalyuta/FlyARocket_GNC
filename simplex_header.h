/**
 * @file simplex_header.h
 * @author Simplex header file <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief Simplex header file.
 *
 * This is the header to simplex_funcs.c containing necessary definitions and
 * initializations. This header supports, but was not provided with, the code
 * kindly provided code by Jean-Pierre Moreau <a href="http://jean-pierre.moreau.pagesperso-orange.fr/Cplus/tsimplex_cpp.txt">here</a>.
 * where the header definitions in this file were directly incorporated into the
 * code at the above url.
 *
 * License from the original cpp file:
 * @verbatim
***************************************************************
*          LINEAR PROGRAMMING: THE SIMPLEX METHOD             *
*------------------------------------------------------------ *
* ------------------------------------------------------------*
* Reference: "Numerical Recipes By W.H. Press, B. P. Flannery,*
*             S.A. Teukolsky and W.T. Vetterling, Cambridge   *
*             University Press, 1986" [BIBLI 08].             *
*                                                             *
*                       C++ Release 1.0 By J-P Moreau, Paris  *
*                                 (www.jpmoreau.fr)           *
***************************************************************
   @endverbatim
 */

#ifndef SIMPLEX_HEADER_H_
#define SIMPLEX_HEADER_H_

#define  MMAX  5 ///< Number of rows of the simplex table
#define  NMAX  6 ///< Number of columns of the simplex table
#define  REAL  double ///< Alias for a double

typedef REAL MAT[MMAX][NMAX]; ///< A [MMAXxNMAX] matrix

MAT  A; ///< Simplex table
int  IPOSV[MMAX], IZROV[NMAX];
int  i; ///< Index variable for loops
int j; ///< Index variable for loops
int ICASE,N,M,M1,M2,M3;

/** @cond INCLUDE_WITH_DOXYGEN */
void simplx(MAT a,int m,int n,int m1,int m2,int m3,int *icase,int *izrov, int *iposv);
void simp1(MAT a,int mm,int *ll,int nll,int iabf,int *kp,REAL *bmax);
void simp2(MAT a, int m,int n,int *l2,int nl2,int *ip,int kp,REAL *q1);
void simp3(MAT a,int i1,int k1,int ip,int kp);
void get_simplex_solution(int ICASE, int *IPOSV, MAT A, int M, int N, double *R1, double *R2, double *R3, double *R4);
/** @endcond */

#endif /* SIMPLEX_HEADER_H_ */
