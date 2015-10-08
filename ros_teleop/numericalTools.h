
#ifndef numericalTools_h
#define numericalTools_h

#include <Arduino.h>

#define _recast(a, _a, m) float (*a)[m] = (float(*)[m]) _a

void multMat(float** _c, float** _a, float** _b, int n1, int n2, int n3);
void calcJacobian( void (*func)(float*, float*), float* X, int n, float** _J );
void LUDecomp( float** _A, float** _L, float** _U, float** _P, int n );
void solveLS( float** _A, float** _B, int n, float** _X );
int solveNLS( void (*func)(float*, float*), float* Y, int n, float* X );
void printMatrix( float** _M, int n, int m, char* msg );

#endif
