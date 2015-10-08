#include <Arduino.h>
#include "numericalTools.h"

#define _recast(a, _a, m) float (*a)[m] = (float(*)[m]) _a

void multMat(float** _c, float** _a, float** _b, int n1, int n2, int n3) {
  
  float Y[n1][n3];
  _recast( a, _a, n2 );
  _recast( b, _b, n3 );
  _recast( c, _c, n3 );
  float tmp;
        
  for( int i = 0; i < n1; i++ )
    for( int j = 0; j < n3; j++ ) {
      tmp = 0;
      for( int k = 0; k < n2; k++ )
          tmp += a[i][k]*b[k][j];
      Y[i][j] = tmp;
    }
    
  for( int i = 0; i < n1; i++ )
    for( int j = 0; j < n3; j++ )
      c[i][j] = Y[i][j];
}

void calcJacobian( void (*func)(float*, float*), float* X, int n, float** _J ) {
  // approximate jacobian as finite difference
  float (*J)[n] = (float(*)[n]) _J;
  const float eps = 1e-4;
  float Ypp[n], Ymm[n];
    
  for( int j = 0; j < n; j++ ) {
    
    X[j] += eps;
    (*func)( X, Ypp );
    X[j] -= eps;
    
    X[j] -= eps;
    (*func)( X, Ymm );
    X[j] += eps;
    
    for( int i = 0; i < n; i++ )
      J[i][j] = ( Ypp[i] - Ymm[i] ) / ( 2 * eps );
  }
}

void LUDecomp( float** _A, float** _L, float** _U, float** _P, int n ) {
  
  _recast( A, _A, n );
  _recast( L, _L, n );
  _recast( U, _U, n );
  _recast( P, _P, n );
  
  for( int i = 0; i < n; i++ )
    for( int j = 0; j < n; j++ ) {
      P[i][j] = (i == j);
      L[i][j] = (i == j);
      U[i][j] = 0;
    }
  
  for( int i = 0; i < n; i++ ) {
    int max_j = i;
    for( int j = i; j < n; j++ )
      if( fabs(A[j][i]) > fabs(A[max_j][i]) )
        max_j = j;
    if( max_j != i )
      for( int k = 0; k < n; k++ ) {
        float temp = P[i][k];
        P[i][k] = P[max_j][k];
        P[max_j][k] = temp;
      }
  }
  
  multMat( (float**) U, (float**) P, (float**) A, n, n, n );
    
  for( int j = 0; j < n-1; j++ )
    for( int i = j + 1; i < n; i++ ) {
      float tmp = -U[i][j] / U[j][j];
      for( int k = j; k < n; k++ )
        U[i][k] = tmp*U[j][k] + U[i][k];
      L[i][j] = -tmp;
    }
}

void solveLS( float** _A, float** _B, int n, float** _X ) {
  // solve A*X = B using LU Decomposition
  _recast( A, _A, n );
  _recast( B, _B, 1 );
  _recast( X, _X, 1 );
  float L[n][n], U[n][n], P[n][n];
  float Y[n];
  float BB[n][1];
  
  
  LUDecomp( (float**) A, (float**) L, (float**) U, (float**) P, n );
  multMat( (float**) BB, (float**) P, (float**) B, n, n, 1 );
    
  // Forward pass
  for( int i = 0; i < n; i++ ) {
    float tmp = 0;
    for( int j = 0; j <= i-1; j++ )
      tmp += L[i][j]*Y[j];
    Y[i] = ( BB[i][0] - tmp ) / L[i][i];
  }
  
  // Backward pass
  for( int i = n-1; i >= 0; i-- ) {
    float tmp = 0;
    for( int j = i+1; j < n; j++ )
      tmp += U[i][j]*X[j][0];
    X[i][0] = ( Y[i] - tmp ) / U[i][i];
  }
  
//  printMatrix( (float**) A, n, n, "\nA = " );
//  printMatrix( (float**) L, n, n, "\nL = " );
//  printMatrix( (float**) U, n, n, "\nU = " );
//  printMatrix( (float**) P, n, n, "\nP = " );
//  printMatrix( (float**) X, n, 1, "\nX = " );
//  printMatrix( (float**) B, n, 1, "\nB = " );
//  printMatrix( (float**) BB, n, 1, "\nBB = " );
}

int solveNLS( void (*func)(float*, float*), float* Y, int n, float* X ) { 
  // solve Y = func(X) as Multivariate Newton Method
  // return number of iterations or -1, in case of divergence
  float J[n][n];
  float B[n], dX[n];
  float tmp;
  const float TOL_SQR = 1e-6;
  const int MAX_ITER = 100;
  
  for( int iter = 0; iter < MAX_ITER; iter++ ) {
    
    // calculate jacobian J
    calcJacobian( func, X, n, (float**) J );

    // negate J
    for( int i = 0; i < n; i++ )
      for( int j = 0; j < n; j++ )
        J[i][j] = -J[i][j];
    
    // set B = F(X) - Y
    func( X, B );
    for( int j = 0; j < n; j++ )
      B[j] -= Y[j];
      
    // calculate dX = J \ ( F(X) - Y )
    solveLS( (float**) J, (float**) B, n, (float**) dX );
    
    // increment X = X + dX and check update rate
    tmp = 0.0;
    for( int j = 0; j < n; j++ ) {
      X[j] += dX[j];
      tmp += dX[j]*dX[j];
    }
    
    if( tmp < TOL_SQR )
      return iter;
  }
  return -1;
}

void printMatrix( float** _M, int n, int m, char* msg ) {
  float (*M)[m] = (float(*)[m]) _M;
  
  Serial.println( msg );
  for( int i = 0; i < n; i++ ) {
    for( int j = 0; j < m; j++ ) {
      Serial.print( M[i][j] );
      Serial.print('\t');
    }
    Serial.print('\n');
  }
}
