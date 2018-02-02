#ifndef _MATRIX_H
#define _MATRIX_H

  #include "Vector3x1";

  typedef union{
    float m[3][3];
  } Matrix3x3;

  Matrix3x3 MultMatrices ( Matrix3x3 &pA, Matrix3x3 &pB){
    Matrix3x3 pResult;
    int ra, cb, i;
    float dotp;

    for(ra=0; ra<3; ra++){ // For each row vector in Matrix A of index ra
      for(cb=0; cb<3; cb++){ // with each column vector in Matrix B of index cb,
        dotp = 0;
        for(i=0; i<3; i++){ // Get the Dot Product of the two.
          dotp += pA.m[ra][i] * pB.m[i][cb];
        } // i<3
        pResult.m[ra][cb] = dotp; // And store the result in index ra, cb
      } // cb<3
    } // ra<3

    return pResult;
  } // #MultMatrices

  Vector3x1 MultMatVec ( Vector3x1 &pV, Matrix3x3 &pM){\
    Vector3x1 pResult;
    int cb, i;
    float dotp;

    for(cb=0; cb<3; cb++){ // For each column vector in the matrix of index cb
      dotp = 0;
      for(i=0; i<3; i++){ // get its dot product with the vector, pV.
        dotp += pV->v[i] * pM->m[i][cb];
      } // i<3
      pResult->v[cb] = dotp; // And store the result in index cb.
    } // cb<3
  }

#endif // _MATRIX_H
