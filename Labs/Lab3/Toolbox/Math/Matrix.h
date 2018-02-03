#ifndef _MATRIX_H
#define _MATRIX_H

  #include "Vector.h";

  typedef union{
    float m[3][3];
  } Matrix3x3;

  Matrix3x3 AddMatrices(Matrix3x3 &pA, Matrix3x3 &pB){
    Matrix3x3 pResult;
    int i, j;

    for(i=0; i<3; i++){
      for(j=0; j<3; j++){
        pResult.m[i][j] = pA.m[i][j] + pB.m[i][j];
      }
    }

    return pResult;
  }

  Matrix3x3 MultMatrices(Matrix3x3 &pA, Matrix3x3 &pB){
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

  Vector3x1 MultMatVec(Matrix3x3 &pM, Vector3x1 &pV){
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

    return pResult;
  } // #MultMatVec

  // Matrix Determinant
  float DetMat(Matrix3x3 &pM){
    return ( pM.m[0][0] * ( pM.m[1][1]*pM.m[2][2] - pM.m[2][1]*pM.m[1][2])
           - pM.m[0][1] * ( pM.m[1][0]*pM.m[2][2] - pM.m[1][2]*pM.m[2][0])
           - pM.m[0][2] * ( pM.m[1][0]*pM.m[2][1] - pM.m[1][1]*pM.m[2][0]) );
  } // #DetMat

  Matrix3x3 InverseMat(Matrix3x3 &pM){
    Matrix3x3 pRes;

    float det = DetMat(pM);
    if(det){ //!=0
      invdet = 1/det;

      pRes.m[0][0] = invdet*(pM.m[1][1]*pM.m[2][2] - pM.m[2][1]*pM.m[1][2]);
      pRes.m[0][1] = invdet*(pM.m[0][2]*pM.m[2][1] - pM.m[0][1]*pM.m[2][2]);
      pRes.m[0][2] = invdet*(pM.m[0][1]*pM.m[1][2] - pM.m[0][2]*pM.m[1][1]);

      pRes.m[1][0] = invdet*(pM.m[1][2]*pM.m[2][0] - pM.m[1][0]*pM.m[2][2]);
      pRes.m[1][1] = invdet*(pM.m[0][0]*pM.m[2][2] - pM.m[0][2]*pM.m[2][0]);
      pRes.m[1][2] = invdet*(pM.m[1][0]*pM.m[0][2] - pM.m[0][0]*pM.m[1][2]);

      pRes.m[2][0] = invdet*(pM.m[1][0]*pM.m[2][1] - pM.m[2][0]*pM.m[1][1]);
      pRes.m[2][1] = invdet*(pM.m[2][0]*pM.m[0][1] - pM.m[0][0]*pM.m[2][1]);
      pRes.m[2][2] = invdet*(pM.m[0][0]*pM.m[1][1] - pM.m[1][0]*pM.m[0][1]);

    } // det?

    return pRes;
  } // #InverseMat

#endif // _MATRIX_H
