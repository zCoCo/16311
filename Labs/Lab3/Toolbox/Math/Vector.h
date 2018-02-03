#ifndef _VECTOR_H
#define _VECTOR_H

  typedef union{
    float v[3];
  } Vector3x1;

  Vector3x1 AddVectors(Vector3x1 &pA, Vector3x1 &pB){
    Vector3x1 pResult;
    int i;
    /* This is error-checking, and wasn't required */
    if(pA == NULL || pB == NULL || pResult == NULL)
        return;
    /* Loop through the 3 elements of the vectors and add them; store in
     the vector pointed to by pResult */
    for(i = 0; i < 3; i++)
        pResult.v[i] = pA.v[i] + pB.v[i];

    return pResult;
  }

  Vector3x1 DotVectors(Vector3x1 &pA, Vector3x1 &pB){
    Vector3x1 pResult;
    int i;
    float sum;
    sum = 0;

    for(i=0; i<3; i++){
      sum += pA.v[i] * pB.v[i];
    }
    pResult.v[0] = sum; //again, just explicit, no {}memcpy
    pResult.v[1] = 0.0;
    pResult.v[2] = 0.0;

    return pResult;
  }

  Vector3x1 CrossVectors(Vector3x1 &pA, Vector3x1 &pB){
    Vector3x1 pResult;
    //Just explicit:
    pResult.v[0] = pA.v[1] * pB.v[2] - pA.v[2] * pB.v[1];
    pResult.v[1] = pA.v[2] * pB.v[0] - pA.v[0] * pB.v[2];
    pResult.v[2] = pA.v[0] * pB.v[1] - pA.v[1] * pB.v[0];

    return pResult;
  }

#endif // _VECTOR_H
