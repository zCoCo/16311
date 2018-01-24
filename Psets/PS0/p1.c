/* @file p1.c
 * 16-311 Problem Set 0
 * Part 1
 * Assignment: Create a basic Vector and Matrix Math Toolset
 * Author: Connor W. Colombo
****/
/*
 * Provided Core Attributes:
 * 16-311 Introduction to Robotics: Homework 0
 *
 * @author Cuban
 * @author Howie
 * @author Brennan
 * @author Oscar
 * @author Hannah
 */

#include "p1.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

/* STEP 1: GO TO THE HEADER FILE AND UNDERSTAND THE ENUMERATED VARIABLES (STRUCTURES) */

/* STEP 3: UNDERSTAND THE IS VECTOR/MATRIX COMMANDS - similar to an islower command */

int IsVector( char* c )
{
    /* *c is the first character in the string pointed to by the
     character pointer c, and thus can be compared with the constants
     'a' and 'z'.  Note that 'a' is a character, whereas "a" is a string
     (and is thus null-terminated).  The lower-case letters are contiguous
     in the ASCII table, so we can just check if we're between the boundaries.
     */
    if( (*c) >= 'a' && (*c) <= 'z' ) return 1;
    return 0;
}

int IsMatrix( char* c )
{
    /* Same as IsVector, 'cept with different constants */
    if( (*c) >= 'A' && (*c) <= 'Z' ) return 1;
    return 0;
}

/* SETP 4: UNDERSTAND THE GET COMMANDS - We are creating pointers to an element in the array*/

vector* GetVector( char* c ) // Please do not change this function
{
    /* Funny-looking, isn't it?  Here's the basic idea:
     (*c) - 'a' gives you a number in the range [0 25], since char's can
     be treated as integers in the range [0 255], and subtracting 'a's
     value from a lower-case letter shifts *c down to the [0 25] range.
     the "&" gives you the address of the element of g_vecs we're interested
     in, since we're returning a pointer.
     */

    return &g_vecs[ (*c) - 'a' ];
}

matrix* GetMatrix( char* c )
{
    /* Same as GetVector, 'cept with a different constant */
    return &g_mats[ (*c) - 'A' ];
}

/* STEP 5: UNDERSTAND THE PRINT COMMANDS */

void PrintVector( vector* pVec )
{
    /* %f prints a float.
     pVec is a pointer to a vector, so you must dereference using the
     -> operator, then access the member array v
     */
    printf("%f %f %f\n",
           pVec->v[0],
           pVec->v[1],
           pVec->v[2] );
    printf("\n");
}

void PrintMatrix( matrix* mat )
{
    /* Put newlines (\n's) in strategic places to make it
     look pretty.  The 5 in %.5f makes each number take
     print exactly 5 digits following the "."
     */
    printf("%.5f %.5f %.5f\n%.5f %.5f %.5f\n%.5f %.5f %.5f\n",
           mat->m[0][0], mat->m[0][1], mat->m[0][2],
           mat->m[1][0], mat->m[1][1], mat->m[1][2],
           mat->m[2][0], mat->m[2][1], mat->m[2][2]);
    printf("\n");

}

/* STEP 6: ADD YOUR CODE HERE - This is where we will do the actual arithmetic. */

/* One function to get you started.*/
void AddVectors( vector* pA, vector* pB, vector* pResult )
{
    int i;
    /* This is error-checking, and wasn't required */
    if(pA == NULL || pB == NULL || pResult == NULL)
        return;
    /* Loop through the 3 elements of the vectors and add them; store in
     the vector pointed to by pResult */
    for(i = 0; i < 3; i++)
        pResult->v[i] = pA->v[i] + pB->v[i];
}

void AddMatrices( matrix* pA, matrix* pB, matrix* pResult){
  int i, j;

  for(i=0; i<3; i++){
    for(j=0; j<3; j++){
      pResult->m[i][j] = pA->m[i][j] + pB->m[i][j];
    }
  }

}

void DotVectors ( vector* pA, vector* pB, vector* pResult){
  int i;
  float sum;
  sum = 0;

  for(i=0; i<3; i++){
    sum += pA->v[i] * pB->v[i];
  }
  pResult->v[0] = sum; //again, just explicit, no {}memcpy
  pResult->v[1] = 0.0;
  pResult->v[2] = 0.0;
}

void CrossVectors ( vector* pA, vector* pB, vector* pResult){
  //Just explicit:
  pResult->v[0] = pA->v[1] * pB->v[2] - pA->v[2] * pB->v[1];
  pResult->v[1] = pA->v[2] * pB->v[0] - pA->v[0] * pB->v[2];
  pResult->v[2] = pA->v[0] * pB->v[1] - pA->v[1] * pB->v[0];
}

void MultMatrices ( matrix* pA, matrix* pB, matrix* pResult){
  int ra, cb, i;
  float dotp;

  for(ra=0; ra<3; ra++){ // For each row vector in Matrix A of index ra
    for(cb=0; cb<3; cb++){ // with each column vector in Matrix B of index cb,
      dotp = 0;
      for(i=0; i<3; i++){ // Get the Dot Product of the two.
        dotp += pA->m[ra][i] * pB->m[i][cb];
      } // i<3
      pResult->m[ra][cb] = dotp; // And store the result in index ra, cb
    } // cb<3
  } // ra<3
} // #MultMatrices

void MultMatVec ( vector* pV, matrix* pM, vector* pResult){
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

/* STEP 2: UNDERSTAND THE PARSE FUNCTION - Here we get our input into ptokens, and then start deciding what to do with it. If there is only one ptoken, this is easy, print out somehing. If there are 11 tokens, this is easy too, it is assign a matrix. Everything else has five tokens. Note, there is a perhaps confusing decision tree later on summarizing this */

int Parse( char* pString )
{
    char* pTokens[11]; // The most tokens we'll get is 11 (matrix assignment).
    char* pCurrTok;
    int nTokens;
    int i, j;
    matrix* mat;
    vector* vec;

    nTokens = 0;

    // Early-out.  If the string is equal to END then end.

    if( 0 == strcmp( "END", pString ) )  /* strcmp compares two strings */
    {
        // Time to quit
        printf("Ending session");
        return 1;
    }

    // User enters help. If the string is equal to HELP, display help information
    else if ( 0 == strcmp( "HELP", pString ) )
    {
        // Print further instructions for formatting
        printf("To enter a vector, use a lower case letter and enter the vector in the following format:\nv_=_a_b_c where each underscore is a space and a, b and c are floats.\nTo enter a matrix, use an upper case letter and enter the matrix in the following format: M_=_a_b_c_d_e_f_g_h_i where each underscore is a space and each letter a through i is a float.\n                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 Valid commands are:\n1. Assigning a vecor or matrix to a variable\n2. Reprinting a vector or matrix\n3. Adding two vectors or two matrices. (c = a + b or C = A + B)\n4. The dot product of two vectors, the cross product of two vectors, the product of two matrices or the product of a vector times a matrix.(c = a . b, c = a * b, C = A * B or c = A * b)");
        return 0;
    }

    /* SEE HOW pTokens is stuffed */
    pCurrTok = (char*)strtok( pString, " \n" );
    while( pCurrTok )
    {
        // Assign the current token.
        pTokens[nTokens++] = pCurrTok;

        // Grab the next one.
        pCurrTok = (char*)strtok( NULL, " \n" );
    }

    /* The following is a decision tree. If you find this confusing, please skit it.
     After you read the code, come back to it and it will make loads of sense */
    /* Decision tree:

     L1: Number of tokens:
     1:  Print
     11: Matrix assignment
     5:  L2
     3rd token a digit?
     Y: vector assignment
     N: L3
     Is first token a vector?
     Y: L4 (a vector op)
     Is 4th token a...
     "+": vector addition
     ".": vector dot product
     "*": Is 3rd token a vector?
     Y: vector cross product
			  N: matrix/vector multiplication
     N: L4 (a matrix op)
     Is 4th token a...
     "+": matrix addition
     "*": matrix multiplication
     */


    switch(nTokens) {
            /* Print case (only one token) */

            /* STEP 6: UNDERSTAND  VECTOR  assignment. Go look at some previously
             defined functions now */
        case 1:  /* Only one thing was inputted, so it is either a vector or a matrix */

            if(IsVector(pTokens[0])) /*  If a lower case letter, then this function
                                      returns a 1 which is like "true," 0 is false
                                      Note, we could have used islower and isupper */
            {
                PrintVector(GetVector(pTokens[0]));
            }
            else if(IsMatrix(pTokens[0])){
                PrintMatrix(GetMatrix(pTokens[0]));
            }
            else
                return 0;  /* If the character is not a letter, uppoer or lower case, then
                            return a zero */
            break;

            /* Matrix assignment (only operation with 11 tokens) */
        case 11:
            printf("Assigning to matrix\n");
            mat = GetMatrix(pTokens[0]); /* Returns a POINTER to a matrix in g_mats[ ],
                                          so mat is a pointer to a structure */
            for(i = 0; i < 3; i++)
                for(j = 0; j < 3; j++) {
                    mat->m[i][j] = atof(pTokens[2 + 3*i + j]); /* one-dim --> two-dim */
                }
            break;

            /* All kinds of foo (there are many operations with 5 tokens)*/
        case 5:

            /* Check if the third token is a digit or minus sign (=> assignment) */
            if(isdigit(pTokens[2][0]) || pTokens[2][0] == '-') {
                printf("Assigning to vector\n");
                /* Vector assignment */
                vec = GetVector(pTokens[0]);
                for(i = 0; i < 3; i++)
                    vec->v[i] = atof(pTokens[2 + i]);
            }

            else if(IsVector(pTokens[0])) {
                if(0 == strcmp( pTokens[3], "+")) {
                    printf("Adding vectors\n");
                    AddVectors(GetVector(pTokens[2]), GetVector(pTokens[4]), GetVector(pTokens[0]));
                }

                else if( 0 == strcmp( pTokens[3], ".")) {
                    printf("Dotting vectors\n");
                    DotVectors(GetVector(pTokens[2]), GetVector(pTokens[4]), GetVector(pTokens[0]));
                }

                else if( 0 == strcmp( pTokens[3], "*")) {
                    if(IsVector(pTokens[4])) {
                        printf("Crossing vectors\n");
                        CrossVectors(GetVector(pTokens[2]), GetVector(pTokens[4]), GetVector(pTokens[0]));
                    }
                    else {
                        printf("Doing Matrix-Vector multiplication\n");
                        MultMatVec(GetVector(pTokens[2]), GetMatrix(pTokens[4]), GetVector(pTokens[0]));
                    }
                }
            } /* END OF THE ISVECTORS */

            else if(IsMatrix(pTokens[0])) {
                if(0 == strcmp(pTokens[3], "+")) {
                    printf("Adding matrices\n");
                    AddMatrices(GetMatrix(pTokens[2]), GetMatrix(pTokens[4]), GetMatrix(pTokens[0]));
                }
                else if(0 == strcmp(pTokens[3], "*")) {
                    printf("Multiplying matrices\n");
                    MultMatrices(GetMatrix(pTokens[2]), GetMatrix(pTokens[4]), GetMatrix(pTokens[0]));
                }
            }
            break;
        default: return 0;
    }

    // Don't quit
    return 0; // Tells main to keep going
}
