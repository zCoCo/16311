/* @file p1.h
 * 16-311 Problem Set 0
 * Part 1
 * Assignment: Create a basic Vector and Matrix Math Toolset
 * Author: Connor W. Colombo
****/
/*
 * Provided Core Attributes:
 * 16-311 Introduction to Robotics: Homework 0
 *
 * @author Hannah
 */

#ifndef p1_h
#define p1_h

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ENUMERATED VARIABLES (STRUCTURES) */

typedef struct vector{
    float v[3];
} vector;

vector g_vecs[26];

typedef struct matrix{
    float m[3][3];
} matrix;

matrix g_mats[26];

int IsVector( char* c );
int IsMatrix( char* c );

vector* GetVector( char* c );
matrix* GetMatrix( char* c );

void PrintVector( vector* pVec );
void PrintMatrix( matrix* mat );

void AddVectors( vector* pA, vector* pB, vector* pResult );
void AddMatrices ( matrix* pA, matrix* pB, matrix* pResult);
void DotVectors ( vector* pA, vector* pB, vector* pResult);
void CrossVectors ( vector* pA, vector* pB, vector* pResult);
void MultMatrices (matrix* pA, matrix* pB, matrix* pResult);
void MultMatVec ( vector* pV, matrix* pM, vector* pResult);

int Parse( char* pString );

#endif /* p1_h */
