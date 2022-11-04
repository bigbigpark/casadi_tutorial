#!/usr/bin/env python3
#-*-coding:utf-8 -*-
import rospy

from casadi import *

'''
SX: symbolic 타입으로 matrix(행렬) 형태를 선언해준다
scalar는 1x1 행렬 취급
'''
x = SX.sym('x')
y = SX.sym('y', 5)
Z = SX.sym('Z', 4, 2)

f = x**2 + 10
print(f)

'''
위처럼 primitives 없이 행렬 선언도 가능함
모든 성분이 0으로 구성되어 있음

Sparse matrix with structure zero: 실제 zero와 구분하기 위해 00으로 표기 되어 있음
Dense matrix with actual zero: @1=0 식으로 표현되어 있음
'''
B1 = SX.zeros(4, 5) # dense matrix
B2 = SX.ones(3,2)   # dense matrix
B3 = SX.eye(5)
B4 = SX(6,6) # structure zero

print(B1)
print(B2)
print(B3)
print(B4)

C = SX(9)
print(C)

D1 = SX([1,2,3,4])  # numpy
D2 = SX([[1,2],[3,4]]) # numpy

print(D1)
print(D2)

# 많이 사용하는 SX 표현들
'''
SX.sym(name,n,m): Create an n-by-m symbolic primitive
SX.zeros(n,m): Create an n-by-m dense matrix with all zeros
SX(n,m): Create an n-by-m sparse matrix with all structural zeros
SX.ones(n,m): Create an n-by-m dense matrix with all ones
SX.eye(n): Create an n-by-n diagonal matrix with ones on the diagonal
           and structural zeros elsewhere.


SX(scalar type): Create a scalar (1-by-1 matrix) with value given by the argument.
                 This method can be used explicitly,
                 e.g. SX(9), or implicitly, e.g. 9 ∗ SX.ones(2,2).
SX(matrix type): Create a matrix given a numerical matrix given as a NumPy or
                 SciPy matrix (in Python) or as a dense or sparse matrix (in MATLAB/Octave).
                 In MATLAB/Octave e.g. SX ([1,2,3,4]) for a row vector,
                 SX ([1;2;3;4]) for a column vector and SX ([1,2;3,4]) for a 2-by-2 matrix.
                 This method can be used explicitly or implicitly.
repmat(v,n,m): Repeat expression v n times vertically and m times horizontally.
repmat(SX(3),2,1) will create a 2-by-1 matrix with all elements 3.
(Python only) SX(list): Create a column vector (n-by-1 matrix) with the elements
                        in the list, e.g. SX ([1,2,3,4]) (note the difference 
                        between Python lists and MAT-LAB/Octave horizontal concatenation, 
                        which both uses square bracket syntax)
(Python only) SX(list of list ): Create a dense matrix with the elements in the lists,
                                 e.g. SX ([[1,2],[3,4]]) or a row vector (1-by-n matrix) 
                                 using SX ([[1,2,3,4]]).
'''