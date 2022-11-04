#!/usr/bin/env python3
#-*-coding:utf-8 -*-

from casadi import *

'''
Matrix나 Sparsity pattern을 확인할 수 있음
'''

y = SX.sym('y', 10,1)
print(y)
print(y.size1()) # row
print(y.size2()) # col
print(y.shape) # size(nrow, ncol)
print(y.numel) # the number of element : nrow*ncol
print(y.is_dense())
print(y.is_constant())

'''
[자주 사용되는 Querying properties]
A.size1() The number of rows
A.size2() The number of columns
A.shape (in MATLAB ”size”) The shape, i.e. the pair (nrow,ncol )
A.numel() The number of elements, i.e nrow ∗ ncol
A.nnz() The number of structurally nonzero elements, equal to A.numel() if dense.
A.sparsity() Retrieve a reference to the sparsity pattern
A.is dense() Is a matrix dense, i.e. having no structural zeros
A.is scalar() Is the matrix a scalar, i.e. having dimensions 1-by-1?
A.is column() Is the matrix a vector, i.e. having dimensions n-by-1?
A.is square() Is the matrix square?
A.is triu() Is the matrix upper triangular?
A.is constant() Are the matrix entries all constant?
A.is integer() Are the matrix entries all integer-valued?
'''