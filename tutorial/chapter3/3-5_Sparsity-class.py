#!/usr/bin/env python3
#-*-coding:utf-8 -*-

from casadi import *

'''
CasADi에서 행렬은 CCS(compressed column storage)를 이용하여 저장됨.
이것은 element-wise operations처럼 선형대수학 연산에서 standard임.
'''

# 자주 사용되는 Sparsity patterns
'''
Sparsity .dense(n,m): Create a dense n-by-m sparsity pattern
Sparsity(n,m): Create a sparse n-by-m sparsity pattern
Sparsity .diag(n): Create a diagonal n-by-n sparsity pattern
Sparsity .upper(n): Create an upper triangular n-by-n sparsity pattern
Sparsity .lower(n): Create a lower triangular n-by-n sparsity pattern
'''

A = SX.sym('x',3,3)
print(A)

B = SX.sym('x', Sparsity.upper(3))
print(B)

C = SX.sym('x', Sparsity.diag(5) )
print(C)

D = SX.eye(5)
print(D)

'''
C++/Python: index가 0부터 시작
Matlab: index가 1부터 시작

--> python의 -인덱스는 end부터 시작됨
'''
M = SX([[3,7], [3,5]])
print(M[0,:])
M[0,:]=1
print(M)

# CasADi에서는 slicing으로 값 대입 안 됨 !
M = SX([[3,7], [4,5]])
M[0,:][0,0] = 1
print(M)

# Single element access
M = diag(SX([3,4,5,6]))
print(M)
print(M[0,0])
print(M[1,1])
print(M[-1,-1])
print(M[1,:])
print(M[1:,1:4:2]) # 1행부터 시작하는데, 1열부터 3열까지 2열간격으로 

# List access 가능한데 slices access보다 비효율적
M = SX([[3,7,8,9], [4,5,6,1]])
print(M)
print(M[0, [0,3]])
print(M[0, 0:4:3])