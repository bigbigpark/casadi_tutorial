#!/usr/bin/env python3
#-*-coding:utf-8 -*-

from casadi import *

'''
여러 수학 연산 가능
- addition
- multiplications
- powers
- trigometric
'''

x = SX.sym('x')
y = SX.sym('y',2,2)
print(sin(y) - x)

'''
[Element-wise Matrix Multiplication]
C++/Python : *
Matlab: .*

[Matrix Multiplication]
mtimes(A,B)
'''
print(y*y) # 원소끼리만 곱하는거
print(mtimes(y,y)) # 원소 행렬 곱

'''
[Transpose]
C++    : A.T()
Python : A.T
Matlab : A' or A.'
'''
print(y)
print(y.T)

'''
[Reshaping]
행렬의 열과 행의 개수를 바꾸는거임
하지만 원소의 개수는 그대로 유지해야함
--> 아래 문법을 이용하면 계산 시간이 엄청 적게 듦
'''
x = SX.eye(4)
reshape_x = reshape(x,2,8)
print(reshape_x)

'''
[Concatenation]
정의: stacking matrices horizontally or vertically.
CasADi는 column major 방식으로 element를 저장하기 때문에 horizontally하는게 효율적임.
그리고 Matrices는 column vector의 집합이기 때문에 vertically로 해도 효율적임

--> vertcat(), horzcar()을 이용하면 됨
'''
x = SX.sym('x',3,1)
y = SX.sym('y',3,1)
print(vertcat(x,y))
print(horzcat(x,y))

'''
[Horizontal and vertical split]
정의: 위에서 언급한 concatenation의 inverse operation

first argument: 'offset vector' -> must be set 0 처음부터 시작이란 뜻
last argument: 'column 개수' -> 몇 개의 칼럼을 할 건데
'''
x = SX.sym('x',5,2)
w = horzsplit(x, [0,1,2])
print(w[0])
print(w[1])
v = vertsplit(x, [0,3,5])
print(v)