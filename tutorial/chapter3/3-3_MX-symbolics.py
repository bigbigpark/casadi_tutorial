#!/usr/bin/env python3
#-*-coding:utf-8 -*-

from casadi import *

# SX
'''
SX를 이용한 symbolic 표현에서 어떻게 행렬 곱이랑 덧셈이 행해지는지 알 수 있음
output인 f는 2x2 행렬임
SX는 elementary operation 표현이 가능
'''
x = SX.sym('x', 2, 2)
y = SX.sym('y')
f = 3*x + y
print(f)
print(f.shape)

# MX
'''
More negeral matrix expression type: MX
MX는 elementary operation이 제한됨
'''
x = MX.sym('x', 2, 2)
y = MX.sym('y')
f = 3*x + y
print(f)
print(f.shape)

'''
MX는 SX와 같이 elements의 set, get 가능, but 구현 방법이 다름
'''
x = MX.sym('x', 2, 2)
print(x[0,0])

x = MX.sym('x', 2)
A = MX( 2 , 2 )
A[0,0] = x[0]
A[1,1] = x[0] + x[1]
print(A)