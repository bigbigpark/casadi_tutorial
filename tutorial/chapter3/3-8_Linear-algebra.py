#!/usr/bin/env python3
#-*-coding:utf-8 -*-

from casadi import *

'''
CasADi는 일부 선형대수학 연산을 지원함
'''
A = MX.sym('A',3,3)
b = MX.sym('b',3)
print(solve(A,b))