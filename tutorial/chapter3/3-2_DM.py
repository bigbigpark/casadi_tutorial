#!/usr/bin/env python3
#-*-coding:utf-8 -*-
import rospy

from casadi import *
from numpy import array
from scipy.sparse import csc_matrix
'''
DM은 SX와 매우 유사하지만 nonzero인 값들이 numerial values이고 
symblic expressions가 아니라는 차이점 존재
문법은 SX.sym 같은 함수들 빼면 동일함

DM은 함수의 input & output으로서 주로 matrices를 storing할 때 사용됨.
하지만 computationally intensive calculation의 목적이 아님.
따라서 numpy(python)나 eigen(c++)로 자주 conversion 한다
'''
C = DM(2,3)
print(C)

C_dense = C.full()
C_dense = array(C) # equivalent
print(C_dense)

C_sparse = C.sparse()
C_sparse = csc_matrix(C) # equivalent
print(C_sparse)