#!/usr/bin/env python3
#-*-coding:utf-8 -*-

from casadi import *

'''
단순히 SX 객체와 MX 객체를 곱할 수는 없음
같은 표현으로 operation이 불가능 함 
--> 하지만 SX 안에 정의된 함수를 MX에서 호출할 수 있음 !
Chapter 4에서 자세히 다룰 것

SX와 MX의 mixing은 종종 좋은 아이디어임
왜냐하면 SX 표현은 연산당 overhead가 적음 따라서 scalar operation에서 빠르다

=> SX는 low level operation을 위해서 사용되어 짐
=> MX는 constraint function의 formulation같은 NLP 문제 해결에 사용됨
'''