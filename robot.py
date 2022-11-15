#!/usr/bin/env python3
import rospy
import casadi as ca
import numpy as np
import math

class MobileRobotNMPC:
  def __init__(self, init_pose):
    self.T = 0.1          # time step
    self.N = 30            # horizon length

    self.Q = np.diag([50.0, 50.0, 10.0]) # Weight matrix for states  
    self.R = np.diag([1.0, 1.0, 1.0])    # Weight matrix for controls
    
    # Constraints
    self.min_vx = -1.0
    self.max_vx =  1.0

    self.min_vy = -1.0
    self.max_vy =  1.0

    self.min_omega = -0.7854
    self.max_omega =  0.7854

    self.max_dvx = 0.8
    self.max_dvy = 0.8
    self.max_domega = math.pi/6
    
    # The history states and controls
    self.next_states = np.ones((self.N+1, 3))*init_pose
    self.u0 = np.zeros((self.N, 3))
        
    print('init success !')
    pass
  
  def setup_controller(self):
    self.opti = ca.Opti()
    
    # state variable: position & velocity
    self.opt_states = self.opti.variable(self.N + 1, 3)
    x = self.opt_states[:,0]
    y = self.opt_states[:,1]
    yaw = self.opt_states[:,2]
    
    # the velocity
    self.opt_controls = self.opti.variable(self.N, 3)
    vx = self.opt_controls[:,0]
    vy = self.opt_controls[:,1]
    omega = self.opt_controls[:,2]
    
    
    pass

  
if __name__ == '__main__':
  print('Main node')
  
  init_pose = np.array([0.0, 0.0, 0.0])
  nmpc = MobileRobotNMPC(init_pose)
  nmpc.setup_controller()
  