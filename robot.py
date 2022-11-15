#!/usr/bin/env python3
import rospy
import casadi as ca
import numpy as np
import math

import utilities
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
# from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MobileRobotNMPC:
  def __init__(self, init_pose):
    self.dt = 0.1          # time step
    self.N = 20            # horizon length

    self.Q = np.diag([50.0, 50.0, 10.0]) # Weight matrix for states  
    self.R = np.diag([1.0, 1.0])    # Weight matrix for controls
    
    # Constraints
    self.min_v = -1.0
    self.max_v =  1.0
    self.min_w = -0.7854
    self.max_w =  0.7854

    self.min_dv = -0.8
    self.max_dv =  0.8
    self.min_dw = -math.pi/6
    self.max_dw =  math.pi/6
    
    # The history states and controls
    self.next_states = np.ones((self.N+1, 3))*init_pose
    self.u0 = np.zeros((self.N, 2))
        
    # ROS
    rospy.init_node('robot_node', anonymous=True)
    self.r = rospy.Rate(1/self.dt)
    self.odom_sub_ = rospy.Subscriber('/rbt1/odom_ground_truth', Odometry, self.odom_callback, queue_size=1)
    self.cmd_vel_pub_ = rospy.Publisher('/rbt1/jackal_velocity_controller/cmd_vel', Twist, queue_size=1)
    self.predict_path_pub_ = rospy.Publisher('/rbt1/predicted_path', Path, queue_size=1)
    
    self.xpose = np.zeros((3,1))
    
    print('init success !')
    pass
  
  def odom_callback(self, msg):
    roll, pitch, yaw = utilities.euler_from_quaternion(msg.pose.pose.orientation.x,
                                                       msg.pose.pose.orientation.y,
                                                       msg.pose.pose.orientation.z,
                                                       msg.pose.pose.orientation.w,)
    self.xpose[0] = msg.pose.pose.position.x
    self.xpose[1] = msg.pose.pose.position.y
    self.xpose[2] = yaw
    # print('xpose: {}'.format(self.xpose))
    
  def setup_controller(self):
    self.opti = ca.Opti()
    
    # state variable: position & velocity
    self.opt_states = self.opti.variable(self.N+1, 3)
    x = self.opt_states[:,0]
    y = self.opt_states[:,1]
    theta = self.opt_states[:,2]
    
    # the control input
    self.opt_controls = self.opti.variable(self.N, 2)
    v = self.opt_controls[:,0]
    w = self.opt_controls[:,1]
    
    # create model
    f = lambda x_, u_: ca.vertcat(*[
      u_[0]*ca.cos(x_[2]),  # dx
      u_[0]*ca.sin(x_[2]),  # dy
      u_[1]                 # dtheta
    ])
    
    # parameters, these parameters are the reference traj. of the pose and inputs
    self.opt_states_ref = self.opti.parameter(self.N+1, 3)
    self.opt_controls_ref = self.opti.parameter(self.N, 2)
    
    # Initial Condition
    self.opti.subject_to(self.opt_states[0,:] == self.opt_states_ref[0,:])
    for i in range(self.N):
      states_next = self.opt_states[i,:] + f(self.opt_states[i,:], self.opt_controls[i,:]).T*self.dt # state propagation
      self.opti.subject_to(self.opt_states[i+1,:] == states_next)

    # Cost Function
    obj = 0
    for i in range(self.N):
      state_error   = self.opt_states[i,:]   - self.opt_states_ref[i,:]
      control_error = self.opt_controls[i,:] - self.opt_controls_ref[i,:]

      obj = obj + state_error @ self.Q @ state_error.T \
                + control_error @ self.R @ control_error.T
      self.opti.minimize(obj)
    
    # constraint about change of the velocity
    for i in range(self.N-1):
      dvel = (self.opt_controls[i+1,:] - self.opt_controls[i,:]) / self.dt
      self.opti.subject_to(self.opti.bounded(self.min_dv, dvel[0], self.max_dv))
      self.opti.subject_to(self.opti.bounded(self.min_dw, dvel[1], self.max_dw))
      
    # constraint about the velocity
    for i in range(self.N):
      vel = self.opt_controls[i,:]
      self.opti.subject_to(self.opti.bounded(self.min_v, vel[0], self.max_v))
      self.opti.subject_to(self.opti.bounded(self.min_w, vel[1], self.max_w))
      
    # solver options
    options = {
      'ipopt.max_iter' : 2000,
      'ipopt.print_level' : 0,
      'print_time' : 0,
      'ipopt.acceptable_tol' : 1e-8,
      'ipopt.acceptable_obj_change_tol' : 1e-6
    }
    
    self.opti.solver('ipopt', options)
    
  def solve(self, ref_traj):
    ref_controls = np.zeros((self.N, 2))
    
    # set parameter, here only update initial state of x (x0)
    self.opti.set_value(self.opt_states_ref, ref_traj)
    self.opti.set_value(self.opt_controls_ref, ref_controls)
    
    # provide the initial guess of the optimizaiton targets
    self.opti.set_initial(self.opt_states, self.next_states)
    self.opti.set_initial(self.opt_controls, self.u0)
    
    # solve the problem
    sol = self.opti.solve()
    
    cmd_vel = sol.value(self.opt_controls)
    
    return cmd_vel

  def generate_ref_traj(self, time_stamp):
    ref_traj = np.zeros((self.N+1, 3))
    nominal_speed = 0.2
    
    for i in range(np.shape(ref_traj)[0]):
      ref_traj[i,0] = nominal_speed*(time_stamp + i*nominal_speed)  # ref_x
      ref_traj[i,1] = 0.0  # ref_y
      ref_traj[i,2] = 0.0  # ref_theta
    # print(ref_traj)
    
    return ref_traj

if __name__ == '__main__':
  print('Main node')
  
  init_pose = np.array([0.0, 0.0, 0.0])
  nmpc = MobileRobotNMPC(init_pose)
  nmpc.setup_controller()
  
  time_stamp = 0.0
  while not rospy.is_shutdown():
    print('time_stamp: {}'.format(time_stamp))
    time_stamp += nmpc.dt
    
    # 1. Generate Reference Traj.
    ref_traj = nmpc.generate_ref_traj(time_stamp)    
    
    # 2. Optimize the Traj.
    cmd_vel = nmpc.solve(ref_traj)
    
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = cmd_vel[0][0]
    cmd_vel_msg.angular.z = cmd_vel[0][1]
    nmpc.cmd_vel_pub_.publish(cmd_vel_msg)
    
    nmpc.r.sleep()