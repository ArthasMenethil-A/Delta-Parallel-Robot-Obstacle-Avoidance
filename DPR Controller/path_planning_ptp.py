
# =================================================================================================
# -- imports --------------------------------------------------------------------------------------
# =================================================================================================

from delta_robot import DeltaRobot 
from delta_robot import tand, sind, cosd

import numpy as np 
import math 
import matplotlib.pyplot as plt 
from scipy.optimize import fsolve


# =================================================================================================
# -- point to point path planning class -----------------------------------------------------------
# =================================================================================================

class PathPlannerPTP: 
	def __init__(self, robot, ee_pos_i, ee_pos_f, thetadot_max):
		self.ee_pos_i 	= np.array(ee_pos_i)
		self.ee_pos_f 	= np.array(ee_pos_f)
		self.thetadot_max 	= thetadot_max*6 # convert rpm to deg/s
		self.theta_i = robot.inverse_kin(self.ee_pos_i).reshape((3, 1)) 
		self.theta_f = robot.inverse_kin(self.ee_pos_f).reshape((3, 1))
		self.robot = robot

		# print(self.theta_i)
		# print(self.theta_f)


	def point_to_point_345(self):
		FREQUENCY = 1000 

		# overall time period
		T = 15/8*(self.theta_f - self.theta_i)/self.thetadot_max
		T = [1 for _ in T]
		T = math.floor(max(T)*FREQUENCY)
		tau = np.array(range(0, T))/T


		# theta time profile
		s_tau = 6*tau**5 - 15*tau**4 + 10*tau**3
		theta_t = np.array(self.theta_i) + np.array(self.theta_f - self.theta_i)*s_tau

		# theta dot time profile
		s_tau_d = 30*tau**4 - 60*tau**3 + 30*tau**2
		theta_dot_t = np.array(self.theta_f - self.theta_i)/T*s_tau_d

		# theta double dot time profile
		s_tau_dd = 120*tau**3 - 180*tau**2 + 60*tau
		theta_ddot_t = np.array(self.theta_f - self.theta_i)/T*s_tau_dd

		# theta triple dot time profile
		s_tau_ddd = -360*tau**2 - 360*tau + 60
		theta_dddot_t = np.array(self.theta_f - self.theta_i)/T*s_tau_ddd

		# checking the forward kinematics 
		ee_pos_t = np.zeros(theta_t.shape)
		for idx, i in enumerate(theta_t.transpose()):
			ee_pos_t[:, idx] = self.robot.forward_kin(theta_t[:, idx])

		return (tau, theta_t, theta_dot_t, theta_ddot_t, theta_dddot_t, ee_pos_t)


	def point_to_point_4567(self):
		FREQUENCY = 100

		# overall time period
		T = abs(35/16*(self.theta_f - self.theta_i)/self.thetadot_max)
		T = [1 for _ in T]
		T = math.floor(max(T)*FREQUENCY)
		tau = np.array(range(0, T))/T


		# theta time profile
		s_tau = -20*tau**7 + 70*tau**6 - 84*tau**5 + 35*tau**4
		theta_t = np.array(self.theta_i) + np.array(self.theta_f - self.theta_i)*s_tau
		

		# theta dot time profile
		s_tau_d = -140*tau**6 + 420*tau**5 - 420*tau**4 + 140*tau**3
		theta_dot_t = np.array(self.theta_f - self.theta_i)/T*s_tau_d

		# theta double dot time profile
		s_tau_dd = -840*tau**5 + 2100*tau**4 - 1680*tau**3 + 420*tau**2
		theta_ddot_t = np.array(self.theta_f - self.theta_i)/T*s_tau_dd

		# theta triple dot time profile
		s_tau_ddd = -4200*tau**4 + 8400*tau**3 - 5040*tau**2 + 840*tau**1
		theta_dddot_t = np.array(self.theta_f - self.theta_i)/T*s_tau_ddd

		# checking the forward kinematics 
		ee_pos_t = np.zeros(theta_t.shape)
		# print(theta_t.shape)
		for idx, i in enumerate(theta_t.transpose()):
			ee_pos_t[:, idx] = self.robot.forward_kin(theta_t[:, idx])

		return (tau*T, theta_t, theta_dot_t, theta_ddot_t, theta_dddot_t, ee_pos_t)


	def trapezoidal_ptp(self, k=1/3): 
		FREQUENCY = 1000
		theta_i = max(self.theta_i)
		theta_f = max(self.theta_f)
		thetadot_max = np.array(self.thetadot_max)

		# overall time period 
		T = int((theta_f - theta_i)/(thetadot_max*(1-k))*FREQUENCY)
		T = [1 for _ in T]*FREQUENCY
		t = np.array(range(0, T))

		# acceleration duration 
		T_a = int(T*k)

		# acceleration 
		a = thetadot_max/(T_a)

		# theta time profile 
		theta_t = np.zeros(t.shape)

		theta_t[0:T_a+1] 		= a/2*t[0: T_a+1]**2
		theta_t[T_a+1: -T_a-1] 	= a*T_a**2/2 + thetadot_max*(t[T_a+1: -T_a-1] - T_a)
		theta_t[-T_a-1:]		= a*T_a**2/2 + thetadot_max*(T - 2*T_a) + a/2*(T - T_a)**2 - a*T*(T - T_a) + a*T*t[-T_a-1:] - a*t[-T_a-1:]**2/2

		theta_t = theta_t/max(theta_t)*(theta_f - theta_i) + theta_i 

		# theta dot time profile 
		thetadot_t = np.zeros(t.shape)

		thetadot_t[0:T_a+1] 		= a*t[0:T_a+1]
		thetadot_t[T_a+1: -T_a-1] 	= thetadot_max
		thetadot_t[-T_a-1:]			= a*(T - t[-T_a-1:])

		# theta double dot time profile 
		thetadotdot_t = np.zeros(t.shape)

		thetadotdot_t[0:T_a+1] 		= a
		thetadotdot_t[T_a+1: -T_a-1] 	= 0
		thetadotdot_t[-T_a-1:]			= -a


# =================================================================================================
# -- main -----------------------------------------------------------------------------------------
# =================================================================================================

if __name__ == "__main__":
	pass 

	# TEST 1 

	# defining robot 
	# delta_robot = DeltaRobot(0.2, 0.46, 0.1, 0.074)

	# defining path planner
	# path_planner = PathPlannerPTP(delta_robot, [0.05, 0.05, -0.31], [0, -0.15, -0.42], 20)

	# using 345 planner 
	# results_345 = path_planner.point_to_point_345()
	# path_planner.plot_results(results_345)

	# using 4567 planner
	# results_4567 = path_planner.point_to_point_4567()
	# path_planner.plot_results(results_4567)

	# trapezoidal method
	# path_planner.trapezoidal_ptp()



	# TEST 2
	# delta_robot = DeltaRobot(0.2, 0.46, 0.1, 0.074)

	# # defining path planner
	# path_planner = PathPlannerPTP(delta_robot, [0.00, -0.05, -0.42], [0.00, -0.05, -0.31], 20)
	# # using 4567 planner
	# results_4567_1 = path_planner.point_to_point_4567()

	# # defining path planner
	# path_planner = PathPlannerPTP(delta_robot, [0.00, -0.05, -0.31], [0.00, 0.00, -0.31], 20)
	# # using 4567 planner
	# results_4567_2 = path_planner.point_to_point_4567()

	# # defining path planner
	# path_planner = PathPlannerPTP(delta_robot, [0.00, 0.00, -0.31], [0.00, 0.00, -0.42], 20)
	# # using 4567 planner
	# results_4567_3 = path_planner.point_to_point_4567()
