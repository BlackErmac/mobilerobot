import numpy as np
from utils import *
from copy import deepcopy
from scipy.optimize import minimize
import skfuzzy.control as ctrl
import matplotlib.pyplot as plt
import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt
from geneticalgorithm import geneticalgorithm as ga



class FIS:
	def __init__(self,show:bool,actuator_range=10):
		self.show = show
		self.universe = np.linspace(-actuator_range, actuator_range, 5)
		# Create the three fuzzy variables - two inputs, one output
		self.names_error = ['nb', 'ns', 'ze', 'ps', 'pb']
		self.names_delta = ['nb', 'ns', 'ze', 'ps', 'pb']
		self.names_output = ['nb', 'ns', 'ze', 'ps', 'pb']
		


	def create_mf(self):

		#create the three fuzzy variables 2 inputs and 1 output [Antecedent for input and consequent for output]
		self.error = ctrl.Antecedent(self.universe, 'error')
		self.delta = ctrl.Antecedent(self.universe, 'delta')
		self.output = ctrl.Consequent(self.universe, 'output')

		#populate the fuzzy variables with terms
		self.error.automf(names=self.names_error)
		self.delta.automf(names=self.names_delta)
		self.output.automf(names=self.names_output)

	def create_rules(self) -> list:
		# create the rules 
		rule0 = ctrl.Rule(antecedent=((self.error['nb'] & self.delta['nb']) |
                              (self.error['ns'] & self.delta['nb']) |
                              (self.error['nb'] & self.delta['ns'])),
                  consequent=self.output['nb'], label='rule nb')

		rule1 = ctrl.Rule(antecedent=((self.error['nb'] & self.delta['ze']) |
									(self.error['nb'] & self.delta['ps']) |
									(self.error['ns'] & self.delta['ns']) |
									(self.error['ns'] & self.delta['ze']) |
									(self.error['ze'] & self.delta['ns']) |
									(self.error['ze'] & self.delta['nb']) |
									(self.error['ps'] & self.delta['nb'])),
						consequent=self.output['ns'], label='rule ns')

		rule2 = ctrl.Rule(antecedent=((self.error['nb'] & self.delta['pb']) |
									(self.error['ns'] & self.delta['ps']) |
									(self.error['ze'] & self.delta['ze']) |
									(self.error['ps'] & self.delta['ns']) |
									(self.error['pb'] & self.delta['nb'])),
						consequent=self.output['ze'], label='rule ze')

		rule3 = ctrl.Rule(antecedent=((self.error['ns'] & self.delta['pb']) |
									(self.error['ze'] & self.delta['pb']) |
									(self.error['ze'] & self.delta['ps']) |
									(self.error['ps'] & self.delta['ps']) |
									(self.error['ps'] & self.delta['ze']) |
									(self.error['pb'] & self.delta['ze']) |
									(self.error['pb'] & self.delta['ns'])),
						consequent=self.output['ps'], label='rule ps')

		rule4 = ctrl.Rule(antecedent=((self.error['ps'] & self.delta['pb']) |
									(self.error['pb'] & self.delta['pb']) |
									(self.error['pb'] & self.delta['ps'])),
						consequent=self.output['pb'], label='rule pb')

		return [rule0 , rule1 , rule2 , rule3 , rule4]
	
	def fuzzy_control(self , rules , flush_after_run = 21 * 21):
		system = ctrl.ControlSystem(rules = rules)
		sim = ctrl.ControlSystemSimulation(system, flush_after_run=flush_after_run + 1)
		if self.show == True:
			self.show_plot(sim)
			self.show = False

		return sim
			

	def show_plot(self , sim):
		self.error.view()
		self.delta.view()
		self.output.view()

		# how the control surface
		# higher resolution could make full accuracy
		upsampled = np.linspace(-2, 2, 21)
		x, y = np.meshgrid(upsampled, upsampled)
		z = np.zeros_like(x)

		for i in range(21):
			for j in range(21):
				sim.input['error'] = x[i, j]
				sim.input['delta'] = y[i, j]
				sim.compute()
				z[i, j] = sim.output['output']

		#plot the result in 3D mode with 2D trajection
		fig = plt.figure(figsize=(8, 8))
		ax = fig.add_subplot(111, projection='3d')

		surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap='viridis',
					linewidth=0.4, antialiased=True)

		cset = ax.contourf(x, y, z, zdir='z', offset=-2.5, cmap='viridis', alpha=0.5)
		cset = ax.contourf(x, y, z, zdir='x', offset=3, cmap='viridis', alpha=0.5)
		cset = ax.contourf(x, y, z, zdir='y', offset=3, cmap='viridis', alpha=0.5)

		ax.view_init(30, 200)
		plt.show()
		
	def change_names(self):
		pass


class PID:
	#initializing first parameters of the controller and error tanks
	def __init__(self, 
					kp_linear = 0.1, kd_linear = 0.1, ki_linear = 0, 
					kp_angular = 0.1, kd_angular = 0.1, ki_angular = 0 , linear_output_threshold=5):

		self.kp_linear = kp_linear
		self.kd_linear = kd_linear
		self.ki_linear = ki_linear

		self.kp_angular = kp_angular
		self.kd_angular = kd_angular
		self.ki_angular = ki_angular

		self.prev_error_position = 0
		self.prev_error_angle = 0

		self.prev_body_to_goal = 0
		self.prev_waypoint_idx = -1
		self.linear_output_threshold = linear_output_threshold
	# core of the pid controller
	def get_control_inputs(self, x, goal_x, nose, waypoint_idx):

		error_position = get_distance(x[0, 0], x[1, 0], goal_x[0], goal_x[1])
		body_to_goal = get_angle(x[0, 0], x[1, 0], goal_x[0], goal_x[1])
		#body_to_nose = get_angle(x[0, 0], x[1, 0], nose[0], nose[1])
		error_angle = (-body_to_goal) - x[2, 0]


		linear_velocity_control = self.kp_linear*error_position + self.kd_linear*(error_position - self.prev_error_position)
		angular_velocity_control = self.kp_angular*error_angle + self.kd_angular*(error_angle - self.prev_error_angle)

		self.prev_error_angle = error_angle
		self.prev_error_position = error_position

		self.prev_waypoint_idx = waypoint_idx
		self.prev_body_to_goal = body_to_goal

		if linear_velocity_control> self.linear_output_threshold:
			linear_velocity_control = self.linear_output_threshold

		return linear_velocity_control, angular_velocity_control



class PIDF:
	# initializing first parameters of the controller and error tanks
	# def __init__(self, 
	# 				kp_linear = 0.1, kd_linear = 0.1, ki_linear = 0, 
	# 				kp_angular = 0.1, kd_angular = 0.1, ki_angular = 0 , linear_output_threshold=5):
	def __init__(self, 
					kp_linear, kd_linear , ki_linear , 
					kp_angular , kd_angular , ki_angular  , linear_output_threshold=5):
		print(kp_linear , kp_angular)
		self.kp_linear = kp_linear
		self.kd_linear = kd_linear
		self.ki_linear = ki_linear

		self.kp_angular = kp_angular
		self.kd_angular = kd_angular
		self.ki_angular = ki_angular

		self.prev_error_position = 0
		self.prev_error_angle = 0

		self.prev_body_to_goal = 0
		self.prev_waypoint_idx = -1
		self.linear_output_threshold = linear_output_threshold
	# core of the pid controller
	def get_control_inputs(self, x, goal_x, nose, waypoint_idx):

		error_position = get_distance(x[0, 0], x[1, 0], goal_x[0], goal_x[1])
		body_to_goal = get_angle(x[0, 0], x[1, 0], goal_x[0], goal_x[1])
		#body_to_nose = get_angle(x[0, 0], x[1, 0], nose[0], nose[1])
		error_angle = (-body_to_goal) - x[2, 0]


		linear_velocity_control = self.kp_linear*error_position + self.kd_linear*(error_position - self.prev_error_position)
		angular_velocity_control = self.kp_angular*error_angle + self.kd_angular*(error_angle - self.prev_error_angle)

		self.prev_error_angle = error_angle
		self.prev_error_position = error_position

		self.prev_waypoint_idx = waypoint_idx
		self.prev_body_to_goal = body_to_goal

		if linear_velocity_control> self.linear_output_threshold:
			linear_velocity_control = self.linear_output_threshold

		return linear_velocity_control, angular_velocity_control



class MPC:
	def __init__(self, horizon):
		self.horizon = horizon
		self.R = np.diag([0.01, 0.01])                 # input cost matrix
		self.Rd = np.diag([0.01, 1.0])                 # input difference cost matrix
		self.Q = np.diag([1.0, 1.0])                   # state cost matrix
		self.Qf = self.Q							   # state final matrix

	def cost(self, u_k, car, path):
		path = np.array(path)
		controller_car = deepcopy(car)
		u_k = u_k.reshape(self.horizon, 2).T
		z_k = np.zeros((2, self.horizon+1))

		desired_state = path.T

		cost = 0.0

		for i in range(self.horizon):
			controller_car.set_robot_velocity(u_k[0,i], u_k[1,i])
			controller_car.update(0.5)
			x, _ = controller_car.get_state()
			z_k[:,i] = [x[0, 0], x[1, 0]]
			cost += np.sum(self.R@(u_k[:,i]**2))
			cost += np.sum(self.Q@((desired_state[:,i]-z_k[:,i])**2))
			if i < (self.horizon-1):     
				cost += np.sum(self.Rd@((u_k[:,i+1] - u_k[:,i])**2))

		return cost

	def optimize(self, car, points):
		self.horizon = len(points)
		bnd = [(0, 5),(np.deg2rad(-60), np.deg2rad(60))]*self.horizon
		result = minimize(self.cost, args=(car, points), x0 = np.zeros((2*self.horizon)), method='SLSQP', bounds = bnd)
		return result.x[0],  result.x[1]







# class GenAlgo(FIS):
# 	def __init__(self , FIS_model):
# 		self.FIS = FIS_model
# 		self.update_FIS = 
		
# 	def gen_run(self):
# 		varbound=np.array([[-7.5,-2.5],[2.5,7.5]]*2)
# 		model=ga(function=self.update_FIS,dimension=4,variable_type='real',variable_boundaries=varbound)
# 		model.run()








# 		self.x_error = np.arange(-10, 10.1,2.5 )
# 		self.x_error_dot = np.arange(-10, 10.1,2.5)
# 		self.x_output  = np.arange(-10, 10.1,2.5)

# 		# Generate fuzzy membership functions
# 		self.error_nb = fuzz.trimf(self.x_error, [-10, -10, -5])
# 		self.error_n = fuzz.trimf(self.x_error, [-10, -5, 0])
# 		self.error_z = fuzz.trimf(self.x_error, [-5, 0, 5])
# 		self.error_p = fuzz.trimf(self.x_error, [0, 5, 10])
# 		self.error_pb = fuzz.trimf(self.x_error, [5, 10, 10])

# 		self.error_dot_nb = fuzz.trimf(self.x_error_dot, [-10, -10, -5])
# 		self.error_dot_n = fuzz.trimf(self.x_error_dot, [-10, -5, 0])
# 		self.error_dot_z = fuzz.trimf(self.x_error_dot, [-5, 0, 5])
# 		self.error_dot_p = fuzz.trimf(self.x_error_dot, [0, 5, 10])
# 		self.error_dot_pb = fuzz.trimf(self.x_error_dot, [5, 10, 10])

# 		self.output_dot_nb = fuzz.trimf(self.x_output, [-10, -10, -5])
# 		self.output_dot_n = fuzz.trimf(self.x_output, [-10, -7.5, -5])
# 		self.output_dot_z = fuzz.trimf(self.x_output, [-5, 0, 5])
# 		self.output_dot_p = fuzz.trimf(self.x_output, [0, 5, 10])
# 		self.output_dot_pb = fuzz.trimf(self.x_output, [5, 10, 10])

# 		fig, (ax0, ax1, ax2) = plt.subplots(nrows=3, figsize=(8, 9))
# 		ax0.plot(self.x_error, self.error_nb, 'b', linewidth=1.5, label='negative big')
# 		ax0.plot(self.x_error, self.error_n, 'g', linewidth=1.5, label='negative')
# 		ax0.plot(self.x_error, self.error_z, 'r', linewidth=1.5, label='zero')
# 		ax0.plot(self.x_error, self.error_p, 'k', linewidth=1.5, label='positve')
# 		ax0.plot(self.x_error, self.error_pb, 'y', linewidth=1.5, label='positive big')
# 		ax0.set_title('Error')
# 		ax0.legend()

# 		ax1.plot(self.x_error_dot, self.error_dot_nb, 'b', linewidth=1.5, label='negative big')
# 		ax1.plot(self.x_error_dot, self.error_dot_n, 'g', linewidth=1.5, label='negative')
# 		ax1.plot(self.x_error_dot, self.error_dot_z, 'r', linewidth=1.5, label='zero')
# 		ax1.plot(self.x_error_dot, self.error_dot_p, 'k', linewidth=1.5, label='positve')
# 		ax1.plot(self.x_error_dot, self.error_dot_pb, 'y', linewidth=1.5, label='positive big')
# 		ax1.set_title('Error Dot')
# 		ax1.legend()
# 		plt.tight_layout()

# 		plt.show()

# 		# ax1.plot(self.x_error_dot, self.error_dot_nb, 'b', linewidth=1.5, label='negative big')
# 		# ax1.plot(self.x_error_dot, self.error_dot_n, 'g', linewidth=1.5, label='negative')
# 		# ax1.plot(self.x_error_dot, self.error_dot_z, 'r', linewidth=1.5, label='zero')
# 		# ax1.plot(self.x_error_dot, self.error_dot_p, 'k', linewidth=1.5, label='positve')
# 		# ax1.plot(self.x_error_dot, self.error_dot_pb, 'y', linewidth=1.5, label='positive big')
# 		# ax1.set_title('Error Dot')
# 		# ax1.legend()


# if __name__ == "__main__":
# 	ga = GenAlgo()
