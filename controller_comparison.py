from car import Car
from draw import Draw
from controllers import PID, MPC,FIS , PIDF
import cv2
from utils import *
import numpy as np
import matplotlib.pyplot as plt
import random
import pdb
# road list
way_points = []
from fuzzypid import FISPID
# Windows Dimensions
W, H = 800, 800

# call Draw class from class for making windows
draw = Draw(W, H, window_name = "AI_pro1")

# defination of the cars Car(x_initiate_pos , y_initiate_pos)
car_pid   = Car(50, 50)
car_fuzzy = Car(51, 50)
car_mpc   = Car(52, 50)
car_fuzzypid = Car(53,50)

# defination of the controllers

## defination of the 2 pid controller for control of the linear & angular parameters
### PID -> linear_velocity_control, angular_velocity_control
controller_pid = PID(kp_linear = 0.5, kd_linear = 0.1, ki_linear = 0,
						kp_angular = 3, kd_angular = 0.1, ki_angular = 0 , linear_output_threshold= 5)

## defination of the 2 fuzzy controller for control of the linear & angular parmeters
### FIS (show : bool [for indicating plots])-> linear_velocity_control, angular_velocity_control
show = False
controller_fuzzy_linear_v = FIS(show=show , actuator_range =10)
controller_fuzzy_angular_v = FIS(show=show , actuator_range =5)

## defination of the one MPC controller for control of the velocity and angular
### MPC (horizon : int)-> linear_velocity_control, angular_velocity_control
### TODO
horizon = 5
controller_mpc = MPC(horizon = horizon)


## defination of the two fuzzy pid contorller for control of the velocity and angular 
### TODO
controller_fuzzypid = FISPID(show=show)
controller_fuzzypid_a = FISPID(show=show)

ADD_NOISE = False
#create the road
for x in range(120, 750, 2):
	rand_num = random.random()
	if rand_num < .5 and ADD_NOISE: rand_num = 1
	else: rand_num = 0
	y = 50*(np.sin(2*np.pi*0.25*(x + 200)/100)+np.sin(2*np.pi*0.5*(x + 200)/100)) + H/2 - 50 + rand_num*10
	way_points.append([x, int(y)])

# car points of the cars
mpc_car_points = []
pid_car_points = []
fuzzy_car_points = []
fuzzypid_car_points = []


# first initializing the code
flag_burned = True
current_idx_mpc = 0
current_idx_pid = 0
current_idx_fuzzy = 0
current_idx_fuzzypid = 0
lw = 0
rw = 0


linear_v = 0
angular_v = 0

# initializing fuzzy controller
flag_show_fuzzy_error = False
error_fuzzy_v = []
error_fuzzy_a = []
MSE_fuzzy_v = []
MSE_fuzzy_a = []
## linear v control simulator
controller_fuzzy_linear_v.create_mf()
rules_v = controller_fuzzy_linear_v.create_rules()
simulator_v = controller_fuzzy_linear_v.fuzzy_control(rules=rules_v)
## linear a control simulator
controller_fuzzy_angular_v.create_mf()
rules_a = controller_fuzzy_angular_v.create_rules()
simulator_a = controller_fuzzy_angular_v.fuzzy_control(rules=rules_a)
####################################################################
#fuzzy genetice algorithm
# controller_fuzzy_linear_v.error.__dict__['universe'][1] = -4.69358743 
# controller_fuzzy_linear_v.error.__dict__['universe'][3] =  6.03105489
# controller_fuzzy_linear_v.delta.__dict__['universe'][1] = -3.22792746
# controller_fuzzy_linear_v.delta.__dict__['universe'][3] =  7.15173572

# print(controller_fuzzy_linear_v.error.__dict__['universe'])
#pdb.set_trace()


# initializing fuzzy-pid controller
flag_show_fuzzypid_error = False
error_fuzzypid_v = []
error_fuzzypid_a = []
MSE_fuzzypid_v = []
MSE_fuzzypid_a = []
Kp_v_fuzzypid = []
Kd_v_fuzzypid = []
Ki_v_fuzzypid = []
Kp_a_fuzzypid = []
Kd_a_fuzzypid = []
Ki_a_fuzzypid = []

## linear v control simulator
controller_fuzzypid.create_mf()
rules_v_fuzzypid = controller_fuzzypid.create_rules()
simulator_v_fuzzypid = controller_fuzzypid.fuzzy_control(rules=rules_v_fuzzypid)

controller_fuzzypid_a.create_mf()
rules_a_fuzzypid = controller_fuzzypid_a.create_rules()
simulator_a_fuzzypid = controller_fuzzypid_a.fuzzy_control(rules = rules_a_fuzzypid)
# ## linear v control simulator
# controller_fuzzy_angular_v.create_mf()
# rules_a = controller_fuzzy_angular_v.create_rules()
# simulator_a = controller_fuzzy_angular_v.fuzzy_control(rules=rules_a)

# pdb.set_trace()

# initializing pid controller
flag_show_pid_error = False
error_pid_v = []
error_pid_a = []
MSE_pid_v = []
MSE_pid_a = []
flag_show = True

# main loop of the code
while True:
	draw.clear()
	if len(way_points)>0:
		draw.draw_path(way_points, color = (255, 0, 0), thickness = 1)

	if len(mpc_car_points)>0:
		draw.draw_path(mpc_car_points, color = (0, 255, 0), thickness = 1, dotted = True)

	if len(pid_car_points)>0:
		draw.draw_path(pid_car_points, color = (0, 0, 255), thickness = 1, dotted = True)

	if len(fuzzy_car_points)>0:
		draw.draw_path(fuzzy_car_points, color = (100, 100,100), thickness = 1, dotted = True)

	if len(fuzzypid_car_points)>0:
		draw.draw_path(fuzzypid_car_points, color = (255, 165,0), thickness = 1, dotted = True)
	
	
	
	draw.draw(car_mpc.get_points(), color = (0, 255, 0), thickness = 1)
	draw.draw(car_pid.get_points(), color = (0, 0, 255), thickness = 1)
	draw.draw(car_fuzzy.get_points() , color= (100,100,100) , thickness=1)
	draw.draw(car_fuzzypid.get_points() , color= (255, 165,0) , thickness=1)

	draw.add_text("PID Controller", color = (0, 0, 255), fontScale = 0.5, thickness = 1, org = (100, 50))
	draw.add_text("MPC Controller", color = (0, 255, 0), fontScale = 0.5, thickness = 1, org = (100, 75))
	draw.add_text("Fuzzy Controller", color=(100,100,100) , fontScale=0.5 , thickness=1, org = (100,100) )
	draw.add_text("Fuzzypid Controller", color=(255, 165,0) , fontScale=0.5 , thickness=1, org = (100,125) )
	draw.add_text("Trajectory", color = (255, 0, 0), fontScale = 0.5, thickness = 1, org = (100, 150))
	

	k = draw.show()

	# MPC Car
	# x, _ = car_mpc.get_state()
	# if len(way_points)>0 and current_idx_mpc != len(way_points):
	# 	mpc_car_points.append([int(x[0, 0]), int(x[1, 0])])
	# 	goal_pt = way_points[current_idx_mpc]
	# 	linear_v, angular_v = controller_mpc.optimize(car = car_mpc, points = way_points[current_idx_mpc:current_idx_mpc+horizon])
	# 	dist = get_distance(x[0, 0], x[1, 0], goal_pt[0], goal_pt[1])
	# 	if dist<10:
	# 		current_idx_mpc+= 1
	# else:
	# 	linear_v = 0
	# 	angular_v = 0
	# car_mpc.set_robot_velocity(linear_v, angular_v)
	# car_mpc.update(0.5)

	########################################################## FIS car ########################################################
	x , _ = car_fuzzy.get_state()
	if len(way_points) > 0 and current_idx_fuzzy != len(way_points):
		fuzzy_car_points.append([int(x[0,0]),int(x[1,0])])
		goal_pt = way_points[current_idx_fuzzy]

		#preapare positino error
		error_position = get_distance(x[0,0],x[1,0],goal_pt[0],goal_pt[1])
		MSE_fuzzy_v.append(error_position**2)
		if len(error_fuzzy_v) == 0:
			error_fuzzy_v.append(error_position)
			error_fuzzy_v.append(error_position)
		else : error_fuzzy_v.append(error_position)
		delta_position = error_fuzzy_v[-1] - error_fuzzy_v[-2]
		#if delta_position == 0: delta_position = .36

		#prepare angle error
		body_to_goal = get_angle(x[0,0],x[1,0],goal_pt[0],goal_pt[1])
		error_angle = (-body_to_goal) - x[2,0]
		MSE_fuzzy_a.append(error_angle**2)
		if len(error_fuzzy_a) == 0:
			error_fuzzy_a.append(error_angle)
			error_fuzzy_a.append(error_angle)
		else: error_fuzzy_a.append(error_angle)
		delta_angle = error_fuzzy_a[-1] - error_fuzzy_a[-2]
		#if delta_angle == .2 : delta_angle = -6.23


		#linear v control simulator
		simulator_v.input['error'] = error_position
		simulator_v.input['delta'] = delta_position
		#print("P : " , error_position , delta_position)
		simulator_v.compute()
		linear_v = simulator_v.output['output']

		#angular control simulator
		simulator_a.input['error'] = error_angle
		simulator_a.input['delta'] = delta_angle
		#print('A : ',error_angle , delta_angle)
		simulator_a.compute()
		angular_v = simulator_a.output['output']

		dist = get_distance(x[0, 0], x[1, 0], goal_pt[0], goal_pt[1])
		if dist < 10:
			current_idx_fuzzy += 1
	else:
		linear_v = 0
		angular_v = 0	
	
	car_fuzzy.set_robot_velocity(linear_v,angular_v)
	car_fuzzy.update(0.5)
	if current_idx_fuzzy == len(way_points) and flag_burned:
		flag_show_fuzzy_error = True



	########################################################## PID Car ################################################
	x, _ = car_pid.get_state()
	if len(way_points)>0 and current_idx_pid != len(way_points):
		pid_car_points.append([int(x[0, 0]), int(x[1, 0])])
		goal_pt = way_points[current_idx_pid]
		linear_v, angular_v = controller_pid.get_control_inputs(x, goal_pt, car_pid.get_points()[2], current_idx_pid)
		#print(linear_v , angular_v)
		dist = get_distance(x[0, 0], x[1, 0], goal_pt[0], goal_pt[1])
		body_to_goal = get_angle(x[0,0],x[1,0],goal_pt[0],goal_pt[1])
		error_angle = (-body_to_goal) - x[2,0]
		error_pid_v.append(dist)
		error_pid_a.append(error_angle)
		MSE_pid_v.append(dist**2)
		MSE_pid_a.append(error_angle**2)
		if dist<10:
			current_idx_pid+= 1

		flag_show = True
	else:
		linear_v = 0
		angular_v = 0
	car_pid.set_robot_velocity(linear_v, angular_v)
	car_pid.update(0.5)
	if current_idx_pid == len(way_points):
		flag_show_pid_error = True

	############################################ FISPID Car ##########################################################
	x, _ = car_fuzzypid.get_state()
	if len(way_points)>0 and current_idx_fuzzypid != len(way_points):
		fuzzypid_car_points.append([int(x[0, 0]), int(x[1, 0])])
		goal_pt = way_points[current_idx_fuzzypid]
		dist = get_distance(x[0, 0], x[1, 0], goal_pt[0], goal_pt[1])
		body_to_goal = get_angle(x[0,0],x[1,0],goal_pt[0],goal_pt[1])
		error_angle = (-body_to_goal) - x[2,0]
		error_fuzzypid_v.append(dist)
		error_fuzzypid_a.append(error_angle)
		try:
			delta_error = error_fuzzypid_v[-1] - error_fuzzypid_v[-2]
		except IndexError:
			delta_error = 0


		try:
			delta_error_angle = error_fuzzypid_a[-1] - error_fuzzypid_a[-2]
		except IndexError:
			delta_error_angle = 0

		#print(dist/1000., delta_error)
		#pdb.set_trace()
		simulator_v_fuzzypid.input['error'] = dist/50.
		simulator_v_fuzzypid.input['delta'] = delta_error
		simulator_v_fuzzypid.compute()
		kp = simulator_v_fuzzypid.output['output_kp']
		ki = simulator_v_fuzzypid.output['output_ki']
		kd = simulator_v_fuzzypid.output['output_kd']

		Kp_v_fuzzypid.append(kp)
		Kd_v_fuzzypid.append(kd)
		Ki_v_fuzzypid.append(ki)


		simulator_a_fuzzypid.input['error'] = error_angle
		simulator_a_fuzzypid.input['delta'] = delta_error_angle
		simulator_a_fuzzypid.compute()
		kp_a = simulator_a_fuzzypid.output['output_kp']
		ki_a = simulator_a_fuzzypid.output['output_ki']
		kd_a = simulator_a_fuzzypid.output['output_kd']

		Kp_a_fuzzypid.append(kp_a)
		Kd_a_fuzzypid.append(kd_a)
		Ki_a_fuzzypid.append(ki_a)


		#print(kp,ki,kd)

		controller_pidtofuzzy = PIDF(kp_linear = kp, kd_linear = kd, ki_linear = ki,
						kp_angular = kp_a, kd_angular = kd_a, ki_angular = ki_a , linear_output_threshold= 5)
		linear_fuzzypid_v, angular_fuzzypid_v = controller_pidtofuzzy.get_control_inputs(x, goal_pt, car_fuzzypid.get_points()[2], current_idx_fuzzypid)
		#print(linear_v , angular_v)
		
		error_fuzzypid_v.append(dist)
		error_fuzzypid_a.append(error_angle)
		MSE_fuzzypid_v.append(dist**2)
		MSE_fuzzypid_a.append(error_angle**2)
		if dist<10:
			current_idx_fuzzypid+= 1

		flag_show = True
	else:
		linear_fuzzypid_v = 0
		angular_fuzzypid_v = 0
	car_fuzzypid.set_robot_velocity(linear_fuzzypid_v, angular_fuzzypid_v)
	car_fuzzypid.update(0.5)
	if current_idx_fuzzypid == len(way_points):
		flag_show_fuzzypid_error = True

	if k == ord("q"):
		break

	

	#print(flag_show_fuzzy_error , flag_show_pid_error , flag_burned)
	if flag_show_fuzzy_error and flag_show_pid_error and flag_burned:
			
			draw.add_text(f"fuzzy MSE position: {sum(MSE_fuzzy_v)**.5/len(MSE_fuzzy_v):.4f} , finish in {len(MSE_fuzzy_v)} frame", color = (255, 0, 0), fontScale = 0.5, thickness = 1, org = (300, 100))
			draw.add_text(f"fuzzy MSE angular: {sum(MSE_fuzzy_a)**.5/len(MSE_fuzzy_a):.4f}", color = (255, 0, 0), fontScale = 0.5, thickness = 1, org = (300, 125))
			draw.add_text(f"pid MSE position: {sum(MSE_pid_v)**.5/len(MSE_pid_v):.4f} , finish in {len(MSE_pid_v)} frame", color = (255, 0, 0), fontScale = 0.5, thickness = 1, org = (300, 150))
			draw.add_text(f"pid MSE angular: {sum(MSE_pid_a)**.5/len(MSE_pid_a):.4f}", color = (255, 0, 0), fontScale = 0.5, thickness = 1, org = (300, 175))
			draw.add_text(f"pidfuzzy MSE position: {sum(MSE_fuzzypid_v)**.5/len(MSE_fuzzypid_v):.4f} , finish in {len(MSE_fuzzypid_v)} frame", color = (255, 0, 0), fontScale = 0.5, thickness = 1, org = (300, 200))
			draw.add_text(f"pidfuzzy MSE angular: {sum(MSE_fuzzypid_a)**.5/len(MSE_fuzzypid_a):.4f}", color = (255, 0, 0), fontScale = 0.5, thickness = 1, org = (300, 225))
			
			draw.show()

			fig_FIS, axs_FIS = plt.subplots(2)
			fig_FIS.suptitle('errors of the fuzzy controller')
			axs_FIS[0].set_title('position error')
			axs_FIS[1].set_title('angular error')
			axs_FIS[0].grid(color = 'blue', linestyle = '--', linewidth = 0.5)
			axs_FIS[1].grid(color = 'blue', linestyle = '--', linewidth = 0.5)
			axs_FIS[0].plot(error_fuzzy_v , 'tab:orange')
			axs_FIS[1].plot(error_fuzzy_a , 'tab:green')
			fig_FIS.tight_layout()

			fig_PID , axs_PID = plt.subplots(2)
			fig_PID.suptitle('errors of the pid controller')
			axs_PID[0].set_title('position error')
			axs_PID[1].set_title('angular error')
			axs_PID[0].grid(color = 'blue', linestyle = '--', linewidth = 0.5)
			axs_PID[1].grid(color = 'blue', linestyle = '--', linewidth = 0.5)
			axs_PID[0].plot(error_pid_v , 'tab:orange')
			axs_PID[1].plot(error_pid_a , 'tab:green')
			fig_PID.tight_layout()
			
			fig_FPID , axs_FPID = plt.subplots(2)
			fig_FPID.suptitle('errors of the fuzzy pid controller')
			axs_FPID[0].set_title('position error')
			axs_FPID[1].set_title('angular error')
			axs_FPID[0].grid(color = 'blue', linestyle = '--', linewidth = 0.5)
			axs_FPID[1].grid(color = 'blue', linestyle = '--', linewidth = 0.5)
			axs_FPID[0].plot(error_fuzzypid_v , 'tab:orange')
			axs_FPID[1].plot(error_fuzzypid_a , 'tab:green')

			fig_FPIDK , axs_FPIDK = plt.subplots(2)
			axs_FPIDK[0].set_title('pid gains in positino controller')
			axs_FPIDK[1].set_title('pid gains in angular controller')
			axs_FPIDK[0].grid(color = 'blue', linestyle = '--', linewidth = 0.5)
			axs_FPIDK[1].grid(color = 'blue', linestyle = '--', linewidth = 0.5)
			axs_FPIDK[0].plot(Kp_v_fuzzypid , 'tab:red' , label='kp')
			axs_FPIDK[0].plot(Kd_v_fuzzypid , 'tab:blue' , label='kd')
			axs_FPIDK[0].plot(Ki_v_fuzzypid , 'tab:pink' , label='ki')
			axs_FPIDK[1].plot(Kp_a_fuzzypid , 'tab:red' , label='kp')
			axs_FPIDK[1].plot(Kd_a_fuzzypid , 'tab:blue' , label='kd')
			axs_FPIDK[1].plot(Ki_a_fuzzypid , 'tab:pink' , label='ki')
			plt.legend(loc = 'best')
			fig_FPIDK.tight_layout()

			# fig_com , axs_com = plt.subplots(1)
			# fig_com.suptitle("Differences in controller errors")
			# axs_com.plot(np.array(error_fuzzy_v) - np.array(error_pid_v[:len(error_fuzzy_v)]))
			# axs_com.grid()
			plt.show()
			flag_burned = False


