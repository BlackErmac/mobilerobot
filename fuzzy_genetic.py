from car import Car
from controllers import FIS
from utils import *
import numpy as np
import matplotlib.pyplot as plt
import random
# from controllers import GenAlgo
from geneticalgorithm import geneticalgorithm as ga
import pdb


car_fuzzy = Car(51, 50)
flag_burned = True
# road list
way_points = []
show = False
controller_fuzzy_linear_v = FIS(show=show , actuator_range =10)
controller_fuzzy_angular_v = FIS(show=show , actuator_range =5)
W, H = 800, 800


for x in range(120, 130, 2):
	rand_num = random.random()
	if rand_num < .5: rand_num = 1
	else: rand_num = 0
	y = 50*(np.sin(2*np.pi*0.25*(x + 200)/100)+np.sin(2*np.pi*0.5*(x + 200)/100)) + H/2 - 50 + rand_num*0
	way_points.append([x, int(y)])


fuzzy_car_points = []

current_idx_fuzzy = 0
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
## linear v control simulator
controller_fuzzy_angular_v.create_mf()
rules_a = controller_fuzzy_angular_v.create_rules()
simulator_a = controller_fuzzy_angular_v.fuzzy_control(rules=rules_a)


# pdb.set_trace()

def main(membership_tops):
    current_idx_fuzzy = 0
    linear_v = 0
    angular_v = 0

    controller_fuzzy_linear_v.error.__dict__['universe'][1] = membership_tops[0]
    controller_fuzzy_linear_v.error.__dict__['universe'][3] = membership_tops[1]
    controller_fuzzy_linear_v.delta.__dict__['universe'][1] = membership_tops[2]
    controller_fuzzy_linear_v.delta.__dict__['universe'][3] = membership_tops[3]
    #gen_algo_v.call_FIS()
    for _ in way_points:
        # FIS car
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

    return(sum(MSE_fuzzy_v)/len(way_points))


varbound=np.array([[-7.5,-2.5],[2.5,7.5]]*2)
model=ga(function=main,dimension=4,variable_type='real',variable_boundaries=varbound)
model.run()



