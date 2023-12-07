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
from controllers import PID
import pdb

class FISPID:
    def __init__(self,show:bool, actuator_range=10 ):

        self.show = show
        self.universe_in = np.linspace(-actuator_range, actuator_range, 5)
        self.universe_out = np.linspace(0, 1, 5)
        # Create the three fuzzy variables - two inputs, one output
        self.names_error = ['nb', 'ns', 'ze', 'ps', 'pb']
        self.names_delta = ['nb', 'ns', 'ze', 'ps', 'pb']
        self.names_Kp = ['nb', 'ze','pb']
        self.names_Ki = ['nb',  'ze', 'pb']
        self.names_Kd = ['nb', 'ze', 'pb']
        


    def create_mf(self):

        #create the three fuzzy variables 2 inputs and 3 output [Antecedent for input and consequent for output]
        self.error = ctrl.Antecedent(self.universe_in, 'error')
        self.delta = ctrl.Antecedent(self.universe_in, 'delta')
        self.output_kp = ctrl.Consequent(self.universe_out, 'output_kp')
        self.output_ki = ctrl.Consequent(self.universe_out, 'output_ki')
        self.output_kd = ctrl.Consequent(self.universe_out, 'output_kd')

        #populate the fuzzy variables with terms
        self.error.automf(names=self.names_error)
        self.delta.automf(names=self.names_delta)
        self.output_kp.automf(names=self.names_Kp)
        self.output_ki.automf(names=self.names_Ki)
        self.output_kd.automf(names=self.names_Kd)

        #pdb.set_trace()

    def create_rules(self) -> list:
        # create the rules 
        rule0 = ctrl.Rule(antecedent=((self.error['nb'] & self.delta['nb']) |
                                (self.error['ns'] & self.delta['nb']) |
                                (self.error['nb'] & self.delta['ns']) |
                                (self.error['nb'] & self.delta['ze']) |
                                (self.error['ze'] & self.delta['ze'])),
                    consequent=self.output_kp['nb'], label='rule nb kp')



        rule1 = ctrl.Rule(antecedent=((self.error['nb'] & self.delta['pb']) |
                                    (self.error['ns'] & self.delta['ps']) |
                                    (self.error['ze'] & self.delta['ze']) |
                                    (self.error['ps'] & self.delta['ns']) |
                                    (self.error['ze'] & self.delta['ns']) |
                                    (self.error['ns'] & self.delta['ze']) |
                                    (self.error['pb'] & self.delta['nb'])),
                        consequent=self.output_kp['ze'], label='rule ze kp')


        rule2 = ctrl.Rule(antecedent=((self.error['ps'] & self.delta['pb']) |
                                    (self.error['pb'] & self.delta['pb']) |
                                    (self.error['pb'] & self.delta['ze']) |
                                    (self.error['ze'] & self.delta['pb']) |
                                    (self.error['pb'] & self.delta['ps'])),
                        consequent=self.output_kp['pb'], label='rule pb kp')
        
        rule3 = ctrl.Rule(antecedent=((self.error['nb'] & self.delta['nb']) |
                        (self.error['ns'] & self.delta['nb']) |
                        (self.error['nb'] & self.delta['ns'])),
                            consequent=self.output_kd['nb'], label='rule nb kd')

        rule4 = ctrl.Rule(antecedent=((self.error['nb'] & self.delta['pb']) |
                                    (self.error['ns'] & self.delta['ps']) |
                                    (self.error['ze'] & self.delta['ze']) |
                                    (self.error['ps'] & self.delta['ns']) |
                                    (self.error['pb'] & self.delta['nb'])),
                        consequent=self.output_kd['ze'], label='rule ze kd')

        rule5 = ctrl.Rule(antecedent=((self.error['ps'] & self.delta['pb']) |
                                    (self.error['pb'] & self.delta['pb']) |
                                    (self.error['pb'] & self.delta['ps'])),
                        consequent=self.output_kd['pb'], label='rule pb kd')
        
        rule6 = ctrl.Rule(antecedent=((self.error['nb'] & self.delta['nb']) |
                        (self.error['ns'] & self.delta['nb']) |
                        (self.error['ze'] & self.delta['ze']) |
                        (self.error['nb'] & self.delta['ns'])),
            consequent=self.output_ki['nb'], label='rule nb ki')

        rule7 = ctrl.Rule(antecedent=((self.error['nb'] & self.delta['pb']) |
                                      (self.error['ns'] & self.delta['ps']) |                                     
                                      (self.error['ps'] & self.delta['ns']) 
                                      ),
                        consequent=self.output_ki['ze'], label='rule ze ki')

        rule8 = ctrl.Rule(antecedent=((self.error['ps'] & self.delta['pb']) |
                                      (self.error['pb'] & self.delta['pb']) |
                                      (self.error['pb'] & self.delta['nb']) |
                                      (self.error['pb'] & self.delta['ps'])),
                        consequent=self.output_ki['pb'], label='rule pb ki')

        return [rule0 , rule1 , rule2 , rule3 , rule4 , rule5, rule6 , rule7 , rule8]



    def fuzzy_control(self , rules , flush_after_run = 21 * 21):
        system = ctrl.ControlSystem(rules = rules)
        self.sim = ctrl.ControlSystemSimulation(system, flush_after_run=flush_after_run + 1)
        if self.show == True:
            self.show_plot(self.sim)
            self.show = False

        return self.sim
            

    def show_plot(self):
        self.error.view()
        self.delta.view()
        self.output.view()

        # how the control surface
        # higher resolution could make full accuracy
        upsampled = np.linspace(-2, 2, 21)
        x, y = np.meshgrid(upsampled, upsampled)
        z = np.zeros_like(x)

        for k in range(3):
            for i in range(21):
                for j in range(21):
                    self.sim.input['error'] = x[i, j]
                    self.sim.input['delta'] = y[i, j]
                    self.sim.compute()
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


    def pidcontroller(self,error , errordot ,x, goal_x, nose, waypoint_idx ):
        self.sim.input['error'] = error
        self.sim.input['delta'] = errordot
        self.sim.compute()
        kp = self.sim.output['output_kp']
        ki = self.sim.output['output_ki']
        kd = self.sim.output['output_kd']
        print(kp , ki , kd)
        pid_controller = PID( kp_linear = kp, kd_linear = kd, ki_linear = ki)

        linear_velocity_control, angular_velocity_control = pid_controller.get_control_inputs(x , goal_x , nose , waypoint_idx)

        return linear_velocity_control , angular_velocity_control