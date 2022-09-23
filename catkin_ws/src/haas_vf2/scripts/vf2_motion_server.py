#!/usr/bin/env python

#TODO This service should take in an x and y position and emit joint position messages that achieve that position.


# /y_axis_to_x_axis_controller/command std_msgs/Float64 10.0



from __future__ import print_function

from haas_vf2.srv import vf2, vf2Response
from std_msgs.msg import Float64
import rospy
import time
import numpy as np

class Obstacle():
    def __init__(self, position, radius):
        self.position = position
        self.radius = radius
        
    def test(self, point, padding=0.0):
        return np.linalg.norm(self.position-point) <= (self.radius + padding)
        
def normalize_vector(v):
    return v / np.sqrt(np.sum(v**2))

def gaussian_sample(val, sigma):
    return np.random.normal(val, sigma)

class MotionPlanner():
    def __init__(self, config):
        self.x_pub = rospy.Publisher('/y_axis_to_x_axis_controller/command', Float64, queue_size=1)
        self.y_pub = rospy.Publisher('/base_to_y_axis_controller/command', Float64, queue_size=1)

        self.part_size = np.array(config['part_size'])
        self.step = config['step']
        self.sigma_init = config['sigma_init']
        self.sigma_plus = config['sigma_plus']
        self.num_samples = config['num_samples']
        self.max_turns = config['max_turns']
        self.padding = config['padding']
        self.interlude = config['interlude']

        self.obstacles = []

        self.chosen_points = []
        self.current_position =  np.array([0.0, 0.0])

    def set_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def check_collisions(self, position, padding=None):
        for _obstacle in self.obstacles:
            if _obstacle.test(position, padding):
                return True
        return False

    def handle(self,req):
        # collect targes position
        x_pos  = float(req.x_pos)
        y_pos = float(req.y_pos)
        target_position = np.array([x_pos, y_pos])

        if self.plan(target_position):
            print("motion plan completed... moving...")
            self.run()
            print("finished moving.")
        else:
            print("failed to complete motion plan. aborting")

        return vf2Response(*self.current_position)

    def plan(self, end_position):
        step = self.step
        sigma = self.sigma_init
        num_samples = self.num_samples

        max_turns = self.max_turns
        padding = self.padding

        turns = []


        current_position = self.current_position
        retry = False

        # for turn in range(max_turns):
        while np.linalg.norm(end_position - current_position):
            if len(turns) >= max_turns:
                print(f"failed to reach end_position after {len(turns)} turns")
                return False
                break
            
            turn = {
                'valid': [],
                'selected': None,
                'rejected': []
            }    
            
            # compute target-current position path_to_exit as a unit vector
            path_to_exit = end_position - current_position
            
            # SUCCESS stopping condition
            if np.linalg.norm(path_to_exit) <= step:
                print(f"Reached the end_position after {len(turns)} turns")
                turn['selected'] = end_position
                break
            else:  
                way_to_exit = normalize_vector(path_to_exit)

                sampler = lambda t: gaussian_sample(t, sigma)

                samples = [current_position + sampler(way_to_exit) for _ in range(num_samples)]

                samples.sort( key=lambda s:np.linalg.norm(end_position-s))


                # process samples         
                for _s in samples:
                    part_center = _s - (self.part_size/2)
                    if self.check_collisions(part_center, padding=padding):
                        turn['rejected'].append(_s)
                    else:
                        if turn['selected'] is None:
                            turn['selected'] = _s
                        else: 
                            turn['valid'].append(_s)

                # if there are no valid samples... increase variance and try again
                if turn['selected'] is not None:
                    current_position = turn['selected']
                    retry = False
                    sigma = self.sigma_init
                else:    
                    print('retry')
                    current_position = current_position
                    retry = True
                    sigma += self.sigma_plus
                
            turns.append(turn)

            


        self.chosen_points.append(self.current_position)
        for _t in turns:
            if _t['selected'] is not None:
                self.chosen_points.append(_t['selected'])

        return True

    def run(self):
        for _p in self.chosen_points:
            self.send(*_p)
            time.sleep(self.interlude)
        self.chosen_points = []
        return

    def send(self, x_pos, y_pos):
        self.x_pub.publish(x_pos)
        self.y_pub.publish(y_pos)
        return


def handle_motion_server():
    rospy.init_node('vf2_motion_server')

    # set up the sampling based motion planner
    motion_planner_config = {
        'part_size': [3, 3],
        'step': 1,
        'interlude': 0.25,
        'sigma_init': 0.5,
        'sigma_plus': 0.1,
        'num_samples': 5,
        'max_turns': 50,
        'padding': 2.0
    }
    motion_planner = MotionPlanner(motion_planner_config)

    # The obstacle
    obstacle_position = (12.0, 12.0)
    obstacle_radius = 5.0
    padding = 2.0
    # padding = 0.0
    obstacle = Obstacle(obstacle_position, obstacle_radius)
    motion_planner.set_obstacle(obstacle)

    # handle requests
    s = rospy.Service('vf2_motion', vf2, motion_planner.handle)
    print("Ready to plan and execute motion.")
    rospy.spin()

if __name__ == "__main__":
    handle_motion_server()