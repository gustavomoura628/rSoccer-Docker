import numpy as np
from gymnasium.spaces import Box
from rsoccer_gym.Entities import Ball, Frame, Robot
from rsoccer_gym.ssl.ssl_gym_base import SSLBaseEnv


import math

def normalize_vector(x, y):
    magnitude = math.sqrt(x**2 + y**2)
    if magnitude == 0:
        return 0, 0  # Handle zero vector case
    x_normalized = x / magnitude
    y_normalized = y / magnitude
    return x_normalized, y_normalized

def rotate_vector(x, y, theta):
    # Convert degrees to radians
    radians = math.radians(theta)
    
    # Rotation formula
    x_new = x * math.cos(radians) - y * math.sin(radians)
    y_new = x * math.sin(radians) + y * math.cos(radians)
    
    return x_new, y_new
def vector_magnitude(x, y):
    return math.sqrt(x**2 + y**2)

def vector_difference_angle(x0, y0, x1, y1):
    # Compute dot product and magnitudes
    dot_product = x0 * x1 + y0 * y1
    magnitude_0 = math.sqrt(x0**2 + y0**2)
    magnitude_1 = math.sqrt(x1**2 + y1**2)
    
    # Avoid division by zero
    if magnitude_0 == 0 or magnitude_1 == 0:
        return None  # Undefined angle for zero vectors
    
    # Compute cosine of the angle
    cos_theta = dot_product / (magnitude_0 * magnitude_1)
    
    # Clamp cos_theta to avoid floating-point errors outside valid range
    cos_theta = max(-1, min(1, cos_theta))
    
    # Compute the angle in degrees
    angle = math.degrees(math.acos(cos_theta))
    
    return angle

class SSLExampleEnv(SSLBaseEnv):
    def __init__(self, render_mode=None):
        field = 0 # SSL Division A Field
        super().__init__(field_type=0, n_robots_blue=1,
                         n_robots_yellow=0, time_step=0.025,
                         render_mode=render_mode)
        n_obs = 4 # Ball x,y and Robot x, y, theta
        self.action_space = Box(low=-1, high=1, shape=(2, ))
        self.observation_space = Box(low=-self.field.length/2,\
            high=self.field.length/2,shape=(n_obs, ))

    def _frame_to_observations(self):
        ball, robot = self.frame.ball, self.frame.robots_blue[0]
        return np.array([ball.x, ball.y, robot.x, robot.y])

    def _get_commands(self, actions):
        x, y = rotate_vector(actions[0], actions[1], -self.frame.robots_blue[0].theta)
        return [Robot(yellow=False, id=0,
                      v_x=x, v_y=y)]

    def _calculate_reward_and_done(self):
        robot = self.frame.robots_blue[0]
        ball = self.frame.ball

        rbx = ball.x - robot.x
        rby = ball.y - robot.y

        vel_angle_to_ball = vector_difference_angle(rbx,rby,robot.v_x,robot.v_y)
        #print("Vel angle to ball = ", vel_angle_to_ball)

        distance_to_ball = ((self.frame.ball.x - self.frame.robots_blue[0].x)**2+(self.frame.ball.y - self.frame.robots_blue[0].y)**2)**0.5
        #print("distance_to_ball = ", distance_to_ball)

        reward = (-(vel_angle_to_ball*vector_magnitude(robot.v_x,robot.v_y)**2) -1*distance_to_ball)/100


        done = self.steps > 20*self.metadata["render_fps"]
        if distance_to_ball < 0.12:
            reward = 100
            done = True

        return reward, done
    
    def _get_initial_positions_frame(self):
        pos_frame: Frame = Frame()
        pos_frame.ball = Ball(x=(self.field.length/2)\
            - self.field.penalty_length, y=0.)
        pos_frame.robots_blue[0] = Robot(x=0., y=0., theta=0.,)
        return pos_frame
