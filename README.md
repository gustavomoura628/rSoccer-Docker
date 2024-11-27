# rSoccer Docker
## Dockerfile tutorial
**Build image:**
```bash
docker build -t rl_ssl_rsoccer .
```

**X11 forwarding**
```bash
sudo xhost +local:docker
```

**Run container**
```bash
docker run -it --name=rl_ssl_rsoccer --net=host --env DISPLAY=$DISPLAY rl_ssl_rsoccer
```

**Run agent**
```bash
python3.10 agent.py
```

## Manual docker creation (mostly just here to show you how we got to the final dockerfile)


Enable X11 forwarding
```bash
sudo xhost +local:docker
```

start up the container
```bash
docker run -it --name=rl_ssl_rsoccer --net=host --env DISPLAY=$DISPLAY ubuntu:24.04
```


annoying tzdata thing
```bash
apt update
DEBIAN_FRONTEND=noninteractive apt install -y tzdata # usually tzdata is downloaded as a requirement of other stuff, and it asks you to enter a timezone, this just does it automatically
```


open dynamics engine installation. Not actually needed but good to know if you want to build rSim from source
```bash
#apt install -y wget tar gzip build-essential
#wget https://bitbucket.org/odedevs/ode/downloads/ode-0.16.2.tar.gz
#tar xf ode-0.16.2.tar.gz
#cd ode-0.16.2
#./configure
#make
#make install
```


python 3.10 stuff
```bash
apt install -y software-properties-common # For adding a repo
add-apt-repository -y ppa:deadsnakes/ppa #python3.7 repo
apt install -y python3.10 python3.10-distutils
apt install -y curl
curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10
```

gymnasium and QoL
```bash
python3.10 -m pip install gymnasium
apt install -y neovim tree bat btop
```

rsoccer from source
```bash
python3.10 -m pip install rc-robosim
apt install -y git
git clone https://github.com/robocin/rSoccer.git
cd rSoccer
python3.10 -m pip install .
```

rsoccer prebuilt WARNING: DO NOT USE THIS!!!!! IT IS AN EXTREMELY OLD VERSION
```bash
# python3.10 -m pip install rsoccer-gym
```


env.py
```bash
import numpy as np
from gymnasium.spaces import Box
from rsoccer_gym.Entities import Ball, Frame, Robot
from rsoccer_gym.ssl.ssl_gym_base import SSLBaseEnv


class SSLExampleEnv(SSLBaseEnv):
    def __init__(self, render_mode=None):
        field = 0 # SSL Division A Field
        super().__init__(field_type=0, n_robots_blue=1,
                         n_robots_yellow=0, time_step=0.025,
                         render_mode=render_mode)
        n_obs = 4 # Ball x,y and Robot x, y
        self.action_space = Box(low=-1, high=1, shape=(2, ))
        self.observation_space = Box(low=-self.field.length/2,\
            high=self.field.length/2,shape=(n_obs, ))

    def _frame_to_observations(self):
        ball, robot = self.frame.ball, self.frame.robots_blue[0]
        return np.array([ball.x, ball.y, robot.x, robot.y])

    def _get_commands(self, actions):
        return [Robot(yellow=False, id=0,
                      v_x=actions[0], v_y=actions[1])]

    def _calculate_reward_and_done(self):
        if self.frame.ball.x > self.field.length / 2 \
            and abs(self.frame.ball.y) < self.field.goal_width / 2:
            reward, done = 1, True
        else:
            reward, done = 0, False
        return reward, done
    
    def _get_initial_positions_frame(self):
        pos_frame: Frame = Frame()
        pos_frame.ball = Ball(x=(self.field.length/2)\
            - self.field.penalty_length, y=0.)
        pos_frame.robots_blue[0] = Robot(x=0., y=0., theta=0,)
        return pos_frame
```

agent.py
```bash
import gymnasium as gym
import rsoccer_gym
from env import SSLExampleEnv

# Using VSS Single Agent env
env = SSLExampleEnv(render_mode="human")

env.reset()
# Run for 1 episode and print reward at the end
for i in range(1):
    terminated = False
    truncated = False
    while not (terminated or truncated):
        # Step using random actions
        action = env.action_space.sample()
        action = [0.5,0]
        print("action", action)
        next_state, reward, terminated, truncated, _ = env.step(action)
        print("next_state", next_state)
        print("reward",reward)
        print("terminated",terminated)
        print("truncated",truncated)
    print(reward)
```
