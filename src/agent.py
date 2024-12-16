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

