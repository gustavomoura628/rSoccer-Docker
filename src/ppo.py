# -*- coding: utf-8 -*-

# docs and experiment results can be found at https://docs.cleanrl.dev/rl-algorithms/ppo/#ppopy
import os
import random
import time
from dataclasses import dataclass
from collections import defaultdict
import gymnasium as gym
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions.categorical import Categorical
from torch.utils.tensorboard import SummaryWriter


import rsoccer_gym
from env import SSLExampleEnv

"""Hyperparameters and configs"""

from datetime import datetime


@dataclass
class Args:
    exp_name: str = "PPO"
    """the name of this experiment"""
    seed: int = 1
    """seed of the experiment"""
    torch_deterministic: bool = True
    """if toggled, `torch.backends.cudnn.deterministic=False`"""
    cuda: bool = torch.cuda.is_available()
    """if toggled, cuda will be enabled by default"""
    capture_video: bool = True
    """whether to capture videos of the agent performances (check out `videos` folder)"""
    save_model: bool = True
    """whether to save the final model"""

    # Algorithm specific arguments
    env_id: str = "NONE"
    """the id of the environment"""
    total_timesteps: int = 500000
    """total timesteps of the experiments"""
    learning_rate: float = 2.5e-4
    """the learning rate of the optimizer"""
    num_envs: int = 4
    """the number of parallel game environments"""
    num_steps: int = 128
    """the number of steps to run in each environment per policy rollout"""
    anneal_lr: bool = True
    """Toggle learning rate annealing for policy and value networks"""
    gamma: float = 0.99
    """the discount factor gamma"""
    gae_lambda: float = 0.95
    """the lambda for the general advantage estimation"""
    num_minibatches: int = 4
    """the number of mini-batches"""
    update_epochs: int = 4
    """the K epochs to update the policy"""
    norm_adv: bool = True
    """Toggles advantages normalization"""
    clip_coef: float = 0.5
    """the surrogate clipping coefficient"""
    clip_vloss: bool = True
    """Toggles whether or not to use a clipped loss for the value function, as per the paper."""
    ent_coef: float = 0.01
    """coefficient of the entropy"""
    vf_coef: float = 0.5
    """coefficient of the value function"""
    max_grad_norm: float = 0.5
    """the maximum norm for the gradient clipping"""
    target_kl: float = None
    """the target KL divergence threshold"""

    # to be filled in runtime
    batch_size: int = 0
    """the batch size (computed in runtime)"""
    minibatch_size: int = 0
    """the mini-batch size (computed in runtime)"""
    num_iterations: int = 0
    """the number of iterations (computed in runtime)"""


def make_env(idx, capture_video, run_name):
    def thunk():
        if capture_video and idx == 0:
            #env = gym.make(env_id, render_mode="rgb_array")
            env = SSLExampleEnv(render_mode="rgb_array")
            env = gym.wrappers.RecordVideo(env, f"runs/{run_name}/videos")
        else:
            env = SSLExampleEnv()
        env = gym.wrappers.RecordEpisodeStatistics(env)
        return env

    return thunk

def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
    torch.nn.init.orthogonal_(layer.weight, std)
    torch.nn.init.constant_(layer.bias, bias_const)
    return layer


class Agent(nn.Module):
    def __init__(self, envs):
        super().__init__()
        self.critic = nn.Sequential(
            layer_init(nn.Linear(np.array(envs.single_observation_space.shape).prod(), 8)),
            nn.Tanh(),
            layer_init(nn.Linear(8, 8)),
            nn.Tanh(),
            layer_init(nn.Linear(8, 1), std=1.0),
        )
        #self.actor = nn.Sequential(
        #    layer_init(nn.Linear(np.array(envs.single_observation_space.shape).prod(), 8)),
        #    nn.Tanh(),
        #    layer_init(nn.Linear(8, 8)),
        #    nn.Tanh(),
        #    layer_init(nn.Linear(8, envs.single_action_space.n), std=0.01),
        #)
        self.actor_mean = nn.Sequential(
            layer_init(nn.Linear(np.array(envs.single_observation_space.shape).prod(), 8)),
            nn.Tanh(),
            layer_init(nn.Linear(8, 8)),
            nn.Tanh(),
            layer_init(nn.Linear(8, np.prod(envs.single_action_space.shape)), std=0.01),
        )
        self.actor_logstd = nn.Parameter(torch.zeros(1, np.prod(envs.single_action_space.shape)))


    def get_value(self, x):
        return self.critic(x)

#    def get_action_and_value(self, x, action=None):
#        logits = self.actor(x)
#        probs = Categorical(logits=logits)
#        if action is None:
#            action = probs.sample()
#        return action, probs.log_prob(action), probs.entropy(), self.critic(x)
    def get_action_and_value(self, x, action=None):
        mean = self.actor_mean(x)
        std = self.actor_logstd.exp().expand_as(mean)
        dist = torch.distributions.Normal(mean, std)
        if action is None:
            action = dist.sample()
        return action, dist.log_prob(action).sum(axis=-1), dist.entropy().sum(axis=-1), self.critic(x)


from tqdm import tqdm


def run(args: Args):
    metrics = defaultdict(list)
    args.batch_size = int(args.num_envs * args.num_steps)
    args.minibatch_size = int(args.batch_size // args.num_minibatches)
    args.num_iterations = args.total_timesteps // args.batch_size
    args.run_name = f"SSLExampleEnv__{args.exp_name}__{args.seed}__{int(time.time())}"

    writer = SummaryWriter(f"runs/{args.run_name}")
    writer.add_text(
        "hyperparameters",
        "|param|value|\n|-|-|\n%s" % ("\n".join([f"|{key}|{value}|" for key, value in vars(args).items()])),
    )

    # seeding
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.backends.cudnn.deterministic = args.torch_deterministic

    device = torch.device("cuda" if torch.cuda.is_available() and args.cuda else "cpu")

    # env setup
    envs = gym.vector.SyncVectorEnv(
        [make_env(i, args.capture_video, args.run_name) for i in range(args.num_envs)],
    )
    #assert isinstance(envs.single_action_space, gym.spaces.Discrete), "only discrete action space is supported"
    assert isinstance(envs.single_action_space, gym.spaces.Box), "only continuous action space is supported"


    agent = Agent(envs).to(device)
    optimizer = optim.Adam(agent.parameters(), lr=args.learning_rate, eps=1e-5)

    # ALGO Logic: Storage setup
    obs = torch.zeros((args.num_steps, args.num_envs) + envs.single_observation_space.shape).to(device)
    actions = torch.zeros((args.num_steps, args.num_envs) + envs.single_action_space.shape).to(device)
    logprobs = torch.zeros((args.num_steps, args.num_envs)).to(device)
    rewards = torch.zeros((args.num_steps, args.num_envs)).to(device)
    dones = torch.zeros((args.num_steps, args.num_envs)).to(device)
    values = torch.zeros((args.num_steps, args.num_envs)).to(device)

    # start the game
    global_step = 0
    start_time = time.time()
    next_obs, _ = envs.reset(seed=args.seed)
    next_obs = torch.Tensor(next_obs).to(device)
    next_done = torch.zeros(args.num_envs).to(device)

    # --------------------------------------------------------------------------------

    pbar = tqdm(range(1, args.num_iterations + 1), desc="Iterations")
    for iteration in pbar:
        # annealing the rate if instructed to do so.
        if args.anneal_lr:
            frac = 1.0 - (iteration - 1.0) / args.num_iterations
            lrnow = frac * args.learning_rate
            optimizer.param_groups[0]["lr"] = lrnow

        for step in range(0, args.num_steps):
            global_step += args.num_envs
            obs[step] = next_obs
            dones[step] = next_done

            # action logic
            with torch.no_grad():
                action, logprob, _, value = agent.get_action_and_value(next_obs)
                values[step] = value.flatten()
            actions[step] = action
            logprobs[step] = logprob

            # execute the game and log data.
            #next_obs, reward, terminations, truncations, infos = envs.step(action.cpu().numpy())
            next_obs, reward, terminations, truncations, infos = envs.step(action.cpu().numpy().clip(envs.single_action_space.low, envs.single_action_space.high))

            next_done = np.logical_or(terminations, truncations)
            rewards[step] = torch.tensor(reward).to(device).view(-1)
            #print("Reward = ", reward[0])
            next_obs, next_done = torch.Tensor(next_obs).to(device), torch.Tensor(next_done).to(device)

            if "final_info" in infos:
                for info in infos["final_info"]:
                    if info and "episode" in info:
                        writer.add_scalar("charts/episodic_return", info["episode"]["r"], global_step)
                        writer.add_scalar("charts/episodic_length", info["episode"]["l"], global_step)
                        metrics["charts/episodic_return"].append(info["episode"]["r"])
                        metrics["charts/episodic_length"].append(info["episode"]["l"])

                        pbar.set_postfix_str(f"step={global_step}, return={info['episode']['r']}")

        # bootstrap value if not done
        with torch.no_grad():
            next_value = agent.get_value(next_obs).reshape(1, -1)
            advantages = torch.zeros_like(rewards).to(device)
            lastgaelam = 0
            for t in reversed(range(args.num_steps)):
                if t == args.num_steps - 1:
                    nextnonterminal = 1.0 - next_done
                    nextvalues = next_value
                else:
                    nextnonterminal = 1.0 - dones[t + 1]
                    nextvalues = values[t + 1]
                delta = rewards[t] + args.gamma * nextvalues * nextnonterminal - values[t]
                advantages[t] = lastgaelam = delta + args.gamma * args.gae_lambda * nextnonterminal * lastgaelam  # !!! tirei o nextnonterminal
            returns = advantages + values

        # flatten the batch
        b_obs = obs.reshape((-1,) + envs.single_observation_space.shape)
        b_logprobs = logprobs.reshape(-1)
        b_actions = actions.reshape((-1,) + envs.single_action_space.shape)
        b_advantages = advantages.reshape(-1)
        b_returns = returns.reshape(-1)
        b_values = values.reshape(-1)

        # Optimizing the policy and value network
        b_inds = np.arange(args.batch_size)
        clipfracs = []
        for epoch in range(args.update_epochs):
            np.random.shuffle(b_inds)
            for start in range(0, args.batch_size, args.minibatch_size):
                end = start + args.minibatch_size
                mb_inds = b_inds[start:end]

                _, newlogprob, entropy, newvalue = agent.get_action_and_value(b_obs[mb_inds], b_actions.long()[mb_inds])
                logratio = newlogprob - b_logprobs[mb_inds]  # !!! inverti
                ratio = logratio.exp()

                with torch.no_grad():
                    # calculate approx_kl http://joschu.net/blog/kl-approx.html
                    old_approx_kl = (-logratio).mean()
                    approx_kl = ((ratio - 1) - logratio).mean()
                    clipfracs += [((ratio - 1.0).abs() > args.clip_coef).float().mean().item()]

                mb_advantages = b_advantages[mb_inds]
                if args.norm_adv:
                    mb_advantages = (mb_advantages - mb_advantages.mean()) / (mb_advantages.std() + 1e-8)

                # Policy loss
                pg_loss1 = -mb_advantages * ratio  # !!! mudei para postiivo
                pg_loss2 = -mb_advantages * torch.clamp(ratio, 1 - args.clip_coef, 1 + args.clip_coef)  # !!! mudei para postiivo
                pg_loss = torch.max(pg_loss1, pg_loss2).mean()  # !!! mudei para min

                # Value loss
                newvalue = newvalue.view(-1)
                if args.clip_vloss:
                    v_loss_unclipped = (newvalue - b_returns[mb_inds]) ** 2
                    v_clipped = b_values[mb_inds] + torch.clamp(
                        newvalue - b_values[mb_inds],
                        -args.clip_coef,
                        args.clip_coef,
                    )
                    v_loss_clipped = (v_clipped - b_returns[mb_inds]) ** 2
                    v_loss_max = torch.max(v_loss_unclipped, v_loss_clipped)
                    v_loss = 0.5 * v_loss_max.mean()
                else:
                    v_loss = 0.5 * ((newvalue - b_returns[mb_inds]) ** 2).mean()

                # Entropy Loss
                entropy_loss = entropy.mean()
                loss = pg_loss - args.ent_coef * entropy_loss + v_loss * args.vf_coef  # !!! mudei a entropia pra positivo

                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(agent.parameters(), args.max_grad_norm)
                optimizer.step()

            if args.target_kl is not None and approx_kl > args.target_kl:
                break

        y_pred, y_true = b_values.cpu().numpy(), b_returns.cpu().numpy()
        var_y = np.var(y_true)
        explained_var = np.nan if var_y == 0 else 1 - np.var(y_true - y_pred) / var_y

        # TRY NOT TO MODIFY: record rewards for plotting purposes
        writer.add_scalar("charts/learning_rate", optimizer.param_groups[0]["lr"], global_step)
        writer.add_scalar("losses/value_loss", v_loss.item(), global_step)
        writer.add_scalar("losses/policy_loss", pg_loss.item(), global_step)
        writer.add_scalar("losses/entropy", entropy_loss.item(), global_step)
        writer.add_scalar("losses/old_approx_kl", old_approx_kl.item(), global_step)
        writer.add_scalar("losses/approx_kl", approx_kl.item(), global_step)
        writer.add_scalar("losses/clipfrac", np.mean(clipfracs), global_step)
        writer.add_scalar("losses/explained_variance", explained_var, global_step)
        writer.add_scalar("charts/SPS", int(global_step / (time.time() - start_time)), global_step)
        metrics["losses/value_loss"].append(v_loss.item())
        metrics["losses/policy_loss"].append(pg_loss.item())
        metrics["losses/entropy"].append(entropy_loss.item())
        metrics["losses/SPS"].append(int(global_step / (time.time() - start_time)))


    # --------------------------------------------------------------------------------

    if args.save_model:
        model_path = f"runs/{args.run_name}/{args.exp_name}_model"
        torch.save(agent.state_dict(), model_path)

    envs.close()
    writer.close()

    # final evaluation
    eval_episodes = 3
    envs = gym.vector.SyncVectorEnv([make_env(0, True, args.run_name+"final_recording")])
    agent.eval()
    obs, _ = envs.reset()
    episodic_returns = []
    finished_episodes = 0
    while finished_episodes < eval_episodes:
        action, logprob, entropy, value = agent.get_action_and_value(torch.Tensor(obs).to(device))
        action = action.cpu().numpy().clip(envs.single_action_space.low, envs.single_action_space.high)
        next_obs, reward, terminations, truncations, infos = envs.step(action)
        obs = next_obs
        if terminations[0] or truncations[0]:
            finished_episodes+=1
        #if "final_info" in infos:
        #    for info in infos["final_info"]:
        #        if "episode" not in info:
        #            continue
        #        print(f"eval_episode={len(episodic_returns)}, episodic_return={info['episode']['r']}")
        #        episodic_returns += [info["episode"]["r"]]
    return metrics

args = Args(
    env_id = "NONE",
    total_timesteps = 10000000,
    learning_rate= 0.02,
    num_envs=12,
    num_steps=60*41,
    update_epochs=4,
    clip_coef=0.1,
    vf_coef=0.5,
    ent_coef=0.01,
    gamma=0.99,
    gae_lambda=0.95
)

metrics = run(args)