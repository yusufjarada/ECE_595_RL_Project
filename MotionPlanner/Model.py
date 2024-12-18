import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random
import pickle

# Actor-Critic Definitions
class ActorNetwork(nn.Module):
    def __init__(self, state_dim=5, action_dim=2, hidden_dim=256):
        super(ActorNetwork, self).__init__()
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, action_dim)

        self.activation = nn.ReLU()
        self.output_activation = nn.Sigmoid()  # Normalizes wheel velocities to [0, 1]

    def forward(self, state):
        x = self.activation(self.fc1(state))
        x = self.activation(self.fc2(x))
        action = self.output_activation(self.fc3(x))
        v_min = 0.0
        v_max = 1.0
        scaled_action = v_min + action * (v_max - v_min)
        return scaled_action

class CriticNetwork(nn.Module):
    def __init__(self, state_dim=5, action_dim=2, hidden_dim=256):
        super(CriticNetwork, self).__init__()
        self.fc1 = nn.Linear(state_dim + action_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, 1)

        self.activation = nn.ReLU()

    def forward(self, state, action):
        x = torch.cat([state, action], dim=1)
        x = self.activation(self.fc1(x))
        x = self.activation(self.fc2(x))
        value = self.fc3(x)
        return value

# Reward Calculation
def Reward(d_obs_St, d_obs_St1, d_goal_St, d_goal_St1, max_time_step):
    reward = 0
    Negative_high = -1000
    Positive_high = 1000
    alpha = 1
    beta = 100
    terminate_run = 0

    if d_obs_St1 == 0:
        reward = Negative_high
        terminate_run = 1
    else:
        reward -= beta * np.exp(-d_obs_St1)

    if d_goal_St1 == 0:
        reward = Positive_high
        terminate_run = 1
    elif d_goal_St1 < d_goal_St:
        reward += alpha * (d_goal_St - d_goal_St1)
    else:
        reward += alpha * (d_goal_St - d_goal_St1)

    if max_time_step == 50:
        terminate_run = 1

    return reward, terminate_run

# Replay Buffer
class ReplayBuffer:
    def __init__(self, max_size=100000):
        self.buffer = []
        self.max_size = max_size

    def add(self, transition):
        if len(self.buffer) >= self.max_size:
            self.buffer.pop(0)
        self.buffer.append(transition)

    def sample(self, batch_size):
        return random.sample(self.buffer, batch_size)

    def save(self, filename):
        with open(filename, 'wb') as f:
            pickle.dump(self.buffer, f)

    def load(self, filename):
        with open(filename, 'rb') as f:
            self.buffer = pickle.load(f)

# Training Setup
def train_actor_critic(actor, critic, target_actor, target_critic, replay_buffer, actor_optimizer, critic_optimizer, batch_size, gamma=0.99, tau=0.005):
    if len(replay_buffer.buffer) < batch_size:
        return

    # Sample a batch
    transitions = replay_buffer.sample(batch_size)
    states, actions, rewards, next_states, dones = zip(*transitions)

    states = torch.tensor(states, dtype=torch.float32)
    actions = torch.tensor(actions, dtype=torch.float32)
    rewards = torch.tensor(rewards, dtype=torch.float32).unsqueeze(1)
    next_states = torch.tensor(next_states, dtype=torch.float32)
    dones = torch.tensor(dones, dtype=torch.float32).unsqueeze(1)

    # Critic update
    with torch.no_grad():
        target_actions = target_actor(next_states)
        target_q_values = target_critic(next_states, target_actions)
        y = rewards + gamma * (1 - dones) * target_q_values

    critic_loss = nn.MSELoss()(critic(states, actions), y)
    critic_optimizer.zero_grad()
    critic_loss.backward()
    critic_optimizer.step()

    # Actor update
    predicted_actions = actor(states)
    actor_loss = -critic(states, predicted_actions).mean()
    actor_optimizer.zero_grad()
    actor_loss.backward()
    actor_optimizer.step()

    # Soft update of target networks
    for target_param, param in zip(target_actor.parameters(), actor.parameters()):
        target_param.data.copy_(tau * param.data + (1 - tau) * target_param.data)
    for target_param, param in zip(target_critic.parameters(), critic.parameters()):
        target_param.data.copy_(tau * param.data + (1 - tau) * target_param.data)

    #print losses
    print(f"Critic Loss: {critic_loss.item():.4f}, Actor Loss: {actor_loss.item():.4f}")

# Simulation + Training Loop
def run_simulation_and_train(actor, critic, target_actor, target_critic, replay_buffer, num_episodes=100, max_time_steps=50, batch_size=64):
    actor_optimizer = optim.Adam(actor.parameters(), lr=1e-4)
    critic_optimizer = optim.Adam(critic.parameters(), lr=1e-3)

    for episode in range(num_episodes):
        # Reset environment (replace with actual simulation setup)
        state = np.random.uniform(0, 1, size=(5,))  # Example initial state
        total_reward = 0

        for t in range(max_time_steps):
            state_tensor = torch.tensor(state, dtype=torch.float32).unsqueeze(0)
            action = actor(state_tensor).detach().numpy()[0]  # Predict action
            action = np.clip(action, 0, 1)  # Clip actions to valid range

            # Simulate environment step (replace with actual environment step)
            next_state = state + np.random.uniform(-0.1, 0.1, size=(5,))  # Dummy next state
            d_obs_St, d_obs_St1 = state[3], next_state[3]
            d_goal_St, d_goal_St1 = state[4], next_state[4]
            reward, done = Reward(d_obs_St, d_obs_St1, d_goal_St, d_goal_St1, t)
            total_reward += reward

            # Store transition
            replay_buffer.add((state, action, reward, next_state, done))
            state = next_state

            if done:
                break

        print(f"Episode {episode + 1}/{num_episodes}, Total Reward: {total_reward}")

        # Train after each episode
        train_actor_critic(actor, critic, target_actor, target_critic, replay_buffer, actor_optimizer, critic_optimizer, batch_size)

# Initialize Components
if __name__ == "__main__":
    actor = ActorNetwork()
    critic = CriticNetwork()
    target_actor = ActorNetwork()
    target_critic = CriticNetwork()
    target_actor.load_state_dict(actor.state_dict())
    target_critic.load_state_dict(critic.state_dict())
    replay_buffer = ReplayBuffer()

    # Run Simulation and Training
    run_simulation_and_train(actor, critic, target_actor, target_critic, replay_buffer)