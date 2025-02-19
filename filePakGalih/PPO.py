# Import necessary libraries and modules for the implementation
import sys  # System-specific parameters and functions
import socket  # Networking interface for socket programming
import threading  # Thread-based parallelism
import random  # Random number generation
import numpy as np  # Numerical operations on arrays, including Q-table
import json  # JSON encoding and decoding
import gym  # OpenAI Gym for creating RL environments
from gym import spaces  # Spaces module to define action and observation spaces
import os  # Operating system interfaces
import subprocess  # To execute system commands (e.g., adjusting data rate)
 
# Define beta value used in vehicle density adjustment formula
beta = 3  # Sensitivity parameter for how vehicle density changes with power transmission
 
# Ensure SUMO_HOME is set, necessary for controlling the SUMO traffic simulation tool
if 'SUMO_HOME' not in os.environ:
    os.environ['SUMO_HOME'] = "/usr/share/sumo"  # Set SUMO_HOME to a default path if not already set
sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))  # Add SUMO tools to the system path
 
import traci  # Import TraCI to control SUMO from Python
 
# Define MCS tables (Modulation and Coding Scheme) for different IEEE 802.11 standards, including 802.11bd
# These tables map the MCS index to the modulation type and data rate
mcs_tables = {
    '802.11a': [
        ('BPSK', 6.0), ('BPSK', 9.0),
        ('QPSK', 12.0), ('QPSK', 18.0),
        ('16-QAM', 24.0), ('16-QAM', 36.0),
        ('64-QAM', 48.0), ('64-QAM', 54.0)
    ],
    '802.11b': [
        ('DSSS', 1.0), ('DSSS', 2.0),
        ('CCK', 5.5), ('CCK', 11.0)
    ],
    '802.11g': [
        ('BPSK', 6.0), ('BPSK', 9.0),
        ('QPSK', 12.0), ('QPSK', 18.0),
        ('16-QAM', 24.0), ('16-QAM', 36.0),
        ('64-QAM', 48.0), ('64-QAM', 54.0)
    ],
    '802.11n': [
        (0, 'BPSK', 6.5, 13.5),
        (1, 'QPSK', 13.0, 27.0),
        (2, 'QPSK', 19.5, 40.5),
        (3, '16-QAM', 26.0, 54.0),
        (4, '16-QAM', 39.0, 81.0),
        (5, '64-QAM', 52.0, 108.0),
        (6, '64-QAM', 58.5, 121.5),
        (7, '64-QAM', 65.0, 135.0)
    ],
    '802.11ac': [
        (0, 'BPSK', 6.5, 13.5, 29.3, 58.5),
        (1, 'QPSK', 13.0, 27.0, 58.5, 117.0),
        (2, 'QPSK', 19.5, 40.5, 87.8, 175.5),
        (3, '16-QAM', 26.0, 54.0, 117.0, 234.0),
        (4, '16-QAM', 39.0, 81.0, 175.5, 351.0),
        (5, '64-QAM', 52.0, 108.0, 234.0, 468.0),
        (6, '64-QAM', 58.5, 121.5, 263.3, 526.5),
        (7, '64-QAM', 65.0, 135.0, 292.5, 585.0),
        (8, '256-QAM', 78.0, 162.0, 351.0, 702.0),
        (9, '256-QAM', None, None, 390.0, 780.0)
    ],
    '802.11ax': [
        (0, 'BPSK', 7.3, 15.0, 30.0, 60.0),
        (1, 'QPSK', 14.6, 30.0, 60.0, 120.0),
        (2, 'QPSK', 21.9, 45.0, 90.0, 180.0),
        (3, '16-QAM', 29.2, 60.0, 120.0, 240.0),
        (4, '16-QAM', 43.9, 90.0, 180.0, 360.0),
        (5, '64-QAM', 58.5, 120.0, 240.0, 480.0),
        (6, '64-QAM', 65.8, 135.0, 270.0, 540.0),
        (7, '64-QAM', 73.1, 150.0, 300.0, 600.0),
        (8, '256-QAM', 87.8, 180.0, 360.0, 720.0),
        (9, '256-QAM', 97.5, 200.0, 400.0, 800.0)
    ],
    '802.11p': [
        ('BPSK', 3.0), ('BPSK', 6.0),
        ('QPSK', 12.0), ('QPSK', 18.0),
        ('16-QAM', 24.0), ('16-QAM', 36.0),
        ('64-QAM', 48.0), ('64-QAM', 54.0)
    ],
    '802.11bd': [
        # Example MCS table for 802.11bd
        (0, 'BPSK', 7.0, 14.0),  # MCS 0
        (1, 'QPSK', 14.0, 28.0),  # MCS 1
        (2, 'QPSK', 21.0, 42.0),  # MCS 2
        (3, '16-QAM', 28.0, 56.0),  # MCS 3
        (4, '16-QAM', 42.0, 84.0),  # MCS 4
        (5, '64-QAM', 56.0, 112.0),  # MCS 5
        (6, '64-QAM', 63.0, 126.0),  # MCS 6
        (7, '64-QAM', 70.0, 140.0),  # MCS 7
        (8, '256-QAM', 84.0, 168.0),  # MCS 8
        (9, '256-QAM', 105.0, 210.0)  # MCS 9
    ]
}
 
# Function to get modulation and data rate based on IEEE standard and bandwidth
def get_modulation_and_datarate(ieee_standard, mcs_index, bandwidth):
    # Fetch the appropriate MCS table based on the IEEE standard
    mcs_table = mcs_tables.get(ieee_standard)
    if not mcs_table:
        # Print an error message if the MCS table for the specified standard is not found
        print(f"No MCS table found for the defined standard {ieee_standard}.")
        return 'N/A', 'N/A'
 
    modulation = None
    datarate = None
 
    # For standards with variable rates based on bandwidth
    if ieee_standard in ['802.11n', '802.11ac', '802.11ax', '802.11bd']:
        for entry in mcs_table:
            if entry[0] == mcs_index:
                modulation = entry[1]
                datarate = entry[2] if bandwidth == 20 else entry[3]
                break
    else:
        # For legacy standards with fixed rates
        modulation, datarate = mcs_table[mcs_index]
 
    return modulation, datarate
 
# Function to adjust data rate using 'iw' command
def adjust_data_rate(interface, datarate):
    try:
        # Use the 'iw' command to set the data rate on the specified wireless interface
        subprocess.check_output(['iw', 'dev', interface, 'set', 'bitrate', f'{datarate}Mbps'])
        print(f"Data rate set to {datarate} Mbps on interface {interface}")
    except subprocess.CalledProcessError as e:
        # Handle any errors that occur during the command execution
        print(f"Failed to set data rate: {e}")
 
# Define the VANET (Vehicular Ad-Hoc Network) environment class using PPO (Proximal Policy Optimization)
class VANETEnv(gym.Env):
    def __init__(self, car_logs, ieee_standard='802.11p', bandwidth=20):
        super(VANETEnv, self).__init__()
 
        # Define the action space as a continuous value representing the change in power transmission
        self.action_space = spaces.Box(low=np.array([-3]), high=np.array([3]), dtype=np.float32)  # Actions: [delta_power]
        
        # Define the observation space as continuous values representing the power transmission and vehicle density
        self.observation_space = spaces.Box(low=np.array([1, 0.001]), high=np.array([30, 0.8]), dtype=np.float32)  # Observations: [power_transmission, vehicle_density]
 
        # Store car logs and initialize environment parameters
        self.car_logs = car_logs
        self.current_state = None
 
        # Set environment constants
        self.CBR_target = 0.6  # Target Carrier-to-Interference Ratio
        self.safety_distance = 100  # Safety distance in meters
        self.path_loss_exponent = 2.5  # Path loss exponent for signal attenuation
        self.sensitivity = -92  # Sensitivity of the receiver in dBm
        self.message_size = 536  # Message size in bytes
 
        # IEEE standard and bandwidth to be used
        self.ieee_standard = ieee_standard
        self.bandwidth = bandwidth
 
    def reset(self):
        # Randomly select a car log and initialize the state
        log = random.choice(self.car_logs)
        self.current_state = np.array([log['power_transmission'], log['vehicle_density']])
        return self.current_state
 
    def step(self, action):
        # Extract current state variables: power transmission (p) and vehicle density (rho)
        p, rho = self.current_state
        delta_p = action[0]  # Extract the action (change in power transmission)
 
        # Apply the action to the current state, ensuring power stays within the valid range [1, 30]
        new_p = np.clip(p + delta_p, 1, 30)
 
        # Recalculate vehicle density based on the new power transmission using the beta value
        rho = rho * ((p / new_p) ** (1 / beta))
        self.current_state = np.array([new_p, rho])  # Update the state with new values
 
        # Assume CBR is provided by the VANET simulation script and is passed in the log
        cbr = self.current_state['cbr']
 
        # Compute the reward based on the updated state
        reward = self.reward_function(cbr, new_p, p)
 
        done = True  # Each step ends the episode in this simplified example
        return self.current_state, reward, done, {}  # Return the new state, reward, and done flag
 
    def reward_function(self, cbr, p, p_prime):
        """Calculate the reward based on the current CBR and power change."""
        def H(x):
            return 1 if x >= 0 else 0  # Heaviside function: Returns 1 if x >= 0, else 0
 
        def g(x, k):
            return x * (H(x) - 2 * H(x - k))  # Custom function to calculate reward components
 
        # Reward function: Combines rewards/penalties based on CBR, power change, and predefined weights
        reward = (self.pi_b * g(cbr, self.kb)) - (self.pi_p_prime * abs(p - p_prime)) - (self.pi_p2 * g(p_prime, 20))
 
        # Additional reward or penalty based on proximity to the target CBR
        if abs(cbr - self.CBR_target) <= 0.025:
            reward += 10  # Bonus for being close to the target
        else:
            reward -= 0.1  # Penalty for deviating from the target
 
        return reward  # Return the calculated reward
 
# Define a PPO agent class that interacts with the VANET environment
class PPOAgent:
    def __init__(self, env, learning_rate=0.0003, gamma=0.99, clip_ratio=0.2, hidden_layers=[64, 64]):
        self.env = env  # The environment the agent will interact with
        self.gamma = gamma  # Discount factor for future rewards
        self.clip_ratio = clip_ratio  # Clipping ratio for policy updates
 
        # Initialize policy and value function parameters
        self.policy_weights = [np.random.randn(env.observation_space.shape[0], env.action_space.shape[0]) / np.sqrt(env.observation_space.shape[0])]
        self.value_weights = np.random.randn(env.observation_space.shape[0]) / np.sqrt(env.observation_space.shape[0])
 
        # Learning rates for updating policy and value function
        self.policy_lr = learning_rate
        self.value_lr = learning_rate
 
    def select_action(self, state):
        """Select an action based on the current policy."""
        # Calculate the mean action using the policy weights
        mean_action = np.dot(state, self.policy_weights[0])
        # Add noise to the action to encourage exploration
        action = np.random.normal(mean_action, self.clip_ratio)
        # Ensure the action stays within the valid action space range
        return np.clip(action, self.env.action_space.low, self.env.action_space.high)
 
    def train(self, num_episodes=10000, batch_size=128):
        all_rewards = []  # Store total rewards for each episode
        for episode in range(num_episodes):
            state = self.env.reset()  # Reset the environment at the start of each episode
            rewards, actions, states, next_states = [], [], [], []
            done = False
            while not done:
                action = self.select_action(state)  # Select an action based on the current state
                next_state, reward, done, _ = self.env.step(action)  # Take the action and observe the result
 
                # Store the experience (state, action, reward, next state)
                states.append(state)
                actions.append(action)
                rewards.append(reward)
                next_states.append(next_state)
 
                state = next_state  # Update the current state
 
                # Perform batch updates if enough experiences have been collected
                if len(rewards) >= batch_size:
                    self.update(states, actions, rewards, next_states)
                    states, actions, rewards, next_states = [], [], [], []
 
            total_reward = np.sum(rewards)  # Calculate the total reward for this episode
            all_rewards.append(total_reward)
 
            # Print progress every 100 episodes
            if episode % 100 == 0:
                print(f"Episode {episode}: Total Reward: {total_reward}")
 
        # Plot the total rewards over episodes
        plt.plot(all_rewards)
        plt.xlabel('Episodes')
        plt.ylabel('Total Reward')
        plt.show()
 
    def update(self, states, actions, rewards, next_states):
        # Convert lists to numpy arrays for vectorized operations
        states = np.array(states)
        actions = np.array(actions)
        rewards = np.array(rewards)
        next_states = np.array(next_states)
 
        # Compute discounted rewards
        discounted_rewards = np.zeros_like(rewards)
        running_add = 0
        for t in reversed(range(0, len(rewards))):
            running_add = running_add * self.gamma + rewards[t]
            discounted_rewards[t] = running_add
 
        # Normalize the discounted rewards to stabilize training
        discounted_rewards = (discounted_rewards - np.mean(discounted_rewards)) / (np.std(discounted_rewards) + 1e-10)
 
        # Calculate advantages (difference between discounted rewards and value estimates)
        values = np.dot(states, self.value_weights)
        advantages = discounted_rewards - values
 
        # Update the policy weights using the advantages
        for i, state in enumerate(states):
            grad = advantages[i] * (actions[i] - np.dot(state, self.policy_weights[0]))
            self.policy_weights[0] += self.policy_lr * np.outer(state, grad)
 
        # Update the value function weights using the advantages
        for i, state in enumerate(states):
            grad_value = advantages[i]
            self.value_weights += self.value_lr * grad_value * state
 
# Define the server-side class to receive car logs and apply PPO optimization
class Server:
    def __init__(self, host='localhost', port=12345):
        self.host = host  # Server hostname or IP address
        self.port = port  # Server port number
        self.car_logs = []  # List to store car logs received from clients
        self.net = None  # Placeholder for the Mininet object, used for network emulation
 
    def start_server(self):
        """Start the server to receive car logs."""
        # Create a TCP/IP socket
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Bind the socket to the host and port
        server_socket.bind((self.host, self.port))
        # Listen for incoming connections (up to 5 simultaneous connections)
        server_socket.listen(5)
        print(f"Server listening on {self.host}:{self.port}...")
 
        # Continuously accept and handle incoming connections
        while True:
            # Accept a new client connection
            client_socket, client_address = server_socket.accept()
            print(f"Connection from {client_address}")
            # Handle the client connection in a new thread to allow concurrent connections
            threading.Thread(target=self.handle_client, args=(client_socket,)).start()
 
    def handle_client(self, client_socket):
        """Handle incoming car logs from the client."""
        try:
            # Receive data from the client (up to 4096 bytes)
            data = client_socket.recv(4096)
            if not data:  # If no data received, return early
                return
            # Decode the received data from JSON format
            decoded_data = json.loads(data.decode('utf-8'))
            # Add the decoded car log to the list of car logs
            self.car_logs.append(decoded_data)
            # Process the car log to optimize its settings using PPO
            self.process_car_log(decoded_data)
        except Exception as e:
            print(f"Error handling client: {e}")  # Print any errors that occur
        finally:
            client_socket.close()  # Ensure the client socket is closed after handling
 
    def process_car_log(self, car_log):
        """Process a car log by optimizing its settings using PPO."""
        # Extract relevant parameters from the car log
        ieee_standard = car_log.get('ieee_standard', '802.11g')
        bandwidth = car_log.get('bandwidth', 20)
        mcs_index = car_log.get('mcs_index', 0)
 
        # Initialize the VANET environment and PPO agent
        env = VANETEnv([car_log], ieee_standard=ieee_standard, bandwidth=bandwidth)
        agent = PPOAgent(env)
        agent.train(num_episodes=1)  # Train the agent for one episode
 
        # Select the optimized action (power transmission) based on the current state
        new_power_transmission = agent.select_action(env.current_state)[0]
 
        # Get the modulation and data rate based on the IEEE standard, MCS index, and bandwidth
        modulation, data_rate = get_modulation_and_datarate(ieee_standard, mcs_index, bandwidth)
 
        # Update the current state in the environment with the new power transmission and vehicle density
        env.current_state = [new_power_transmission, car_log['vehicle_density']]
        print(f"[INFO] Car {car_log['car_id']}: Adjusted Power Transmission to {new_power_transmission}, Data Rate = {data_rate}")
 
        # Apply the optimized settings to the car in the Mininet simulation
        self.apply_optimized_settings(car_log['car_id'], new_power_transmission, data_rate)
 
    def apply_optimized_settings(self, car_id, power_transmission, data_rate):
        """Apply the optimized power transmission and data rate to the car in the simulation."""
        try:
            # Access the car object in Mininet by car_id
            car = self.net.getNodeByName(car_id)
            if car:
                # Apply the optimized power transmission and data rate to the car
                car.setParam("txpower", power_transmission)
                adjust_data_rate(car.wintfs[0].name, data_rate)
                print(f"Applied settings to {car_id}: Power Transmission = {power_transmission}, Data Rate = {data_rate}")
        except Exception as e:
            print(f"Error applying settings to {car_id}: {e}")  # Print any errors that occur
 
def main():
    # Initialize and start the server
    server = Server(host='localhost', port=9999)
    server.start_server()  # Start the server to listen for incoming car logs
 
# Entry point of the script
if __name__ == "__main__":
    main()  # Call the main function to start the server
