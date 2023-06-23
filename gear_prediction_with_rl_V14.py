import sys
import csv # Used for writing new data lines
import time
import math
import torch
import smbus # Packages for mpu6050 
import odrive       
import random
import pathlib
import threading
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import multiprocessing as mp # The program uses multi processing to run the data collector, writer and RL AI model at the same time with minimal interfence
from multiprocessing import Queue # Queue objects are used to move data between different processes
from sklearn.preprocessing import MinMaxScaler


'''
Set global vars
'''
# Variables to change for the model
model_name = "conv2d_small_test_2"   # Define the name of the model
model_type = "conv2d"       # Determine the model type; simple, lstm, conv1d or conv2d
total_games = 5           # Epochs amount of the neural network
maximum_steps = 10         # Maximum amount of steps each game
# Regularization method: Lower learning rate to get rid of the NaN values in the loss
learning_rate = 0.0015
# Change the speed of the gear setup
max_speed = 5
min_speed = 2
# Activate null model
null_model = False

# Get file name
file_name = input("Insert file name: ")

# Define paths to save data
model_path = "data/models/{}".format(model_name)       # Define the path for the model
rl_path = "data/rl_games_{}.csv".format(file_name)     # Define the path for saving games performed by the model
samples_path = "data/samples_{}.csv".format(file_name) # Define the path for saving all measurements from each sample (~150 samples per second)

start = time.time()  # Record starting time
stop = 0 # Stop variable to end the program

q = Queue()  # Main queue object for data
qs = Queue() # Queue for stop signal
q_ai_data = Queue(maxsize = 150)  # Window long queue object; Remembers only samples as big as the window
q_predict = Queue(maxsize = 1)    # Queue for broadcasting predictions
q_results = Queue(maxsize = 1)    # Queue for the newest position
q_new_game = Queue(maxsize = 1)   # Queue to start a new game

# Action set for the actions
action_set = {
    0: 'forward',
    1: 'backward',
    2: 'stationary'
}
# Moving set for the moving direction
moving_status = {
    0: 'left',
    1: 'right',
    2: 'still'
}


'''
Main thread to run the odrive
All interaction to the odrive needs to be used in one process, because the connection can't be shared between different threads
'''
# Runs the odrive and all functions that use the odrive
def system(q):
    odrv0 = odrive.find_any() # Defines the odrive
    # adress 6050
    Device_Address = 0x68
    # I2c for 6050
    PWR_MGMT_1   = 0x6B
    SMPLRT_DIV   = 0x19
    CONFIG       = 0x1A
    GYRO_CONFIG  = 0x1B
    INT_ENABLE   = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    GYRO_XOUT_H  = 0x43
    GYRO_YOUT_H  = 0x45
    GYRO_ZOUT_H  = 0x47
    
    # Define the bus to configure the odrive
    bus = smbus.SMBus(1)
    
    # Configures some ports on the odrive
    def MPU_Init():
        bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
        bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
        bus.write_byte_data(Device_Address, CONFIG, 0)
        bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
        bus.write_byte_data(Device_Address, INT_ENABLE, 1)
    
    # Read the raw data of the odrive
    def read_raw_data(addr):
            high = bus.read_byte_data(Device_Address, addr)
            low = bus.read_byte_data(Device_Address, addr+1)
            value = ((high << 8) | low)
            if(value > 32768):
                    value = value - 65536
            return value

    # Increase or decrease the velocity (speed)
    def velocity_change():
        odrv0.axis0.controller.config.control_mode = 2   # 2 is vel; 1 = torque; 0 is volt; 3 is pos

        speed, y, target = 0, 0, 10
        # Continues to accelerate or decelerate
        while True:
            if qs.empty():
                # Increase speed
                if speed <= target:
                    target = max_speed
                    speed = speed + 0.2
                # Decrease speed
                else:
                    target = min_speed
                    speed = speed - 0.2
                # Set new speed
                odrv0.axis0.controller.input_vel = speed                 
                y += 0.8
                time.sleep(3)
            else:
                # Stop rotating the gear
                odrv0.axis0.controller.input_vel = 0
                break
        
    # Change the position of the rotating gear
    def position_change(q):
        odrv0.axis1.controller.config.control_mode = 3   # 2 is vel; 1 = torque; 0 is volt; 3 is pos
        
        pos, x, target = 0, 0, 0
        # Continues to change the position if a new game is started
        while True:
            if qs.empty():
                # Only changes the position if a new game is started
                if q_new_game.empty():
                    time.sleep(0.1)
                else:
                    # Empty the start game queue
                    empty_queue = q_new_game.get()

                    # Start a new game
                    if qs.empty():
                        # Get a random position between -4 and -1.5 or 1.5 and 4
                        pos = random.choice([random.uniform(-4,-1.5),random.uniform(1.5,4)])
                        # Update the odrive with new values
                        odrv0.axis1.controller.input_pos = pos
                        speed = odrv0.axis0.encoder.vel_estimate
                        time.sleep(5)
                        
                    else:
                        # Resets the gear position to the starting position
                        odrv0.axis1.controller.input_pos = 0
                        break
            else:
                break
            
    # Get all measurements
    def get_measurements(out_q):
        MPU_Init()
        # Continues to get all measurements
        while True:
            if qs.empty():
                # Get processed time
                timer = time.process_time()
                # Get delivered power
                epower = odrv0.axis0.controller.electrical_power
                ppower = odrv0.axis0.controller.electrical_power
                # Get gear data like position and velocity
                pos_estimate_main = odrv0.axis0.encoder.pos_estimate
                pos_estimate_rotate = odrv0.axis1.encoder.pos_estimate
                vel_estimate = odrv0.axis0.encoder.vel_estimate
                pos_setpoint = odrv0.axis1.controller.pos_setpoint
                # Get accelerometer data from the IMU
                accel_x = read_raw_data(ACCEL_XOUT_H)
                accel_y = read_raw_data(ACCEL_YOUT_H)
                accel_z = read_raw_data(ACCEL_ZOUT_H)
                # Get gyroscope data from the IMU
                gyro_x = read_raw_data(GYRO_XOUT_H)
                gyro_y = read_raw_data(GYRO_YOUT_H)
                gyro_z = read_raw_data(GYRO_ZOUT_H)
                
                # Data to save in the measurements file
                out_q.put([[timer,pos_estimate_main,pos_estimate_rotate, epower,ppower,vel_estimate,pos_setpoint,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z]])
                
                # Estimated minimum and maximum values
                min_acc, max_acc = -16384, 16384
                min_gyro, max_gyro = -256, 256

                # Correct the IMU measurements to a scale between -1 and 1 instead of a very large scale
                correct_acc_x = (accel_x - min_acc) / (max_acc - min_acc)
                correct_acc_y = (accel_y - min_acc) / (max_acc - min_acc)
                correct_acc_z = (accel_z - min_acc) / (max_acc - min_acc)
                correct_gyro_x = (gyro_x - min_gyro) / (max_gyro - min_gyro)
                correct_gyro_y = (gyro_y - min_gyro) / (max_gyro - min_gyro)
                correct_gyro_z = (gyro_z - min_gyro) / (max_gyro - min_gyro)
                
                # Provide the collected data to use as input for the AI algorithm
                try:
                    q_ai_data.put([[ppower,vel_estimate,correct_acc_x,correct_acc_y,correct_acc_z,correct_gyro_x,correct_gyro_y,correct_gyro_z]], block=False)
                except:
                    pass
            else:
                break
            
    # Rotate the gear in a specific position based on the AI predictions
    def rotate_gear(q):
        # Continues to rotate the gear
        while True:
            if qs.empty():
                # Only rotates the gear if the AI model gives a prediction
                if q_predict.empty():
                    time.sleep(0.001)
                else:
                    # Get the actions provided by the AI model
                    predictions = q_predict.get()
                    prediction = predictions[0] # Get the current prediction
                    previous_action = predictions[1] # Get the last prediction
                    action = 2 # Define action variable; Stationary action; Just in case

                    # Get a random left or right action in the first step
                    if previous_action == -1:
                        action = np.random.randint(0, 2)                    
                    # Check if the prediction is to go forward
                    elif prediction == 0:
                        action = previous_action
                    # Check if the prediction is to go backward
                    elif prediction == 1:
                        action = 1 - previous_action

                    # Rotate the gear to the left
                    if action == 0:
                        pos = 0.1
                        previous_action = action # Remember the new action for the next game
                    # Rotate the gear to the right
                    elif action == 1:
                        pos = -0.1
                        previous_action = action # Remember the new action for the next game
                    # Leave the gear stationary
                    else:
                        pos = 0
                    
                    # Estimate the new position for the attached gear
                    pos_estimate_rotate = odrv0.axis1.encoder.pos_estimate
                    new_position = pos_estimate_rotate - pos
                    
                    # Model meets winning point if it is almost right and stationary
                    if (new_position <= 0.3 and new_position >= -0.3) and pos == 0:
                        # High reward of 10 for winning the game
                        reward = 10
                    # Model is allowed to move while it is still within its boundaries
                    elif new_position <= 4 and new_position >= -4:  
                        odrv0.axis1.controller.input_pos = new_position # Move to the new position
                        print("Current position:", pos_estimate_rotate)
                        print("Correcting posistion to:", new_position)
                        print("Moving status:", moving_status[prediction]) # Gives the direction for moving the gear
                        
                        # Light penalty for not knowing what to do next and leaving the gear stationairy
                        if pos == 0:
                            reward = -0.1
                        # Reward of 0.1 for moving to the correct side
                        elif abs(new_position) < abs(pos_estimate_rotate):
                            reward = 0.1
                        # Penalty of -0.1 for moving to the wrong side
                        elif abs(new_position) > abs(pos_estimate_rotate):
                            reward = -0.1
                    else:
                        # High penalty of -10 for losing the game
                        reward = -10
                        print("Position is more than 4 and therefore out of bounds")
                        
                    # Return reward, new estimated gear position and action to the ai model
                    q_results.put([reward, pos_estimate_rotate, previous_action])
                time.sleep(0.001)
            else:
                break

    # Start the motors and the data reader to rotate the gears in the setup
    measurements_thread = threading.Thread(target=get_measurements, args = (q,))
    position_thread = threading.Thread(target=position_change, args = (q,))
    velocity_thread = threading.Thread(target=velocity_change, args = ())
    rotate_thread = threading.Thread(target=rotate_gear, args = (q,))

    # Start these 4 independent threads
    measurements_thread.start()
    position_thread.start()
    velocity_thread.start()  
    rotate_thread.start()

            
'''
Data processing thread
'''
# Write the collected data to a csv file
def write_data(in_q):
    time.sleep(0.1)
    # Variable for collecting and writing data
    file = open(samples_path, 'a')
    write = csv.writer(file)
    # Write columns header
    write.writerow(["timer", "pos_estimate_main", "pos_estimate_rotate", "epower", "ppower", "vel_estimate", "pos_setpoint", "gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z"])

    while True:
        # Write the collected data to a csv file
        while (in_q.qsize() != 0):
            write.writerow(in_q.get()[0])


'''
AI models
'''
# Null model to simulate a 50/50 win rate
def nullModel(current_step, consistent_action):
    # Get a random consistent rotation direction for the rest of the game
    if current_step == 1:
        consistent_action = random.randint(0, 1)
    # Stationary action after every 5 steps
    if current_step % 6 == 0:
        action = 2
    else: 
        # Get the consistent action
        action = consistent_action
    # Return the next conducted action from the null model
    return action, consistent_action

# Function for creating a simple model which requires a 1D state
def simpleModel():
    # Create a neural network with 1200 input neurons and 3 output neurons
    model = torch.nn.Sequential(
        torch.nn.Linear(1200, 250), # Input layer expects a torch Tensor 2D array of 1200 values
        torch.nn.Dropout(p=0.5),
        torch.nn.LeakyReLU(),
        torch.nn.Linear(250, 50), # Hidden layer is between the size of the input and size of the output
        torch.nn.Dropout(p=0.5),
        torch.nn.LeakyReLU(),
        torch.nn.Linear(50, 3), # Output layer gives the most obvious action
        torch.nn.Softmax(dim=1) # Ensures that the output layer gives one node per class label
    )
    return model


# Class for creating a LSTM model which requires a 2D state
class lstmModel(torch.nn.Module):
    # Initialize the different layers to be used in the model
    def __init__(self):
        super().__init__()                           # Automatically call this Module.__init__() function
        self.lstm = torch.nn.LSTM(input_size=8,      # Input size per sample
                                  hidden_size=8,     # Output size
                                  num_layers=1,      # One layer (No recurrent layers)
                                  batch_first=True)  # Ensures that the input and output tensor are provided as (batch, sequence, feature)
        self.linear_1 = torch.nn.Linear(1200, 32)    # Multiply input size by 5 to incorporate history of 5 steps
        self.linear_2 = torch.nn.Linear(32, 16)      # Fully connected dense layer which narrows the neurons down
        self.linear_3 = torch.nn.Linear(16, 3)       # Dense layer which narrows the neurons to the 3 actions
        self.dropout = torch.nn.Dropout(p=0.5)       # Dropout regularization layer
        self.relu = torch.nn.ReLU()                  # ReLU activation function
        self.flatten = torch.nn.Flatten()            # Flatten the LSTM layer from 2D to 1D
        self.softmax = torch.nn.Softmax(dim=1)       # Ensures that the output layer gives one node back (and it is not multi-label classification with all 3 classes)

    # Initialise the forward layers for the model in the correct sequence
    def forward(self, x):
        self.lstm.flatten_parameters() # Flatten the lstm parameters to prevent overfitting and prevent constantly predicting to stand still
        x, _ = self.lstm(x) # LSTM layer gives a tuple as output, so only use the first value
        x = self.relu(x)
        x = self.flatten(x)

        x = self.linear_1(x)
        x = self.dropout(x) # Regularization layer
        x = self.relu(x)

        x = self.linear_2(x)
        x = self.dropout(x) # Regularization layer
        x = self.relu(x)
        
        x = self.linear_3(x)
        x = self.relu(x)
        x = self.softmax(x)
        return x
    
# Class for creating a Convolutional 1D model
class conv1dModel(torch.nn.Module):
    # Initialize the different layers to be used in the model
    def __init__(self):
        super().__init__()                               # Automatically call this Module.__init__() function
        self.conv1d = torch.nn.Conv1d(in_channels=150,   # Input size per sample
                                      out_channels=150,  # Output data with same length as input data
                                      kernel_size=3,     # Filter size of 3 to get the gyroscope and accelerometer as one value
                                      padding=1)         # Equally pads the input data with zeros on the beginning and end to maintain an output length equal to the input length
        self.linear_1 = torch.nn.Linear(1200, 32)        # Fully connected dense layer which narrows the neurons down
        self.linear_2 = torch.nn.Linear(32, 16)          # Fully connected dense layer which narrows the neurons down
        self.linear_3 = torch.nn.Linear(16, 3)           # Dense layer which narrows the neurons to the 3 actions
        self.dropout = torch.nn.Dropout(p=0.5)           # Dropout regularization layer
        self.softmax = torch.nn.Softmax(dim=1)           # Ensures that the output layer gives one node back (and it is not multi-label classification with all 3 classes)
        self.relu = torch.nn.ReLU()                      # ReLU activation function
        self.flatten = torch.nn.Flatten()                # Flatten the Convolutional layer from 2D to 1D

    # Initialise the forward layers for the model in the correct sequence
    def forward(self, x):
        x = self.conv1d(x)
        x = self.relu(x)
        x = self.flatten(x)

        x = self.linear_1(x)
        x = self.dropout(x)
        x = self.relu(x)
        
        x = self.linear_2(x)
        x = self.dropout(x)
        x = self.relu(x)
        
        x = self.linear_3(x)
        x = self.relu(x)
        x = self.softmax(x)
        return x
    
# Class for creating a Convolutional 2D model
class conv2dModel(torch.nn.Module):
    # Initialize the different layers to be used in the model
    def __init__(self):
        super().__init__()                             # Automatically call this Module.__init__() function
        self.conv2d = torch.nn.Conv2d(in_channels=150,   # Input size per sample
                                      out_channels=150, # Output data with same length as input data
                                      kernel_size=(3, 1),   # Filter size of 3 to get the gyroscope and accelerometer as one value
                                      stride=1,        #
                                      padding=(1, 0))  # Equally pads the input data with zeros on the beginning and end to maintain an output length equal to the input length
        self.linear_1 = torch.nn.Linear(1200, 64)        # Fully connected dense layer which narrows the neurons down
        self.linear_2 = torch.nn.Linear(64, 16)         # Fully connected dense layer which narrows the neurons down
        self.linear_3 = torch.nn.Linear(16, 3)          # Dense layer which narrows the neurons to the 3 actions
        self.dropout = torch.nn.Dropout(p=0.5)         # Dropout regularization layer
        self.softmax = torch.nn.Softmax(dim=1)         # Ensures that the output layer gives one node back (and it is not multi-label classification with all 3 classes)
        self.relu = torch.nn.ReLU()                    # ReLU activation function
        self.flatten = torch.nn.Flatten()

    # Initialise the forward layers for the model in the correct sequence
    def forward(self, x):
        x = self.conv2d(x)
        x = self.relu(x)
        x = self.flatten(x)
        
        x = self.linear_1(x)
        x = self.dropout(x)
        x = self.relu(x)
        
        x = self.linear_2(x)
        x = self.dropout(x)
        x = self.relu(x)

        x = self.linear_3(x)
        x = self.relu(x)
        x = self.softmax(x)
        return x

            
'''
AI thread
'''
# Run the whole system and move the gears based on the RL predicted actions
def ai(q):
    # Check if model already exist
    if pathlib.Path(model_path).is_file():
        # Load already existing model
        model = torch.load(model_path)
        print("\n\nModel '{}' already exists. \nLoading model...".format(model_name))
    else:
        # Create model
        if model_type == "simple":
            model = simpleModel()
        elif model_type == "lstm":
            model = lstmModel()
        elif model_type == "conv1d":
            model = conv1dModel()
        elif model_type == "conv2d":
            model = conv2dModel()
        print("\n\nModel '{}' does not yet exist. \nCreating new model...".format(model_name))

    # Define the Adam optimizer
    optimizer = torch.optim.Adam(
        model.parameters(), # Later code updates the model parameters
        lr=learning_rate)
    
    # Min max scalar for normalization
    min_max_scaler = MinMaxScaler()

    # Get a batch of 150 samples with the 8 sensor values
    def get_state():
        # Create empty 2D array for 8 sensor values and 150 samples
        state = np.zeros((window, 8))
        # Check if there are already 150 new samples
        while q_ai_data.qsize() < window:
            time.sleep(0.1)
            
        # Create loop to get 150 samples
        for i in range(window):
            new_sample_values = q_ai_data.get()            # Get the newest sensor data from the Queue
            state[i, :] = np.asarray(new_sample_values[0]) # Append the 8 sensor values of the 150 samples to the 2D array
            
        state = min_max_scaler.fit_transform(state) # Normalize the 2D array with values between 0 and 1
        state = state.reshape((1, state.shape[0], state.shape[1])) # Transform the 2D array to a 3D array        
        
        # Transform the 3D array to a 4D array
        if model_type == "conv2d":
            state = state.reshape((state.shape[0], state.shape[1], state.shape[2], 1))
        # Get the state in the 3D shape
        if model_type == "lstm" or model_type == "conv1d" or model_type == "conv2d":
            state = torch.from_numpy(state).float() # Create a torch Tensor
        # Get the state in the 2D shape
        if model_type == "simple":
            state = state.flatten()                 # Flatten the 2D array of size (8, 150) to a 1D array of size (1200)
            state = torch.from_numpy(state).float() # Create a torch Tensor
            state = state.reshape(1,1200)           # Reshape the 1D array of size (1200) to a 2D array os size (1, 1200)
        return state                                # Return the batch of 150 samples 
    
    # Moves the gear in the desired position
    def move(action, previous_action):
        # Fills a queue with the actions to take
        q_predict.put([action, previous_action])
        time.sleep(0.5) # Wait for a new prediction and moves the gear
        
        # Checks if the model already made a prediction
        if not q_results.empty():
            # Get the reward, position and action from the results queue
            results = q_results.get()
            reward = results[0]
            position = results[1]
            previous_action = results[2]
        # Give a penalty if the model has not yet made a prediction
        else:
            reward = -0.1
            position = 0
        # Return the new reward and position
        return reward, position, previous_action

    # Use the epsilon greedy strategy for determining the next action
    def choose_epsilon_greedy(qv, eps):
        if np.random.rand() < eps:
            return np.random.randint(0,3) # Chooses random action with a certain probability
        return np.argmax(qv.data.numpy()) # Chooses most obvious action with a certain probability
    
    # Determines how heavily rewards will weigh
    def discount_rewards(rewards, gamma=0.99):
        lenr = len(rewards)
        disc_return = torch.pow(gamma,torch.arange(lenr).float()) * rewards   # Calculate the exponential descending rewards
        disc_return /= disc_return.max()   # Normalise rewards so all rewards are between the range of 0 and 1 to deliver an improvement on numerical stability
        return disc_return

    # Determines the loss function
    def loss_fn(preds, r):
        return -1 * torch.mean(r * torch.log(preds)) # Loss function of policy gradient method

    # Change the weights of the model
    def update_model_weights_batch(transitions):
        # Get all batches needed for the calculation of the loss
        reward_batch = torch.Tensor([r for (s,a,r) in transitions]).flip(dims=(0,))
        disc_returns = discount_rewards(reward_batch) 
        state_batch = torch.cat([s for (s,a,r) in transitions]) 
        action_batch = torch.Tensor([a for (s,a,r) in transitions]) 
        
        # Make predictions and calculate the probabilities and the loss
        pred_batch = model(state_batch) 
        action_batch_clamped = action_batch.clamp(0, 2) # Ensures that all indices are within the range of valid indices for the second dimension of pred_batch (Removes index error)
        prob_batch = pred_batch.gather(dim=1, index=action_batch_clamped.long().unsqueeze(-1)).squeeze(-1) # Contains indices for all 7500 samples in all 50 state batches combined.
        loss = loss_fn(prob_batch, disc_returns)
        print("Loss:", loss)
        print("End of Game:", i, "  Loss:", loss.item())
        losses.append(loss.item()) # Save losses for later visualisations
        
        # Check for NaN values in the loss function
        if math.isnan(loss.item()):
            return True # Do not update model with NaN value
        
        # Update model weights
        optimizer.zero_grad()      # Sets the gradients of all optimized torch.Tensors to zero
        loss.backward()            # Backpropagate calculated loss and outputs a Tensor
        optimizer.step()           # Performs a single optimization forward step and updates the model parameters based on the current gradient
        return False
        
    # Defines variable to save data
    def open_file():
        file = open(rl_path, 'a')  # Create and/or open file for writing all new data 
        return csv.writer(file)    # Defines variable to write new data rows
        
        
    # Define all variables for the RL algorithm
    window = 150       # Define the sample
    losses = []        # Saves all losses
    win_amount = 0     # Saves total wins
    gamma = 0.9        # The gamma is the discount factor and determines how heavily rewards will weigh. As a result, earlier rewards are weighted more heavily than later rewards
    epsilon = 1.0      # The epsilon represents the inverted probability that the most obvious action will actually be taken. The probability starts with a certainty of 100% (actions will be chosen completely random)
    batch_size = 200   # The batch size indicates the size of the random selection values that are modified in the model. This prevents the whole model from being modified and all learned values from being forgotten

    # Write the columns header to the rl data file
    writer = open_file()
    writer.writerow(["game", "elapsed_time", "current_wins", "taken_steps", "total_reward", "loss", "all_actions", "all_positions"])

    # Executes an x amount of epochs
    for i in range(1, total_games+1):
        # Check if the stop button has been pressed
        if qs.empty():
            # Start a new game
            print("\n\nGame:", i)
            q_new_game.put(1)      # Moves starting position to random position between -4 and 4
            time.sleep(5)          # Wait till gear position is in new setpoint
            step_counter = 1       # Counts amount of steps taken in each game
            total_reward = 0       # Calculates the total reward of each game
            consistent_action = 0  # Determine the action variable for the null model
            history_amount = 5     # Number of history steps to give at the lstm model
            transitions = []       # Variable for the normalized rewards
            stacked_states = []    # Add a history for the last 5 states
            all_actions = []       # Saves all actions in each game
            all_positions = []     # Saves all positions in each game
            previous_action = -1   # No previous action; First step
                    
            # Plays a single game till the model meets the win value or has reached the max moves
            while True:
                # Get the current state with the 150 latest samples of the sensor data
                print("Current step:", step_counter, " - Game:", i)
                current_state = get_state()
                
                # Normalize all values in the state between 0 and 1
                normalized_state = torch.nn.functional.normalize(current_state)
                
                # Stack all states of the current game
                if step_counter == 1:
                    stacked_states = normalized_state # Define the variable stacked states with only the first state
                else:
                    stacked_states = torch.cat((stacked_states, normalized_state))

                # Get the index to get the specific amount of history steps
                if step_counter - history_amount < 0:
                    batches_amount = 0
                else: 
                    batches_amount = step_counter - history_amount

                # Get the current state with the history of the most recent batches
                history_state = stacked_states[batches_amount:]
                
                # Get an action for the null model
                if null_model:
                    action, consistent_action = nullModel(step_counter, consistent_action)
                else:
                    # Chose the most obvious action from the model
                    qval = model(history_state)
                    action = choose_epsilon_greedy(qval, epsilon)
                    
                # Stationary rotation surface if the chosen action does not exist
                if action < 0 or action > 2:
                    action = 2
                print("Action:", action_set[action])
                
                # Get the reward and move the spur gear
                reward, position, previous_action = move(action, previous_action)
                print("Reward:", reward, "\n")
                # Checks whether too many steps were taken
                if step_counter >= maximum_steps:
                    reward = -5 # Gives a negative penalty and ends the current game
                total_reward += reward # Sums up all rewards
                
                # Save all important values to use in the RL predictions
                transitions.append((current_state, action, reward))
                all_actions.append(action) # Add all actions of each game to a list
                all_positions.append(round(position, 3)) # Add all positions of each game to a list
                
                # Checks the win or lose statement
                if reward == -10 or reward == -5 or reward == 10:
                    # Checks if the player has won the game
                    if reward == 10:
                        win_amount += 1 # Increases the number of wins for the model
                    break
                
                step_counter += 1

            # Update the weights of the model after each game
            update_model_weights_batch(transitions)

            # Lowering the epsilon value, lowers the probability that the most obvious action is actually taken. At the same time, it increases the probability that a fully random action is taken
            if epsilon > 0.2:
                epsilon -= (2/total_games) # Lowers the epsilon two times as fast

            # Get the elapsed time from the beginning
            elapsed_time = time.process_time()
            print("Elapsed time:", elapsed_time)
            print("Total wins:", win_amount)
            print("Taken steps:", step_counter)
            print("Total reward:", total_reward)


            # Write the new data to the rl data file
            writer = open_file()
            writer.writerow([i, elapsed_time, win_amount, step_counter, total_reward, losses[-1], all_actions, all_positions])
        else:
            break
        
    # Save PyTorch model
    torch.save(model, model_path)
    # Stops whole program if model is done
    qs.put(5)
            

'''
Start treads
'''
# Create threads using multi processing to run the data collector, writer and RL AI model at the same time with minimal interfence
# write_data_thread = mp.Process(target=write_data, args = (q,),daemon=True)
system_thread = mp.Process(target=system, args = (q,),daemon=True)
ai_thread = mp.Process(target=ai, args = (q,),daemon=True)

# Start these threads
# write_data_thread.start()
system_thread.start()
ai_thread.start()


'''
End program
'''
# Stop program if stop variable is not equal to 0
stop = input("Send input to stop all threads") # Waits till user wants to end the Program
qs.put(5) # Stops all threads

# Empty Queue
while q.qsize() != 0:
    time.sleep(0.5)

print('Program done.')
    
exit(0)    