# gear-misalignment-Digital-Twin-
Code and models for the gear misalignment Digital Twin  
The system is controlled by a pi4 4gb connected to a Odrive motor driver. 
The main script is used to collect data, run the ai and write a .csv.
The other script provided are to control the motors. 
There is also an import folder for the W&B for the ai. 

## Project structure
    ├── README.md                                       
    ├── kill_motors.py                                  <- kills the motors 
    ├── calibration_motor_1.py                          <- calibrates motor for the platform run when using or having wiped the Odrive board
    ├── calibration_motor_2.py                          <- calibrates motor for the platform run when using or having wiped the Odrive board  
    ├── assest                                          <- images
    ├── imports                                         <- contains the models
    │   ├──export.pkl                                   <- first TSAI model for predicting 600 samples                                  
    ├── tsai_loop_V1_local.ipynb                        <- loop to train and save multiple models                
    ├── tsai_V1_local.ipynb                             <- training a model and visualising data 

# flow shart for main_DigitalTwin.py 
![main process](/assets/flowchart.png)

# model predictions 
Predictions of the model on the dataset (not actively correcting the position)
![predictions](/assets/[0, 3, 5, 7, 8, 9].png)

# calobrating the motors 
The motors are calibrated by hand, the values found are in the calibration_motor.py scripts if there is a problem with the calibration of the motors (motors exit its control mode or jittering) run the calibration scripts. If this doesn't work follow Odrives instructions on calibrating the motors ( https://docs.odriverobotics.com/v/0.5.4/control.html )

# training of the ai is done 
the ai's are train using Fast.ai and TSAI the first script to train and the loop used for the research. 

