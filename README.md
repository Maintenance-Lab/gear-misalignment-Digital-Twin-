# gear-misalignment-Digital-Twin-
Code and models for the gear misalignment Digital Twin  

# main_DigitalTwin.py 
![main process](/assets/flowchart.png)

## Project structure
    ├── README.md                                       
    ├── kill_motors.py                                  <- kills the motors 
    ├── calibration_motor_1.py                          <- calibrates motor for the platform run when using or having wiped the Odrive board
    ├── calibration_motor_2.py                          <- calibrates motor for the platform run when using or having wiped the Odrive board  
    ├── assest                                          <- images
    ├── imports                                         <- contains the models
    │   ├──export.pkl                                   <- first TSAI model for predicting 600 samples                                  
