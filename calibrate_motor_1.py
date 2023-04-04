# calibration motor 1
# connect to odrive
import odrive
odrv0 = odrive.find_any()

# settings

odrv0.axis1.motor.config.torque_constant = 8.27/270

# calibarted values for motor 1
odrv0.axis1.controller.config.pos_gain = 10

odrv0.axis1.controller.config.vel_gain = 0.15

odrv0.axis1.controller.config.vel_integrator_gain = 0.75

odrv0.axis1.encoder.config.cpr = 20480

odrv0.config.enable_brake_resistor = "true"

odrv0.axis1.controller.config.vel_limit = 20

odrv0.axis1.encoder.config.use_index ="true"

#set to calibarte on startup
odrv0.axis1.config.startup_motor_calibration= "true"

odrv0.axis1.config.startup_encoder_index_search= "true"

odrv0.axis1.config.startup_encoder_offset_calibration= "true"

odrv0.axis1.config.startup_closed_loop_control= "true"

# if error occurs read and clear error with the volowing commands 

#dump_errors(odrv0)

#odrv0.clear_errors()


