# calibrates motor 2
# connect to odrive
import odrive
odrv0 = odrive.find_any()

# values are calibrated to motor 2 (motor attached to the platform)
odrv0.axis1.motor.config.torque_constant = 8.27/1300

odrv0.axis1.controller.config.pos_gain = 0.4

odrv0.axis1.controller.config.vel_gain = 0.015

odrv0.axis1.controller.config.vel_integrator_gain = 0.19

odrv0.axis1.encoder.config.cpr = 20480

odrv0.axis1.controller.config.vel_limit = 20

odrv0.axis1.encoder.config.use_index ="true"

# set state to idle
odrv0.axis1.requested_state = 1

odrv0.axis0.requested_state = 1

# save configuration must be in idle state
odrv0.save_configuration()

# close loop control
odrv0.axis1.requested_state = 8
