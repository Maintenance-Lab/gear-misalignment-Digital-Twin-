# connect to odrive and kill motors
import odrive
odrv0 = odrive.find_any()
# set speed and position to zero
odrv0.axis0.controller.input_vel = 0
odrv0.axis0.controller.input_pos = 0
# set state to idle
odrv0.axis0.requested_state = 1

# same for motor 2
odrv0.axis1.controller.input_vel = 0
odrv0.axis1.controller.input_pos = 0
odrv0.axis1.requested_state = 1