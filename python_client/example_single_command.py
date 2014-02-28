from spiri import *

""" This is example code to show how to connect to Spiri and communicate with it """
__author__ = "Pleiades Robotics: Nicholas Othieno"
__email__  = "info@pleiades.ca"

spiri1 = Spiri('192.168.1.102')
#spiri1 = Spiri('10.10.0.1')#Spiri is acting as an access point with this IP

#Test mode selection/change
#mode = 1
#errval = spiri1.select_mode(mode)
#print('select_mode error value is {}'.format(errval))

pitch_angle=6.0
roll_angle=14.0
yaw_angle=29.0
rpm=700.0
errval = spiri1.set_Mode_one_flight_parameters(pitch_angle, roll_angle, yaw_angle, rpm)
print('select_mode error value is {}'.format(errval))

