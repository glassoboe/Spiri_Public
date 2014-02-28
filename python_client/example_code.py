from spiri import *

""" This is example code to show how to connect to Spiri and communicate with it """
__author__ = "Pleiades Robotics: Nicholas Othieno"
__email__  = "info@pleiades.ca"

spiri1 = Spiri('192.168.1.102')
#spiri1 = Spiri('10.10.0.1')

#Test mode selection/change
mode = 1
errval = spiri1.select_mode(mode)
print('select_mode error value is {}'.format(errval))

#Read the values from the accelerometer
errval, accel_data = spiri1.get_accel_data()
print('get_accel_data error value is {}'.format(errval))
print(accel_data) 

#Read values from the gyro
errval, gyro_data= spiri1.get_gyro_data()
print('get_gyro_data error value is {}'.format(errval))
print(gyro_data)

#Obtain the temperature
errval, temperature_data = spiri1.get_temperature_data()
print('get_temperature_data error value is {}'.format(errval))
print(temperature_data)

#Read the pressure value
errval, pressure_data = spiri1.get_pressure_data()
print('get_pressure_data error value is {}'.format(errval))
print(pressure_data)

#Read the acoustic range sensor
errval, acoustic_range = spiri1.get_acoustic_range_data()
print('get_acoustic_range_data error value is {}'.format(errval))
print(acoustic_range)

#Obtain attitude readings
errval, y_angle_accel, y_angle_error, y_rate, y_motor_correction = spiri1.get_attitude_readings()
print('get_attitude_readings error value is {}'.format(errval))
attitude_str='y_angle_accel, y_angle_error, y_motor_correction, y_rate:\n'+str((y_angle_accel, y_angle_error, y_motor_correction, y_rate))
print(attitude_str)

#Get the speed of a single motor
errval, first_motor_speed = spiri1.get_speed_of_single_motor(1)
print('get_speed_of_single_motor error value is {}'.format(errval))
print(first_motor_speed)

#Get the speed of all motors
(errval, m1, m2, m3, m4)= spiri1.get_speed_of_all_motors()
print('get_speed_of_all_motors error value is {}'.format(errval))
motor_str = 'm1, m2, m3, m4:\n'+str((m1, m2, m3, m4))
print(motor_str)

#Configure the stabilization PID
"""Nick: I commented out this section because it writes to EEPROM on the microcontroller, causing it to fail
errval = spiri1.configure_stabilization((1,2,3),(4,5,6))
print('configure_stabilization error value is {}'.format(errval))"""

#Read stabilization PID values
errval, stab_data=spiri1.get_stabilization_data()
print('get_stabilization_data error value is {}'.format(errval))
print(stab_data)
"""
#Configure altitude PID
pid_alt = spiri1.PID(10,20,30)
errval = spiri1.configure_altitude_PID(pid_alt)
print('configure_altitude_PID error value is {}'.format(errval))"""

#Read altitude PID coeffecients
errval, alt_stab_data = spiri1.get_altitude_PID_configuration()
print('get_altitude_PID_configuration error value is {}'.format(errval))
print(alt_stab_data)

#Set the target altitude
errval = spiri1.set_target_altitude(30)
print('set_target_altitude error value is {}'.format(errval))

#Set the attitude
errval = spiri1.set_attitude(15.0,10.0)
print('set_attitude error value is {}'.format(errval))

#Calibrate accelerometer
""" Nick: I commented out this section because it writes to EEPROM on the microcontroller, causing it to fail
errval = spiri1.calibrate_accelerometer(1.0,1.0)
print('calibrate_accelerometer error value is {}'.format(errval))"""

#Get all sensor data
errval, sensor_data = spiri1.get_all_sensor_data()
print('get_all_sensor_data error value is {}'.format(errval))
for sensor_name, sens_data in sensor_data.items():
  str_sens=str(sensor_name) + ':'
  print(str_sens)
  print(sens_data)
  
#Get magnetometer readings
errval, magnetometer_data = spiri1.get_magnetometer_data()
print('get_magnetometer_data error value is {}'.format(errval))
print(magnetometer_data)

#Get raw accelerometer data
errval, raw_accel_data = spiri1.get_raw_accel_data()
print('get_raw_accel_data error value is {}'.format(errval))
print(raw_accel_data) 
