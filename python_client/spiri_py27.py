# coding: latin-1

"""Commands to control the Pleiades Spiri drone."""

__author__ = "Pleiades Robotics: Nicholas Othieno"
__email__  = "info@pleiades.ca"

from spiri_connect import * #Holds connection information to Spiri
import struct

BUFFER_SIZE = 128 #Maximum amount of data that can be received
input_buffer = bytearray( BUFFER_SIZE )


class RHALError(Exception):
  """RHAL exception from the flight controller."""
  def __init__(self, value):
    self.value = value
  def __str__(self):
    return repr(self.value)

class ThreeAxis:
  def __init__(self, x, y, z):
    self.x = x
    self.y = y
    self.z = z
    
  def __str__(self):
    return "(x: %f, y: %f, z: %f)"%(self.x, self.y, self.z)

class PID:
  def __init__(self, p, i, d):
    self.p = p
    self.i = i
    self.d = d

  def __str__(self):
    return "(p: %f, i: %f, d: %f)"%(self.p, self.i, self.d)


def __check_res(res):
  if res != 0:
    if res == -1:
      raise RHALError("Unknown command")
    elif res == -2:
      raise RHALError("Parity fail")
    elif res == -3:
      raise RHALError("Param out of bounds")
    elif res == -4:
      raise RHALError("Invalid RHAL state")
    else:
      raise RHALError("Unknown error")
		      
class Spiri(object):
  """ A class that has functions to control a Spiri"""
  def __init__(self, spiri_server_addr):
    self.connection_to_spiri = Spiri_connect(spiri_server_addr)
    self.connection_state = 0 #I will have to figure out the python try cat
    
  def __del__(self):
    try:
      self.connection_to_spiri.disconnect()
    except Exception as e:
      pass #Do nothing
      
  def __str__ (self):
    return self.connection_to_spiri.get_connection_info_string()
    
  def get_all_input(self, pkt_byte_count, IsTCP):
    input_view = memoryview(input_buffer)
    while pkt_byte_count:
      if IsTCP:
        nBytes = self.connection_to_spiri.tcpSpiriSock.recv_into(input_view, pkt_byte_count)
      else:
        nBytes = self.connection_to_spiri.udpSpiriSock.recv_into(input_view, pkt_byte_count)
      #nBytes = self.connection_to_spiri.tcpSpiriSock.recv_into(input_view, pkt_byte_count)
      input_view = input_view[nBytes:] # slicing views is cheap
      pkt_byte_count -= nBytes
    func_offset = input_buffer.find(b'$$') + 6 #Get the offset of function code
    param_offset = func_offset + 4 #Get the offset of first data parameter normally this is the error/success code
    return (func_offset, param_offset)
    
  """Nick: I changed all the functions below, I added res as one of the return values in the tuple because 
  I want users to be able to inspect the err value. Note that the sending format is network endian"""
  def select_mode(self, mode):
    """Mode selection. Obtain a integer valued mode from the user. Current modes allowed are 0 and 1."""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 1
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$',packet_size, FUNCTION_CODE, mode, b'^') ):
      return res
    
    func_offset, param_offset = self.get_all_input(15, True) #We are expecting 15 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return res
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    return res 
    
  def get_accel_data(self):
    """Get accelerometer data. +/-2G for full scale deflection. Returns an instance of ThreeAxis."""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 2
    pkt_padding = 0
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, pkt_padding, b'^') ):
      return (res,ThreeAxis(0, 0, 0))
    
    func_offset, param_offset = self.get_all_input(27, True) #We are expecting 27 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return (res,ThreeAxis(0, 0, 0))
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    (x,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 4 )
    (y,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 8 )
    (z,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 12 )
    return (res,ThreeAxis(x, y, z))
    
  def get_gyro_data(self):
    """Get gyro data. 200 degrees/sec full-scale deflection. Returns an instance of ThreeAxis."""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 3
    pkt_padding = 0
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, pkt_padding, b'^') ):
      return (res,ThreeAxis(0, 0, 0))
    
    func_offset, param_offset = self.get_all_input(27, True) #We are expecting 27 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return (res,ThreeAxis(0, 0, 0))
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    (x,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 4 )
    (y,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 8 )
    (z,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 12 )
    return (res,ThreeAxis(x, y, z))
    
  def get_temperature_data(self):
    """Get temperature data as a float."""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 4
    pkt_padding = 0
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, pkt_padding, b'^') ):
      return (res, 0.0)
    
    func_offset, param_offset = self.get_all_input(19, True) #We are expecting 19 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return (res, 0.0)
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    (sensor_data,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 4 )
    return (res, sensor_data)
    
  def get_pressure_data(self):
    """Get pressure data as a float."""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 5
    pkt_padding = 0
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, pkt_padding, b'^') ):
      return (res, 0.0)
    
    func_offset, param_offset = self.get_all_input(19, True) #We are expecting 19 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return (res, 0.0)
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    (sensor_data,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 4 )
    return (res, sensor_data)
    
  def get_acoustic_range_data(self):
    """Get acoustic range finder data as a float."""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 6
    pkt_padding = 0
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, pkt_padding, b'^') ):
      return (res, 0.0)
    
    func_offset, param_offset = self.get_all_input(19, True) #We are expecting 19 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return (res, 0.0)
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    (sensor_data,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 4 )
    return (res, sensor_data)
    
  def get_attitude_readings(self):
    """Get attitude readings. Returns a tuple of 4 floats (y_angle_accel, y_angle_error, y_motor_correction, y_rate)."""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 7
    pkt_padding = 0
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, pkt_padding, b'^') ):
      return (res, 0, 0, 0, 0)
    
    func_offset, param_offset = self.get_all_input(31, True) #We are expecting 31 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return (res, 0, 0, 0, 0)
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    (a,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 4 )
    (b,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 8 )
    (c,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 12 )
    (d,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 16 )
    return (res, a, b, c, d)
    
  def get_speed_of_single_motor(self, motor):
    """Get the speed of a single motor as a float."""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 8
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, motor, b'^') ):
      return (res, 0.0)
    
    func_offset, param_offset = self.get_all_input(19, True) #We are expecting 19 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return (res, 0.0)
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    (sensor_data,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 4 )
    return (res, sensor_data)
    
  def get_speed_of_all_motors(self):
    """Get the speeds of all motors at once, returns a tuple of 4 floats."""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 9
    pkt_padding = 0
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, pkt_padding, b'^') ):
      return (res, 0, 0, 0, 0)
    
    func_offset, param_offset = self.get_all_input(31, True) #We are expecting 31 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return (res, 0, 0, 0, 0)
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    (a,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 4 )
    (b,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 8 )
    (c,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 12 )
    (d,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 16 )
    return (res, a, b, c, d)
    
  def configure_stabilization(self, x, y):
    """Set PID configuration data - rewrites the constant values in the PID controller for stabilization.
    Expects 2 PID tuples of 3 floats (p, i, d)."""
    res = -5
    packet_size = 35
    FUNCTION_CODE = 10
    if not self.connection_to_spiri.send(struct.pack("!2c2i6fc", b'$', b'$', packet_size, FUNCTION_CODE, x[0], x[1], x[2], y[0], y[1], y[2], b'^') ):
      return res
    
    func_offset, param_offset = self.get_all_input(15, True) #We are expecting 15 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return res
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    return res 
    
  def get_stabilization_data(self):
    """Get stabilization PID configuration data - reads out the constant values in the PID controller for stabilization.
    Returns a tuple of 3 floats (p, i, d)."""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 11
    pkt_padding = 0
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, pkt_padding, b'^') ):
      return (res,ThreeAxis(0, 0, 0))
    
    func_offset, param_offset = self.get_all_input(27, True) #We are expecting 27 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return (res,ThreeAxis(0, 0, 0))
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    (p,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 4 )
    (i,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 8 )
    (d,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 12 )
    return (res, PID(p, i, d) )
    
  def configure_altitude_PID(self, pid):
    """Set altitude PID configuration data. Expects an instance of PID."""
    res = -5
    packet_size = 23
    FUNCTION_CODE = 12
    if not self.connection_to_spiri.send(struct.pack("!2c2i3fc", b'$', b'$', packet_size, FUNCTION_CODE, pid.p, pid.d, pid.i, b'^') ):
      return res
    
    func_offset, param_offset = self.get_all_input(15, True) #We are expecting 15 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return res
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    return res 
    
  def get_altitude_PID_configuration(self):
    """Get Altitude PID configuration data. Returns an instance of PID."""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 13
    pkt_padding = 0
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, pkt_padding, b'^') ):
      return (res,ThreeAxis(0, 0, 0))
    
    func_offset, param_offset = self.get_all_input(27, True) #We are expecting 27 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return (res,ThreeAxis(0, 0, 0))
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    (p,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 4 )
    (i,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 8 )
    (d,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 12 )
    return (res, PID(p, i, d) )
    
  def set_target_altitude(self, altitude):
    """Set Altitude - sets the altitude the robot should move towards. Expects a float."""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 14
    if not self.connection_to_spiri.send(struct.pack("!2c2ifc", b'$', b'$', packet_size, FUNCTION_CODE, altitude, b'^') ):
      return res
    
    func_offset, param_offset = self.get_all_input(15, True) #We are expecting 15 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return res
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    return res
    
  def set_attitude(self, x, y):
    """Set Attitude - sets the X and Y angle from flat the robot should move towards. Expects 2 floats."""
    res = -5
    packet_size = 19
    FUNCTION_CODE = 15
    if not self.connection_to_spiri.send(struct.pack("!2c2i2fc", b'$', b'$', packet_size, FUNCTION_CODE, x, y, b'^') ):
      return res
    
    func_offset, param_offset = self.get_all_input(15, True) #We are expecting 15 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return res
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    return res 
    
  def calibrate_accelerometer(self, x, y):
    """Calibrate the accelerometer. Expects 2 floats: x and y."""
    res = -5
    packet_size = 19
    FUNCTION_CODE = 16
    if not self.connection_to_spiri.send(struct.pack("!2c2i2fc", b'$', b'$', packet_size, FUNCTION_CODE, x, y, b'^') ):
      return res
    
    func_offset, param_offset = self.get_all_input(15, True) #We are expecting 15 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return res
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    return res 
    
  def init_access(self, reg_flight_ctrl):
    """Initialize access to the flight controller, must be invoked before all other functions.
    Set argument to 0 for access to sensors only and 1 to also control the flight processor."""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 0
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$',packet_size, FUNCTION_CODE, reg_flight_ctrl, b'^') ):
      return res
    
    func_offset, param_offset = self.get_all_input(15, True) #We are expecting 15 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return res
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    return res 
    
  def shut_access(self):
    """Shuts down access to the flight controller."""
    res = -5
    packet_size = 15
    FUNCTION_CODE = -1
    pkt_padding = 0
    self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, pkt_padding, b'^') )
    return
    
  def send_dummy_packet(self):#This did not work. But I am leaving it here just in case
    """Use a dummy packet to pad real packets so that our data is always received in a timely fashion at the server"""
    packet_size = 15
    FUNCTION_CODE = -2
    pkt_padding = 0
    self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, pkt_padding, b'^') )
    return
    
    
  def get_all_sensor_data(self):
    """Get all sensor data, returns a dict with the keys: "accel_data", "gyro_data",
    "magnetometer_data", "range" and "pressure" """
    res = -5
    packet_size = 15
    FUNCTION_CODE = 17
    pkt_padding = 0
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, pkt_padding, b'^') ):
      return (res,{"accel_data": ThreeAxis(0, 0, 0),
      "gyro_data": ThreeAxis(0, 0, 0),
      "magnetometer_data": ThreeAxis(0, 0, 0),
      "range": 0,
      "pressure": 0})
    
    func_offset, param_offset = self.get_all_input(59, True) #We are expecting 59 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return (res,{"accel_data": ThreeAxis(0, 0, 0),
      "gyro_data": ThreeAxis(0, 0, 0),
      "magnetometer_data": ThreeAxis(0, 0, 0),
      "range": 0,
      "pressure": 0})
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    (accx,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 4 )
    (accy,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 8 )
    (accz,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 12 )
    (gyrx,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 16 )
    (gyry,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 20 )
    (gyrz,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 24 )
    (magx,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 28 )
    (magy,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 32 )
    (magz,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 36 )
    (acoustic_range,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 40)
    (pressure,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 44 )
    
    return (res,{"accel_data": ThreeAxis(accx, accy, accz),
      "gyro_data": ThreeAxis(gyrx, gyry, gyrz),
      "magnetometer_data": ThreeAxis(magx, magy, magz),
      "range": acoustic_range,
      "pressure": pressure})
      
  def get_magnetometer_data(self):
    """Get readings from magnetometer. Returns an instance of ThreeAxis."""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 18
    pkt_padding = 0
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, pkt_padding, b'^') ):
      return (res,ThreeAxis(0, 0, 0))
    
    func_offset, param_offset = self.get_all_input(27, True) #We are expecting 27 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return (res,ThreeAxis(0, 0, 0))
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    (x,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 4 )
    (y,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 8 )
    (z,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 12 )
    return (res, ThreeAxis(x, y, z) )
  
  def get_raw_accel_data(self):
    """Get accelerometer data. +/-2G for full scale deflection. Returns an instance of ThreeAxis."""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 19
    pkt_padding = 0
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, pkt_padding, b'^') ):
      return (res,ThreeAxis(0, 0, 0))
    
    func_offset, param_offset = self.get_all_input(27, True) #We are expecting 27 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return (res,ThreeAxis(0, 0, 0))
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    (x,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 4 )
    (y,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 8 )
    (z,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 12 )
    return (res, ThreeAxis(x, y, z) )

  def get_GPS(self):
    """Get GPS data. Returns x, y, z, UTC time and HDOP."""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 20
    pkt_padding = 0
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, pkt_padding, b'^') ):
      return (res, 0.0, 0.0, 0.0 ,0.0, 0.0)
    
    func_offset, param_offset = self.get_all_input(35, True) #We are expecting 35 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return (res, 0.0, 0.0, 0.0 ,0.0, 0.0)
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    (x,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 4 )
    (y,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 8 )
    (z,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 12 )
    (UTC_time,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 16 )
    (HDOP,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 20 )
    return (res, x, y, z, UTC_time, HDOP)
    
  def set_Mode_one_flight_parameters(self, pitch_angle_f, roll_angle_f, yaw_angle_f, rpm_f):
    """Set orientation, and avg RPM of the robot should move towards. Expects 4 floats."""
    res = -5
    packet_size = 27
    FUNCTION_CODE = 21
    pitch_angle = int(pitch_angle_f*10)#Multiply by 10 and convert to int16. Flight controller will divide by 10 and convert back to float
    roll_angle = int(roll_angle_f*10)
    yaw_angle = int(yaw_angle_f*10)
    rpm = int(rpm_f)#Convert to int, flight controller will convert back to float
    if not self.connection_to_spiri.UDPsend(struct.pack("!2c2i4ic", b'$', b'$', packet_size, FUNCTION_CODE, pitch_angle, roll_angle, yaw_angle, rpm, b'^') ):
      return res
    
    """func_offset, param_offset = self.get_all_input(15, False) #We are expecting 15 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return res
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )"""
    res = 0
    #print(pitch_angle, roll_angle, yaw_angle, rpm)
    return res 
    
  def get_telemetry(self):
    """Get telemetry"""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 22
    pkt_padding = 0
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, pkt_padding, b'^') ):
      return res, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    
    func_offset, param_offset = self.get_all_input(91, True) #We are expecting 91 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return res, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
      
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    (accx,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 4 )
    (accy,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 8 )
    (accz,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 12 )
    (gyrx,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 16 )
    (gyry,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 20 )
    (gyrz,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 24 )
    (magx,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 28 )
    (magy,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 32 )
    (magz,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 36 )
    (pressure,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 40 )
    (acoustic_range,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 44)
    (OpticalFlowX,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 48)
    (OpticalFlowY,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 52)
    (Motor1,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 56)
    (Motor2,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 60)
    (Motor3,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 64)
    (Motor4,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 68)
    (BatteryVoltage,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 72)
    (BatteryCurrent,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 76)
    
    
    return res,(accx, accy, accz, gyrx, gyry, gyrz, magx, magy, magz, pressure, 
    acoustic_range, OpticalFlowX, OpticalFlowY, Motor1 ,Motor2 ,Motor3 ,Motor4 ,BatteryVoltage ,BatteryCurrent)
    
  def get_angles_and_sensors(self):
    """Get telemetry"""
    res = -5
    packet_size = 15
    FUNCTION_CODE = 23
    pkt_padding = 0
    if not self.connection_to_spiri.send(struct.pack("!2c3ic", b'$', b'$', packet_size, FUNCTION_CODE, pkt_padding, b'^') ):
      return res, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    
    func_offset, param_offset = self.get_all_input(63, True) #We are expecting 91 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return res, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
      
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )
    (accx,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 4 )
    (accy,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 8 )
    (accz,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 12 )
    (gyrx,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 16 )
    (gyry,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 20 )
    (gyrz,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 24 )
    (currentRollAngle,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 28 )
    (currentPitchAngle,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 32 )
    (Motor1,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 36 )
    (Motor2,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 40 )
    (Motor3,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 44)
    (Motor4,) = struct.unpack_from( '!f', buffer(input_buffer),  param_offset + 48)
    
    
    return res,(accx, accy, accz, gyrx, gyry, gyrz, currentRollAngle, currentPitchAngle,
    Motor1 ,Motor2 ,Motor3 ,Motor4)
    
  def set_speed_of_all_motors(self, motor1_f, motor2_f, motor3_f, motor4_f):
    """Set the speed of the motors"""
    res = -5
    packet_size = 27
    FUNCTION_CODE = 24
    motor1 = int(motor1_f)
    motor2 = int(motor2_f)
    motor3 = int(motor3_f)
    motor4 = int(motor4_f)
    if not self.connection_to_spiri.UDPsend(struct.pack("!2c2i4ic", b'$', b'$', packet_size, FUNCTION_CODE, motor1, motor2, motor3, motor4, b'^') ):
      return res
    
    """func_offset, param_offset = self.get_all_input(15, False) #We are expecting 15 bytes from the server
    (func_code,) = struct.unpack_from( '!i', buffer(input_buffer),  func_offset )
    if func_code != FUNCTION_CODE:
      return res
    (res,) = struct.unpack_from( '!i', buffer(input_buffer),  param_offset )"""
    res = 0
    return res
