__author__ = "Pleiades Robotics: Nicholas Othieno"
__email__  = "info@pleiades.ca"

import socket

PORT = 50001
class Spiri_connect(object):
  """ Class for connecting to a Spiri Server"""
  
  def __init__(self, spiri_server_addr):
    self.ip_addr = spiri_server_addr
    self.addr_and_port = (self.ip_addr, PORT)
    self.tcpSpiriSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.tcpSpiriSock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, True) #Disable Nagle's Algorithm
    try:
      self.tcpSpiriSock.connect(self.addr_and_port)
      self.__connection_state = True
    except socket.error as e:
      self.__connection_state = False
    
  def __str__(self):
    return self.get_connection_info_string()
    
  def is_connected(self):
    return self.__connection_state
    
  def disconnect(self):
    self.tcpSpiriSock.close()
    
  def send(self,data_to_send):
    try:
      #First check if we are connected, then attempt to either reconnect and send or just send if connected
      if not self.__connection_state:
        self.tcpSpiriSock.connect(self.addr_and_port)
        self.__connection_state = True
      self.tcpSpiriSock.sendall(data_to_send)
      #print('Packet sent!')
      return True
    except socket.error as e:
      self.__connection_state = False
      print('Send has failed\n')
      return False
    
    
  def get_connection_info_string(self):
    if self.__connection_state == True:
      spiri_conn_info = 'Spiri at '+ str(self.ip_addr)
    else:
      spiri_conn_info = 'Spiri at ' + str(self.ip_addr) + ' not connected (Connection Failed)'
    return spiri_conn_info
  
  def get_socket(self):
    try:
      if self.__connection_state:
        return self.tcpSpiriSock
    except socket.error as e:
      self.__connection_state = False
      return False
