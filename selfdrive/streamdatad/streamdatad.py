import socket
import fcntl
import struct
import os

from openpilot.common.realtime import Ratekeeper
import cereal.messaging as messaging

BUFFER_SIZE = 1024
UDP_PORT = 5006
TCP_PORT = 5007

def get_wlan_ip():
  for iface in os.listdir('/sys/class/net/'):
    if iface.startswith('wl'):
      try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ip = socket.inet_ntoa(fcntl.ioctl(
          s.fileno(), 0x8915, struct.pack('256s', bytes(iface[:15], 'utf-8'))
        )[20:24])
        return ip
      except IOError:
        pass

class Streamer:
  def __init__(self, sm=None):
    self.local_ip = get_wlan_ip()
    self.ip = None
    self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.tcp_conn = None
    self.sm = sm if sm else messaging.SubMaster(['navInstruction'])
    self.rk = Ratekeeper(10) # Ratekeeper for 10 Hz loop

    self.setup_sockets()

  def setup_sockets(self):
    self.udp_sock.bind((self.local_ip, UDP_PORT))
    self.udp_sock.setblocking(False)
    self.tcp_sock.bind((self.local_ip, TCP_PORT))
    self.tcp_sock.listen(1)
    self.tcp_sock.setblocking(False)

  def send_udp_message(self):
    if self.ip:
      self.sm.update()
      message = self.sm['navInstruction'].as_builder().to_bytes()
      self.udp_sock.sendto(message, (self.ip, UDP_PORT))

  def send_tcp_message(self):
    if self.tcp_conn:
      try:
        self.tcp_conn.sendall("Hello, this is a periodic TCP message from KA2".encode('utf-8'))
      except socket.error:
        self.tcp_conn = None  # Reset connection on error

  def accept_new_connection(self):
    if not self.tcp_conn:
      try:
        self.tcp_conn, addr = self.tcp_sock.accept()
        self.ip = addr[0]  # Update client IP for UDP messages
      except socket.error:
        pass

  def receive_udp_message(self):
    try:
      message, addr = self.udp_sock.recvfrom(BUFFER_SIZE)
      print(f"Received UDP message from {addr}: {message.decode('utf-8')}")
      self.ip = addr[0]  # Update client IP
    except socket.error:
      pass

  def receive_tcp_message(self):
    if self.tcp_conn:
      try:
        message = self.tcp_conn.recv(BUFFER_SIZE, socket.MSG_DONTWAIT)
        if message:
          addr = self.tcp_conn.getpeername()
          print(f"Received TCP message from {addr}: {message.decode('utf-8')}")
      except socket.error:
        pass

  def streamd_thread(self):
    while True:
      self.rk.monitor_time()
      self.send_udp_message()
      self.accept_new_connection()
      self.send_tcp_message()
      self.receive_udp_message()
      self.receive_tcp_message()
      self.rk.keep_time()

  def close_connections(self):
    if self.tcp_conn:
      self.tcp_conn.close()
    self.udp_sock.close()

def main():
  streamer = Streamer()
  streamer.streamd_thread()

if __name__ == "__main__":
  main()

