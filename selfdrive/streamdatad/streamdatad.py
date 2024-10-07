import socket
import time
import fcntl
import struct
import os

from openpilot.common.realtime import Ratekeeper
import cereal.messaging as messaging

BUFFER_SIZE = 1024  # Constant for the buffer size

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
  def __init__(self, client_ip, sm=None):
    self.local_ip = get_wlan_ip()
    self.ip = client_ip
    self.udp_port = 5006
    self.tcp_port = 5007
    self.udp_sock = None
    self.tcp_sock = None
    self.tcp_conn = None

    # Setup message subscriber
    self.sm = sm if sm else messaging.SubMaster(['navInstruction'])
    self.rk = Ratekeeper(10, print_delay_threshold=None)

  def setup_udp_endpoint(self):
    try:
      self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      self.udp_sock.bind((self.local_ip, self.udp_port))
      self.udp_sock.setblocking(False)  # Set to non-blocking mode
    except socket.error as e:
      self.udp_sock = None
      print(f"Failed to set up UDP endpoint: {e}")  # Log the error for debugging

  def setup_tcp_endpoint(self):
    try:
      self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      self.tcp_sock.bind((self.local_ip, self.tcp_port))
      self.tcp_sock.listen(1)
      self.tcp_sock.setblocking(False)  # Set to non-blocking mode
    except socket.error as e:
      self.tcp_sock = None
      print(f"Failed to set up TCP endpoint: {e}")  # Log the error for debugging

  def is_udp_connected(self):
    return self.udp_sock is not None

  def is_tcp_connected(self):
    return self.tcp_conn is not None

  def update_and_publish(self):
    if self.is_udp_connected():
      self.sm.update()  # Update the message
      message = "Hello, this is a periodic UDP message from KA2"  # Fixed message to send
      self.udp_sock.sendto(message.encode('utf-8'), (self.ip, self.udp_port))

  def send_tcp_message(self, message):
    if self.is_tcp_connected():
      if message:  # Only send if the message is not empty
        try:
          self.tcp_conn.sendall(message.encode('utf-8'))
        except socket.error:
          self.tcp_conn = None
        except Exception:
          pass  # Handle unexpected errors silently

  def accept_new_connection(self):
    if self.tcp_conn is None:
      if self.tcp_sock is not None:  # Check if tcp_sock is valid
        try:
          self.tcp_conn, _ = self.tcp_sock.accept()
        except socket.error:
          pass  # Handle the error without breaking the flow
      else:
        print("TCP socket is not initialized, cannot accept new connection.")

  def receive_udp_message(self):
    try:
      message, addr = self.udp_sock.recvfrom(BUFFER_SIZE)  # Use the constant buffer size
      print(f"Received UDP message from {addr}: {message.decode('utf-8')}")
      return True
    except socket.error:
      return False

  def receive_tcp_message(self):
    if self.tcp_conn is not None:
      try:
        message = self.tcp_conn.recv(BUFFER_SIZE, socket.MSG_DONTWAIT)  # Non-blocking receive
        if message:
          addr = self.tcp_conn.getpeername()  # Get the IP and port of the connected socket
          print(f"Received TCP message from {addr}: {message.decode('utf-8')}")
          return True
      except socket.error:
        pass  # Non-blocking mode will raise an error if no message is available
    return False

  def streamd_thread(self):
    while True:
      self.rk.monitor_time()

      # Always attempt to send a UDP message
      self.update_and_publish()

      # Accept new TCP connections
      self.accept_new_connection()

      # Send TCP messages periodically
      self.send_tcp_message("Hello, this is a periodic TCP message from KA2")

      # Receive messages without blocking
      self.receive_udp_message()
      self.receive_tcp_message()

      self.rk.keep_time()
      time.sleep(0.0001)  # Prevent tight looping

  def close_connections(self):
    if self.tcp_conn:
      self.tcp_conn.close()
    if self.udp_sock:
      self.udp_sock.close()

def main():
  client_ip = "192.168.100.22"
  streamer = Streamer(client_ip)

  # Setup the UDP and TCP endpoints
  streamer.setup_udp_endpoint()
  streamer.setup_tcp_endpoint()
  streamer.streamd_thread()

if __name__ == "__main__":
  main()

