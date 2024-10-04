# import pickle
# import json
# import sys
import socket
import time

from openpilot.common.realtime import Ratekeeper
import cereal.messaging as messaging

def get_wlan_ip():
  # Connect to an external server (e.g., Google's public DNS server)
  with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    # It doesn't actually send any data
    s.connect(("8.8.8.8", 80))
    return s.getsockname()[0]  # Return the local IP address

class Streamer:
  def __init__(self, client_ip, sm=None):
    # UDP and TCP sockets
    self.local_ip = get_wlan_ip()
    self.ip = client_ip  # IP address of the client
    self.udp_port = 5006
    self.tcp_port = 5007
    self.udp_sock = None
    self.tcp_sock = None
    self.tcp_conn = None  # To store the TCP connection

    # Setup subscriber
    self.sm = sm if sm else messaging.SubMaster(['navInstruction'])  # Conditional initialization

    # Sending data at 10Hz
    self.rk = Ratekeeper(10, print_delay_threshold=None)

  def setup_udp_endpoint(self):
    try:
      self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      self.udp_sock.bind((self.local_ip, self.udp_port))
      print(f"UDP endpoint set up at {self.local_ip}:{self.udp_port}")
    except socket.error as e:
      print(f"Failed to set up UDP endpoint: {e}")
      self.udp_sock = None

  def setup_tcp_endpoint(self):
    try:
      self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      self.tcp_sock.bind((self.local_ip, self.tcp_port))
      self.tcp_sock.listen(1)
      self.tcp_sock.setblocking(False)  # Set TCP socket to non-blocking
      print(f"TCP endpoint set up at {self.local_ip}:{self.tcp_port}")
    except socket.error as e:
      print(f"Failed to set up TCP endpoint: {e}")
      self.tcp_sock = None

  def is_udp_connected(self):
    return self.udp_sock is not None

  def is_tcp_connected(self):
    return self.tcp_conn is not None

  def update_and_publish(self):
    if self.is_udp_connected():
      self.sm.update()  # Update the message

      # message = self.sm['navInstruction'].as_builder().to_bytes()  # Original message retrieval
      message = "Hello, this is a periodic UDP message from KA2"  # Fixed message to send

      if message:
        self.udp_sock.sendto(message.encode('utf-8'), (self.ip, self.udp_port))

  def send_tcp_message(self, message):
    if self.is_tcp_connected():
      try:
        self.tcp_conn.sendall(message.encode('utf-8'))
      except socket.error:
        self.tcp_conn = None  # Mark connection as broken
      except Exception as e:
        print(f"Unexpected error: {e}")

  def accept_new_connection(self):
    if self.tcp_conn is None:
      try:
        self.tcp_conn, addr = self.tcp_sock.accept()
      except socket.error:
        # Non-blocking mode will raise an error if no connection is available
        pass

  def streamd_thread(self):
    while True:
      self.rk.monitor_time()
      self.update_and_publish()
      self.accept_new_connection()
      self.send_tcp_message("Hello, this is a periodic TCP message from KA2")
      self.rk.keep_time()
      time.sleep(0.01)  # Prevent tight looping

  def close_connections(self):
    if self.tcp_conn:
      self.tcp_conn.close()
      print("TCP connection closed.")
    if self.udp_sock:
      self.udp_sock.close()
      print("UDP socket closed.")

def main():
  # streamer = Streamer(sys.argv[1])
  client_ip = "192.168.100.24"
  streamer = Streamer(client_ip)

  # Setup the UDP and TCP endpoints
  streamer.setup_udp_endpoint()
  streamer.setup_tcp_endpoint()
  streamer.streamd_thread()

if __name__ == "__main__":
  main()

