# import pickle
# import json
# import sys
import socket

from openpilot.common.realtime import Ratekeeper
import cereal.messaging as messaging

def get_wlan_ip():
  # Connect to an external server (e.g., Google's public DNS server)
  with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    # It doesn't actually send any data
    s.connect(("8.8.8.8", 80))
    wlan_ip = s.getsockname()[0]
  return wlan_ip

class Streamer:
  def __init__(self, client_ip, sm=None):
    # UDP sockets
    self.local_ip = get_wlan_ip()
    self.ip = client_ip  # IP address of the UDP client, TODO, automate it
    self.udp_port = 5006
    self.tcp_port = 5007
    self.udp_sock = None
    self.tcp_sock = None

    # Setup subscriber
    self.sm = sm
    if self.sm is None:
      self.sm = messaging.SubMaster(['navInstruction'])

    # Sending data at 10hz
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
      print(f"TCP endpoint set up at {self.local_ip}:{self.tcp_port}")
    except socket.error as e:
      print(f"Failed to set up TCP endpoint: {e}")
      self.tcp_sock = None

  def is_udp_connected(self):
    # Since UDP is connectionless, check if the socket is created and bound
    return self.udp_sock is not None

  def is_tcp_connected(self):
    # Check if the TCP socket is created and bound
    return self.tcp_sock is not None

  def update_and_publish(self):
    if self.is_udp_connected():
      self.sm.update()
      self.udp_sock.sendto(self.sm['navInstruction'].as_builder().to_bytes(), (self.ip, self.udp_port))

  def send_udp_message(self, message):
    if self.is_udp_connected():
      self.udp_sock.sendto(message.encode('utf-8'), (self.ip, self.udp_port))

  def send_tcp_message(self, message):
    if self.is_tcp_connected():
      try:
        # Accept a TCP connection; this will block until a client connects
        conn, addr = self.tcp_sock.accept()
        with conn:
          conn.sendall(message.encode('utf-8'))
          print(f"Sent TCP message to {addr}")
      except socket.error as e:
        print(f"Error sending TCP message: {e}")
      except Exception as e:
        print(f"Unexpected error: {e}")
    else:
      print("TCP socket is not connected.")

  def streamd_thread(self):
    while True:
      self.rk.monitor_time()
      self.update_and_publish()
      self.send_udp_message("Hello, this is a periodic UDP message from KA2")
      self.send_tcp_message("Hello, this is a periodic TCP message from KA2")
      self.rk.keep_time()

def main():
  # streamer = Streamer(sys.argv[1])
  client_ip = "192.168.100.24"
  streamer = Streamer(client_ip)

  # Check for hotspot on, then setup the UDP and TCP endpoints
  streamer.setup_udp_endpoint()
  streamer.setup_tcp_endpoint()
  streamer.streamd_thread()

if __name__ == "__main__":
  main()

