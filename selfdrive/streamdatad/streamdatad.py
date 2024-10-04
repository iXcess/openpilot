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
    self.ip = client_ip # ip address of the udp client, TODO, automate it
    self.port = 5006
    self.sock = None

    # Setup subscriber
    self.sm = sm
    if self.sm is None:
      self.sm = messaging.SubMaster(['navInstruction'])

    # Sending data at 10hz
    self.rk = Ratekeeper(10, print_delay_threshold=None)

  def setup_udp_endpoint(self):
    try:
      self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      self.sock.bind((self.local_ip, self.port))
      print(f"UDP endpoint set up at {self.local_ip}:{self.port}")
    except socket.error as e:
      print(f"Failed to set up UDP endpoint: {e}")
      self.sock = None

  def is_connected(self):
    # Since UDP is connectionless, check if the socket is created and bound
    return self.sock is not None

  def update_and_publish(self):
    if self.is_connected():
      self.sm.update()
      self.sock.sendto(self.sm['navInstruction'].as_builder().to_bytes(), (self.ip, self.port))

  def send_udp_message(self, message):
      if self.is_connected():
            self.sock.sendto(message.encode('utf-8'), (self.ip, self.port))

  def streamd_thread(self):
    while True:
      self.rk.monitor_time()
      self.update_and_publish()
      self.send_udp_message("Hello, this is a periodic UDP message from KA2")
      self.rk.keep_time()

def main():
  #streamer = Streamer(sys.argv[1])
  client_ip = "192.168.100.24"
  streamer = Streamer(client_ip)

  # check for hotspot on, then setup the udp endpoint
  streamer.setup_udp_endpoint()
  streamer.streamd_thread()

if __name__ == "__main__":
  main()
