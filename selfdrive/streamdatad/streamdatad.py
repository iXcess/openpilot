import subprocess
import time

class BLE:
  def __init__(self):
    self.device_name = None
    self.adapter = self.get_adapter()

  def run_command(self, cmd):
    """Run a shell command and return the output."""
    process = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return process.stdout.strip(), process.stderr.strip()

  def get_adapter(self):
    """Get the Bluetooth adapter (e.g., hci0)."""
    stdout, _ = self.run_command("sudo hciconfig")
    return stdout.splitlines()[0].split(":")[0] if stdout else "hci0"

  def get_mac_address(self):
    """Retrieve the MAC address of the adapter."""
    stdout, _ = self.run_command(f"sudo hciconfig {self.adapter} | grep 'BD Address'")
    return stdout.split(" ")[2] if stdout else None

  def set_device_alias(self):
    """Set the Bluetooth device name with the last 4 MAC characters."""
    mac_address = self.get_mac_address()
    if mac_address:
      last_4_mac = mac_address.replace(":", "")[-4:].upper()
      self.device_name = f"KommuAssist {last_4_mac}"  # Set the device name dynamically
      self.run_command(f"sudo hciconfig {self.adapter} name '{self.device_name}'")
      print(f"Device alias set to: {self.device_name}")
    else:
      print("Failed to retrieve MAC address.")

  def clear_paired_devices(self):
    """Clear all paired devices."""
    stdout, _ = self.run_command("sudo bluetoothctl paired-devices")
    devices = stdout.splitlines()
    for device in devices:
      if device.strip():  # Ensure the line isn't empty
        device_mac = device.split(" ")[1]  # The second part of the line is the MAC address
        self.run_command(f"sudo bluetoothctl remove {device_mac}")

  def configure_bluetoothctl(self):
    """Configure Bluetooth to automatically accept pairings and start advertising."""
    commands = [
      "sudo systemctl restart bluetooth",  # Restart Bluetooth service
      "sudo bluetoothctl power on",
      "sudo bluetoothctl agent NoInputNoOutput",  # Automatically accepts pairing requests without a PIN
      "sudo bluetoothctl default-agent",
      "sudo bluetoothctl discoverable on",
      "sudo bluetoothctl pairable on",
      "sudo bluetoothctl advertise on",  # Start advertising
    ]
    for cmd in commands:
      self.run_command(cmd)

  def start_ble(self):
    """Run all setup steps to make BLE pairing work without a PIN."""
    self.clear_paired_devices()  # Clear all paired devices before starting
    self.set_device_alias()  # Ensure the name is set before starting
    self.configure_bluetoothctl()  # Configure Bluetooth

    while True:
      time.sleep(10)

def main():
  ble_setup = BLE()
  ble_setup.start_ble()

if __name__ == "__main__":
  main()

