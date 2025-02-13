from pydbus import SystemBus
from gi.repository import GLib
from dbus.mainloop.glib import DBusGMainLoop

class BLE:
    def __init__(self):
        self.device_name = None

    def get_mac_address(self):
        # Fetch the MAC address of the first available Bluetooth adapter
        bus = SystemBus()
        adapter = bus.get("org.bluez", "/org/bluez/hci0")
        # MAC address is part of the Adapter1 interface properties
        return adapter.Address

    def make_ble_discoverable(self):
        if self.device_name is None:
            # Get the MAC address and format the name
            mac_address = self.get_mac_address()
            last_4_mac = mac_address.replace(":", "")[-4:].upper()  # Last 4 characters (no colons)
            self.device_name = f"KommuAssist {last_4_mac}"

        # Get the Bluetooth adapter
        bus = SystemBus()
        adapter = bus.get("org.bluez", "/org/bluez/hci0")

        # Enable Bluetooth adapter and set the properties
        adapter.Powered = True
        adapter.Discoverable = True
        adapter.Pairable = True
        adapter.AuthRequired = False  # Disable authentication for pairing
        adapter.Alias = self.device_name  # Use the formatted name

        # Get the LE Advertising Manager and register the advertisement
        ad_manager = bus.get("org.bluez", "/org/bluez/hci0")
        try:
            ad_manager.RegisterAdvertisement("/org/bluez/advertisement", {})
            print("BLE advertising started")
        except Exception as e:
            print("Failed to start BLE advertising:", e)

def main():
    # Create a BLE instance
    ble_device = BLE()

    # Start BLE advertising with dynamic name based on MAC address
    ble_device.make_ble_discoverable()

    # Main loop to keep the script running
    DBusGMainLoop(set_as_default=True)
    loop = GLib.MainLoop()
    print(f"Bluetooth device '{ble_device.device_name}' is discoverable as a BLE peripheral...")
    loop.run()

if __name__ == "__main__":
    main()

