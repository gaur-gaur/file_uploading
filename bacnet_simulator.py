import time
import random
import socket
from threading import Thread
from bacpypes.app import BIPSimpleApplication
from bacpypes.local.device import LocalDeviceObject
from bacpypes.core import run
from bacpypes.pdu import Address
from bacpypes.primitivedata import Unsigned, Real
from bacpypes.basetypes import ServicesSupported
from bacpypes.object import Property, BinaryValueObject, AnalogValueObject

import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('bacnet_test')

def get_local_ip() -> str:
    """Get the local IP address of the machine.
    
    Returns:
        str: Local IP address with /24 subnet mask or fallback to 0.0.0.0/24
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        logger.info(f"Local IP detected: {local_ip}/24")
        return f"{local_ip}/24"
    except Exception as e:
        logger.error(f"Error getting local IP: {e}")
        return "0.0.0.0/24"

# Define writable BACnet objects that properly handle write operations
class WritableBinaryValueObject(BinaryValueObject):
    def __init__(self, **kwargs):
        from bacpypes.object import BinaryValueObject
        super().__init__(**kwargs)
        self.outOfService = False

    def write_property(self, prop, value, *args, **kwargs):
        print(f"Binary write_property called with {prop}: {value}")
        if prop == 'presentValue':
            self.presentValue = value
            print(f"Binary value updated to: {self.presentValue}")
        else:
            super().write_property(prop, value, *args, **kwargs)


class WritableAnalogValueObject(AnalogValueObject):
    def __init__(self, **kwargs):
        from bacpypes.object import AnalogValueObject
        super().__init__(**kwargs)
        print(f"AnalogValueObject initialized with value: {kwargs.get('presentValue')}")
        self.outOfService = True  # Changed from False to True to allow writes
        # Ensure propertyList is not None
        if self.propertyList is None:
            self.propertyList = []

        # Ensure 'presentValue' is writable
        if 'presentValue' not in self.propertyList:
            self.propertyList.append('presentValue')

    def write_property(self, prop, value, *args, **kwargs):
        print(f"Analog write_property called with {prop}: {value}")
        if prop == 'presentValue':
            print(f"Writing value {value} to {prop}")
            try:
                if isinstance(value, list):  # Handle priority array writes
                    self.presentValue = float(value[0]) if value[0] is not None else self.presentValue
                else:
                    self.presentValue = float(value)
                # Force the value to be updated and persisted
                self._property_changed('presentValue', None, self.presentValue)
                print(f"New value successfully set and persisted: {self.presentValue}")
            except Exception as e:
                print(f"Error updating value: {e}")
        else:
            super().write_property(prop, value, *args, **kwargs)

# Define the BACnet device
class VirtualBACnetDevice:
    def __init__(self, device_id, address):
        # Device configuration
        self.device = LocalDeviceObject(
            objectIdentifier=('device', device_id),
            objectName="VirtualBACnetDevice",
            vendorIdentifier=9999,
            maxApduLengthAccepted=1024,
            segmentationSupported="noSegmentation",
            vendorName="MyVirtualDevice",
            servicesSupported=ServicesSupported().value
        )

        # Create BACnet application
        self.application = BIPSimpleApplication(self.device, Address(address))

        # Add Fan Control Objects
        self.add_objects()

        # Start background thread to send data periodically
        self.running = True
        self.update_thread = Thread(target=self.update_values, daemon=True)
        self.update_thread.start()

    def add_objects(self):
        # Fan Power (On/Off)
        self.fan_power = WritableBinaryValueObject(
            objectIdentifier=('binaryValue', 1),
            objectName='FanPower',
            presentValue=1  # 0 = Off, 1 = On
        )
        
        # Fan Speed (0 - 100%)
        self.fan_speed = WritableAnalogValueObject(
            objectIdentifier=('analogValue', 1),
            objectName='FanSpeed',
            presentValue=71,  # Initial fan speed
            units=98  # Percent
        )
        self.prev_fan_speed = self.fan_speed.presentValue

        # Fan Direction (Forward/Reverse)
        self.fan_direction = WritableBinaryValueObject(
            objectIdentifier=('binaryValue', 2),
            objectName='FanDirection',
            presentValue=1  # 1 = Forward, 0 = Reverse
        )

        # Add objects to the application
        self.application.add_object(self.fan_power)
        self.application.add_object(self.fan_speed)
        self.application.add_object(self.fan_direction)

    def update_values(self):
        """ Periodically update fan speed and print values """
        while self.running:
            # Don't automatically update fan speed - this allows manual control to work
            # Only print current values
            print(f"Fan Power: {'ON' if self.fan_power.presentValue else 'OFF'}")
            print(f"Fan Speed: {self.fan_speed.presentValue}%")
            print(f"Fan Direction: {'Forward' if self.fan_direction.presentValue else 'Reverse'}")
            print("-" * 30)

            time.sleep(3)  # Wait for 3 seconds

    def run(self):
        print("Virtual BACnet Device Running on", self.application.localAddress)
        run()  # Start BACnet stack

# Define BACnet device parameters
DEVICE_ID = 12345  # Change this if needed
# Use the specific IP address that test.py is trying to communicate with
local_ip = get_local_ip()
DEVICE_IP = local_ip  # Fixed IP address to match test.py target
print(f"Using IP: {DEVICE_IP}")

# Create and run the device
if __name__ == "__main__":
    bacnet_device = VirtualBACnetDevice(DEVICE_ID, DEVICE_IP)
    bacnet_device.run()
