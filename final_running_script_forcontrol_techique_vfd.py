from pymodbus.client.sync import ModbusSerialClient

client = ModbusSerialClient(
    method='rtu',
    port='COM38',          # Replace with your actual COM port
    baudrate=9600,
    stopbits=1,
    bytesize=8,
    parity='N',
    timeout=1
)

client.connect()

def write_register(address, value):
    result = client.write_register(address, value, unit=1)
    if result.isError():
        print(f"Failed to write {value} to {hex(address)}: {result}")
    else:
        print(f"Wrote {value} to {hex(address)}")

# Reset fault
write_register(0x0001, 5)

# Set speed to 50% of max
write_register(0x0002, 5000)

# Run forward
write_register(0x0001, 1)

# Stop command (if needed)
#write_register(0x0001, 3)

client.close()
