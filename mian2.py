import BAC0
import time
import os
import sys
import paho.mqtt.client as mqtt
from random import randrange, uniform
import uuid


def blockPrint():
    sys.stdout = open(os.devnull, 'w')


def enablePrint():
    sys.stdout = sys.__stdout__


def get_bac0_device(device, network):
    print("get bac0 device")
    name, vendor, address, device_id = device
    return BAC0.device(address, device_id, network, poll=0, object_list=None)


# MQTT setup
# Local or Azure MQTT broker address

mqttBroker = "otplcloud.com"
port = 1883
cid = str(uuid.uuid4())
client = mqtt.Client(client_id="Gateway_Device_" + cid)

try:
    client.connect(mqttBroker, port)
    print("Connected to MQTT Broker")
except Exception as e:
    print(f"Failed to connect to MQTT Broker: {e}")
    sys.exit(1)


bacnet = BAC0.connect()
bacnet.discover()
device = bacnet.devices[0]
bac0_device = get_bac0_device(device, bacnet)


def main():
    while True:
        points = bac0_device.points
        blockPrint()
        print(points)
        enablePrint()

        for idx, point in enumerate(points):
            msg = str(point.lastValue)
            topic = "{}".format(point.properties.name)
            try:
                client.publish(topic, msg)
                print(f"Published {msg} to topic {topic}")
            except Exception as e:
                print(f"Failed to publish message: {e}")

        time.sleep(4)


if __name__ == "__main__":
    main()
