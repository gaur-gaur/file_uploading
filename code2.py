import BAC0
import time
import os, sys
import paho.mqtt.client as mqtt
from random import randrange, uniform
import uuid


def blockPrint():
    sys.stdout = open(os.devnull, 'w')


def enablePrint():
    sys.stdout = sys.__stdout__




def get_bac0_device(device,network):
    print("get bac0 device")
    name,vender,address,device_id=device
    return BAC0.device(address,device_id,network,poll=0,object_list=None)


#mqtt setup
#local or azure mqtt broker address

mqttBroker =  "otplcloud.com"
port =1883
cid = str(uuid.uuid4())
client = mqtt.Client("Gateway_Device_"+cid)


client.connect(mqttBroker, port)




bacnet = BAC0.connect()
bacnet.discover()
device = bacnet.device[0]
bac0_device =get_bac0_device(device,bacnet)



def main():
    while Ture:
        points = bac0_device.points
        blockPrint()
        print(points)
        enablePrint()

        for idx, point in enumerate (points):
            msg = str(point.lastValue)
            topic = "{}".format(point.properties.name)
            client.publish(topic,msg)

        tiem.sleep(4)


if __name__ == "__main__":
    main()
    
