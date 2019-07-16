#!/usr/bin/env python

import rospy
import cPickle
import socket
import importlib

from socket import *
from collections import deque
from threading import Thread


class UdpIdentifier:
    @staticmethod
    def getId():
        return "MAGIC" + gethostname() + rospy.get_name()

    @staticmethod
    def isOwnId(value):
        return value == UdpIdentifier.getId()

    @staticmethod
    def isId(value):
        return value.startswith("MAGIC")


class UdpMulticaster:
    def __init__(self, broadcast, port):
        self.multicast_address = broadcast
        self.port = port
        self.socket = socket(AF_INET, SOCK_DGRAM)
        self.socket.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

    def sendIdentifierPacket(self):
        self.socket.sendto(UdpIdentifier.getId(), (self.multicast_address, self.port))

    def sendPacket(self, data):
        self.socket.sendto(data, (self.multicast_address, self.port))


class UdpMulticasterReceiver:
    def __init__(self, port, callback):
        self.buf = deque()
        self.port = port
        self.socket = socket(AF_INET, SOCK_DGRAM)
        self.socket.settimeout(5.0)
        self.run = False
        self.own_addr = 0
        self.callback = callback

    def start(self):
        self.socket.bind(("", self.port))
        self.thread = Thread(target=self.receive)
        self.thread.start()

    def receive(self):
        while not rospy.is_shutdown():
            try:
                data, addr = self.socket.recvfrom(4096)
                if (UdpIdentifier.isOwnId(data)):
                    rospy.loginfo("Received own udp identification " + str(addr))
                    self.own_addr = str(addr)

                if (str(addr) != self.own_addr and not UdpIdentifier.isId(data)):
                    self.callback(data)
            except timeout:
                pass


class RosNetworkGap:
    def __init__(self, broadcast, port, inputtopic, exchangetopic, msg_pkg, msg_type, max_rate):
        self.port = int(port)
        self.inputtopic = str(inputtopic)
        self.exchangetopic = str(exchangetopic)
        self.broadcast = str(broadcast)

        self.msg_pkg = str(msg_pkg)
        self.msg_type = str(msg_type)
        self.max_rate = rospy.Rate(max_rate)

        #get class type
        try:
            self.import_module = importlib.import_module(self.msg_pkg)
            self.msg_class = getattr(self.import_module, self.msg_type)
        except:
            rospy.logfatal("Could not load type/module " + self.msg_pkg + " / "+ self.msg_type)
            raise

        # set up ros stuff
        self.ros_publisher = rospy.Publisher(self.exchangetopic,  self.msg_class, queue_size=10)
        self.ros_subscriber =  rospy.Subscriber(self.inputtopic, self.msg_class, self.ros_callback, queue_size=1)

        # set up UDP stuff
        self.udp_sender = UdpMulticaster(broadcast, port)
        self.udp_receiver = UdpMulticasterReceiver(port, self.udp_callback)


        #start receiver
        self.udp_receiver.start()

        # send identifier packet for UDP
        self.udp_sender.sendIdentifierPacket()

    def udp_callback(self, data):
        # callback received, deserialize data and check if of right object type
        obj = cPickle.loads(data);
        if(isinstance(obj, self.msg_class)):
            self.ros_publisher.publish(obj)

    def ros_callback(self, data):
        # not our own message, then execute callback
        if  not data._connection_header["callerid"] == rospy.get_caller_id():
            self.udp_sender.sendPacket(cPickle.dumps(data))

            # wait to be inside throttling frequency
            self.max_rate.sleep()


if __name__ == '__main__':
    rospy.init_node('ros_network_gap', anonymous=True)

    broadcast = str(rospy.get_param("~listener_ip", "10.10.50.255"))
    port = int(rospy.get_param("~listener_port", 14317))
    inputtopic = str(rospy.get_param("~subscriber_topic", "subscriber_topic"))
    exchangetopic = str(rospy.get_param("~publisher_topic", "publisher_topic"))
    msg_pkg = str(rospy.get_param("~msg_pkg", "nav_msgs.msg"))
    msg_type = str(rospy.get_param("~msg_type", "Odometry"))
    max_rate = float(rospy.get_param("~max_publish_rate", 5))

    rospy.loginfo("Starting ros_network_gap with message " + msg_pkg + "/"+msg_type + " on " + broadcast + ":" + str(port))
    node = RosNetworkGap(broadcast, port, inputtopic, exchangetopic, msg_pkg, msg_type, max_rate)
    rospy.spin()
