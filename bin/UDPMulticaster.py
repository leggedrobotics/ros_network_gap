
from socket import *
from collections import deque
from threading import Thread
import rospy

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