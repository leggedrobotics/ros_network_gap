#!/usr/bin/env python
import rospy
import cPickle
import socket
import importlib
import UDPMulticaster


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
        self.udp_sender = UDPMulticaster.UdpMulticaster(broadcast, port)
        self.udp_receiver = UDPMulticaster.UdpMulticasterReceiver(port, self.udp_callback)


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

    broadcast = str(rospy.get_param("~broadcast", "10.10.50.255"))
    port = int(rospy.get_param("~port", 14317))
    inputtopic = "input_topic"
    exchangetopic = "network_gap_topic"
    msg_pkg = str(rospy.get_param("~msg_pkg", "nav_msgs.msg"))
    msg_type = str(rospy.get_param("~msg_type", "Odometry"))
    max_rate = float(rospy.get_param("~max_rate", 5))

    rospy.loginfo("Starting ros_network_gap with message " + msg_pkg + "/"+msg_type + " on " + broadcast + ":" + str(port))
    node = RosNetworkGap(broadcast, port, inputtopic, exchangetopic, msg_pkg, msg_type, max_rate)
    rospy.spin()
