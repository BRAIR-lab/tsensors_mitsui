#!/usr/bin/env python3

import rospy
import serial
import numpy as np
import json
from geometry_msgs.msg import WrenchStamped

class tsensors:
    def __init__(self,id_ports,rate):

        self.num_of_sensors = np.shape(id_ports)[0]
        ports = np.empty(self.num_of_sensors,object)
        self.ser = np.empty(self.num_of_sensors,object)
        for p in range(self.num_of_sensors):
            port_name = '/dev/ttyACM' + str(id_ports[p])
            self.ser[p] = serial.Serial(port_name)
            self.ser[p].baudrate = 230400
            self.ser[p].write(bytes("020501", 'utf-8'))  #Reset offset
            self.ser[p].write(bytes("020202", 'utf-8'))  #Start sending

        self.offset = 2048

        #ROS publisher
        self.tsensor_pubs = np.empty(self.num_of_sensors,object)
        for p in range(self.num_of_sensors):
            tpcname_pub = '/tsensors/sensor_' + str(p+1)
            self.tsensor_pubs[p] = rospy.Publisher(tpcname_pub, WrenchStamped, queue_size=1)

        self.pub_rate = rospy.Rate(rate)
        self.publish()

    def publish(self):
        sensor_data = np.empty(self.num_of_sensors,object)
        for p in range(self.num_of_sensors):
            sensor_data[p] = WrenchStamped()
            sensor_data[p].header.seq = 0
            sensor_data[p].header.frame_id = 'sensor_' + str(p+1)

        rospy.loginfo("Publishing Sensor Data...")
        while not rospy.is_shutdown():

            for p in range(self.num_of_sensors):
                input_str = self.ser[p].readline().decode("utf-8")

                sensor_data[p].header.seq += 1
                sensor_data[p].header.stamp = rospy.Time.now()

                sensor_data[p].wrench.force.x = (int(input_str[4:8], 16)-self.offset)*0.01
                sensor_data[p].wrench.force.y = (int(input_str[8:12], 16)-self.offset)*0.01
                sensor_data[p].wrench.force.z = (int(input_str[12:16], 16)-self.offset)*0.02
                sensor_data[p].wrench.torque.x = (int(input_str[16:20], 16)-self.offset)*0.05
                sensor_data[p].wrench.torque.y = (int(input_str[20:24], 16)-self.offset)*0.05
                sensor_data[p].wrench.torque.z = (int(input_str[24:28], 16)-self.offset)*0.05
 
                self.tsensor_pubs[p].publish(sensor_data[p])

            self.pub_rate.sleep()

    def turn_off(self):

        for p in range(self.num_of_sensors):
            self.ser[p].write(bytes("020200", 'utf-8'))

def tsensors_py():
    # Starts a new node
    rospy.init_node('tactile_sensors_node', anonymous=True)

    # Debug
    rospy.loginfo("********* TSensor Node Interface ***********")

    id_ports = json.loads(rospy.get_param("~id_ports"))
    rate = rospy.get_param("mitsui_rate")
    
    #Create _ros_template_ object
    rospy.loginfo("Create tsensor object.")
    _tsensors_ = tsensors(id_ports,rate)
    
    #Handle shutdown
    rospy.on_shutdown(_tsensors_.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        tsensors_py()
    except rospy.ROSInterruptException:
        pass
