#!/usr/bin/env python

import rospy
import serial
import json
from geometry_msgs.msg import WrenchStamped

class TSensors:
    def __init__(self, id_ports, rate):
        self.offset = 2048
        self.num_sensors = len(id_ports)
        self.ser_ports = []
        self.publishers = []

        for idx, port_id in enumerate(id_ports):
            try:
                port = serial.Serial(f'/dev/ttyACM{port_id}', baudrate=230400)
                port.write(b"020501")  # Reset offset
                port.write(b"020202")  # Start sending
                self.ser_ports.append(port)
                rospy.loginfo(f"Initialized sensor on /dev/ttyACM{port_id}")
            except serial.SerialException as e:
                rospy.logerr(f"Failed to connect to /dev/ttyACM{port_id}: {e}")
                raise

            topic_name = f'/tsensors/sensor_{idx + 1}'
            self.publishers.append(rospy.Publisher(topic_name, WrenchStamped, queue_size=1))

        self.pub_rate = rospy.Rate(rate)
        self.publish_loop()

    def publish_loop(self):
        rospy.loginfo("Publishing Sensor Data...")
        seq_counters = [0] * self.num_sensors

        while not rospy.is_shutdown():
            for idx, ser in enumerate(self.ser_ports):
                try:
                    line = ser.readline().decode("utf-8")
                    msg = self.parse_sensor_data(line, idx + 1, seq_counters[idx])
                    self.publishers[idx].publish(msg)
                    seq_counters[idx] += 1
                except Exception as e:
                    rospy.logwarn(f"Error parsing sensor {idx + 1}: {e}")
            self.pub_rate.sleep()

    def parse_sensor_data(self, data_str, sensor_id, seq):
        msg = WrenchStamped()
        msg.header.seq = seq
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = f'sensor_{sensor_id}'

        try:
            msg.wrench.force.x = (int(data_str[4:8], 16) - self.offset) * 0.01
            msg.wrench.force.y = (int(data_str[8:12], 16) - self.offset) * 0.01
            msg.wrench.force.z = (int(data_str[12:16], 16) - self.offset) * 0.02
            msg.wrench.torque.x = (int(data_str[16:20], 16) - self.offset) * 0.05
            msg.wrench.torque.y = (int(data_str[20:24], 16) - self.offset) * 0.05
            msg.wrench.torque.z = (int(data_str[24:28], 16) - self.offset) * 0.05
        except (ValueError, IndexError) as e:
            raise ValueError(f"Failed to parse sensor data: '{data_str.strip()}'. Error: {e}")
        
        return msg

    def turn_off(self):
        for ser in self.ser_ports:
            try:
                ser.write(b"020200")
            except Exception as e:
                rospy.logwarn(f"Failed to send turn off command: {e}")

def main():
    rospy.init_node('tactile_sensors_node', anonymous=True)
    rospy.loginfo("********* TSensor Node Interface ***********")

    try:
        id_ports = json.loads(rospy.get_param("~id_ports"))
        rate = rospy.get_param("mitsui_rate")
    except KeyError as e:
        rospy.logerr(f"Missing parameter: {e}")
        return

    try:
        sensor_node = TSensors(id_ports, rate)
        rospy.on_shutdown(sensor_node.turn_off)
        rospy.spin()
    except Exception as e:
        rospy.logfatal(f"TSensors initialization failed: {e}")

if __name__ == '__main__':
    main()