#!/usr/bin/env python3

import rospy
import serial
import time
from LoadCellPublisher.msg import LoadCellArray  # Replace with your actual package name
from std_msgs.msg import Header

def main():
    rospy.loginfo("Starting Load Cell Publisher Node")
    rospy.init_node('load_cell_publisher')
    pub = rospy.Publisher('load_cell_array', LoadCellArray, queue_size=10)

    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    time.sleep(2)

    while not rospy.is_shutdown():
        line = ser.readline().decode().strip()
        try:
            parts = line.split()
            if len(parts) == 3:
                values = [float(p) for p in parts]
                msg = LoadCellArray()
                msg.header.stamp = rospy.Time.now()
                msg.load_cell_0 = values[0]
                msg.load_cell_1 = values[1]
                msg.load_cell_2 = values[2]
                pub.publish(msg)
        except Exception as e:
            rospy.logwarn(f"Parse error: {line} — {e}")

if __name__ == '__main__':
    main()
