#!/usr/bin/env python3

import rospy
import serial
import time
from LoadCellPublisher.msg import LoadCellArray  # Replace with your actual package name
from std_msgs.msg import Header

def main():
    rospy.init_node('load_cell_publisher', anonymous=True)
    rospy.loginfo("Starting Load Cell Publisher Node")
    pub = rospy.Publisher('load_cell_array', LoadCellArray, queue_size=10)
    try:
        ser = serial.Serial('/dev/esp32device', 115200, timeout=1)
        ser.reset_input_buffer()
        rospy.loginfo("Serial port opened successfully")
        time.sleep(2)
    except serial.SerialException as e:
        rospy.logerr(f"Serial port error: {e}")
        return

    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            if "rst" in line.lower():
                rospy.loginfo("Reset detected, flushing input buffer")
                ser.reset_input_buffer()
                continue
            
            parts = line.split()
            if len(parts) == 3:
                try:
                    values = [float(p) for p in parts]
                    msg = LoadCellArray()
                    msg.header.stamp = rospy.Time.now()
                    msg.load_cell_0 = values[0]
                    msg.load_cell_1 = values[1]
                    msg.load_cell_2 = values[2]
                    pub.publish(msg)
                except ValueError as e:
                    rospy.logwarn(f"Non-numeric data received: {line} — {e}")
        except Exception as e:
            rospy.logwarn(f"Parse error: {line} — {e}")
            ser.reset_input_buffer()

if __name__ == '__main__':
    main()
