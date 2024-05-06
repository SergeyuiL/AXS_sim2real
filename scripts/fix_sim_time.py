#!/usr/bin/env python3
from typing import Dict
import rospy
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, Imu
from rosgraph_msgs.msg import Clock

encapsulated_namesapce = "axs_localization/"
time_factor = 1/30
sim_real_time_map_history = 100
timestep_interval_threshold = 1/30

current_timestemp: int = None
current_real_time: rospy.Time = None

def convert_stamp_to_sim(header: Header, no_increment=False):
    global current_timestemp, current_real_time

    time_incremented = False

    if current_timestemp is None:
        current_timestemp = 0
        current_real_time = header.stamp
        time_incremented = True
        print(f"++ Timestep = {current_timestemp}, Real time = {current_real_time.to_sec()}")
    elif not no_increment:
        time_diff = (header.stamp - current_real_time).to_sec()
        if time_diff > timestep_interval_threshold:
            current_timestemp += 1
            current_real_time = header.stamp
            time_incremented = True
            # print(f"++ Timestep = {current_timestemp}, Real time = {current_real_time.to_sec()}, time diff = {time_diff}")
        elif time_diff > 0:
            current_real_time = header.stamp

    if time_incremented:
        clock_msg = Clock()
        clock_msg.clock = rospy.Time(current_timestemp * time_factor)
        encapsulated_clock_pub.publish(clock_msg)

    new_header = Header()
    new_header.stamp = rospy.Time(current_timestemp * time_factor)
    new_header.frame_id = header.frame_id
    return new_header

def convert_stamp_to_real(header: Header):
    global current_timestemp, current_real_time

    new_header = Header()
    new_header.stamp = rospy.Time.now()
    new_header.frame_id = header.frame_id
    return new_header

tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=100)
encapsulated_clock_pub = rospy.Publisher(encapsulated_namesapce + "clock", Clock, queue_size=10, latch=True)
encapsulated_imu_pub = rospy.Publisher(encapsulated_namesapce + "imu", Imu, queue_size=10)
encapsulated_tf_pub = rospy.Publisher(encapsulated_namesapce + "tf", TFMessage, queue_size=100)
encapsulated_tf_static_pub = rospy.Publisher(encapsulated_namesapce + "tf_static", TFMessage, queue_size=100, latch=True)
encapsulated_scan_pub = rospy.Publisher(encapsulated_namesapce + "scan", LaserScan, queue_size=10)

static_tfs: Dict[str, TFMessage] = {}

def is_carto_tf_msg(msg: TFMessage) -> bool:
    for transform in msg.transforms:
        if transform.child_frame_id in ["livox_frame_inv", "odom"]:
            return True
    return False

def imu_callback(msg):
    msg.header = convert_stamp_to_sim(msg.header)
    encapsulated_imu_pub.publish(msg)
    # print("IMU")

def tf_callback(msg):
    if is_carto_tf_msg(msg):
        return
    for transform in msg.transforms:
        transform.header = convert_stamp_to_sim(transform.header)
    encapsulated_tf_pub.publish(msg)
    # print(f"TF: {[t.child_frame_id for t in msg.transforms]}")

def tf_static_callback(msg):
    global static_tfs
    for transform in msg.transforms:
        transform.header = convert_stamp_to_sim(transform.header, no_increment=True)
        static_tfs[transform.child_frame_id] = transform
    new_msg = TFMessage()
    new_msg.transforms = [t for t in static_tfs.values()]
    encapsulated_tf_static_pub.publish(new_msg)
    # print(f"Static TF: {[t.child_frame_id for t in msg.transforms]}")

def encapsulated_tf_callback(msg):
    if not is_carto_tf_msg(msg):
        return
    for transform in msg.transforms:
        transform.header = convert_stamp_to_real(transform.header)
    tf_pub.publish(msg)

def scan_callback(msg):
    msg.header = convert_stamp_to_sim(msg.header, no_increment=True)
    encapsulated_scan_pub.publish(msg)
    # print("Scan")

rospy.init_node("fix_sim_time")
imu_sub = rospy.Subscriber("/livox/imu_inv", Imu, imu_callback)
tf_sub = rospy.Subscriber("/tf", TFMessage, tf_callback)
tf_static_sub = rospy.Subscriber("/tf_static", TFMessage, tf_static_callback)
encapsulated_tf_sub = rospy.Subscriber(encapsulated_namesapce + "tf", TFMessage, encapsulated_tf_callback)
scan_sub = rospy.Subscriber("/scan", LaserScan, scan_callback)
rospy.spin()