#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from livox_ros_driver.msg import CustomMsg, CustomPoint
from std_msgs.msg import Header

def pointcloud2_to_custommsg(point_cloud2_msg):
    # 创建 CustomMsg
    custom_msg = CustomMsg()
    custom_msg.header = point_cloud2_msg.header  # 复制头信息
    custom_msg.timebase = rospy.Time.now().to_nsec()  # 使用当前时间作为时间基准
    custom_msg.point_num = len(list(pc2.read_points(point_cloud2_msg)))  # 计算点的数量
    custom_msg.lidar_id = 1  # 假设只有一个LiDAR，ID设为1
    custom_msg.rsvd = [0, 0, 0]  # 填充保留字段

    # 遍历PointCloud2中的每个点
    for point in pc2.read_points(point_cloud2_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
        custom_point = CustomPoint()
        custom_point.x = point[0]
        custom_point.y = point[1]
        custom_point.z = point[2]
        custom_point.reflectivity = int(point[3]) if len(point) > 3 else 0
        custom_point.tag = 0
        custom_point.line = 0
        custom_msg.points.append(custom_point)

    return custom_msg

def callback(data):
    # 调用转换函数
    custom_msg = pointcloud2_to_custommsg(data)
    # 发布CustomMsg
    pub.publish(custom_msg)

if __name__ == '__main__':
    rospy.init_node('pcl2_to_custom_converter')
    input_topic = rospy.get_param('~input_topic', '/livox/lidar')
    output_topic = rospy.get_param('~output_topic', '/livox/cusMsg')
    pub = rospy.Publisher(output_topic, CustomMsg, queue_size=10)  # 创建CustomMsg发布器
    rospy.Subscriber(input_topic, PointCloud2, callback)  # 订阅PointCloud2消息
    rospy.spin()
