import sys
# sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
import os
import os.path as osp
import argparse
import json
import numpy as np
from math import radians
from tqdm import tqdm
from pypcd import pypcd
from utils import *
# ros
import rospy
import rosbag
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Imu, NavSatFix, PointField


def parse_imu_info(can_list):
    for can_file in can_list:
        if '.txt' in can_file:
            imu_info = read_gpsimu_from_txt(can_file)
            break
    return imu_info

def write_imu_msg(bag, imu_info, imu_topic, gps_fix_topic, gps_vel_topic, frame_id):
    print('Writing IMU msgs ...')
    for info in tqdm(imu_info):
        # IMU
        yaw     = radians(info['HeadingPitchRoll'][0])
        pitch   = radians(info['HeadingPitchRoll'][1])
        roll    = radians(info['HeadingPitchRoll'][2])
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        imu = Imu()
        imu.header.frame_id = frame_id
        imu.header.stamp = rospy.Time.from_sec(info['timestamp'])
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.linear_acceleration.x = info['Accel_Vehicle'][0]
        imu.linear_acceleration.y = info['Accel_Vehicle'][1]
        imu.linear_acceleration.z = info['Accel_Vehicle'][2]
        imu.angular_velocity.x = radians(info['AngRateVehicle'][0])
        imu.angular_velocity.y = radians(info['AngRateVehicle'][1])
        imu.angular_velocity.z = radians(info['AngRateVehicle'][2])
        bag.write(imu_topic, imu, imu.header.stamp)

        # GPS
        gps = NavSatFix()
        gps.header.frame_id = frame_id
        gps.header.stamp = rospy.Time.from_sec(info['timestamp'])
        gps.latitude  = info['Latitude']
        gps.longitude = info['Longitude']
        gps.altitude  = info['Altitude']
        gps.status.service = 1
        bag.write(gps_fix_topic, gps, t=gps.header.stamp)

        # Velocity
        vel = TwistStamped()
        vel.header.frame_id = frame_id
        vel.header.stamp = rospy.Time.from_sec(info['timestamp'])
        vel.twist.linear.x = info['VelocityLevel'][0]
        vel.twist.linear.y = info['VelocityLevel'][1]
        vel.twist.linear.z = info['VelocityLevel'][2]
        vel.twist.angular = imu.angular_velocity
        bag.write(gps_vel_topic, vel, t=vel.header.stamp)

def write_lidar_msg(bag, pcd_list, time_list, topic, frame_id):
    print('Writing Lidar msgs ...')
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.UINT16, count=1),
        PointField(name='ring', offset=14, datatype=PointField.UINT16, count=1),
        PointField(name='timestamp', offset=16, datatype=PointField.FLOAT64, count=1)
    ]
    for i in tqdm(range(len(pcd_list))):
        if osp.getsize(pcd_list[i]) < 0.1:
            print('Warning: {} is empty!'.format(pcd_list[i]))
            continue
        pcd = pypcd.PointCloud.from_path(pcd_list[i])
        points = pcd.pc_data[['x', 'y', 'z', 'intensity', 'ring', 'timestamp']]
        header = Header()
        header.seq = i
        header.stamp = rospy.Time.from_sec(time_list[i])
        header.frame_id = frame_id
        pc2 = point_cloud2.create_cloud(header, fields, points)
        bag.write(topic, pc2, header.stamp)
        

def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('data_path', type=str, default='', 
                        help='specify the data path including LiDAR and CAN')
    args = parser.parse_args()
    return args


def bst2bag():
    data_path = args.data_path
    session = data_path.split('/')[-2] if data_path[-1] == '/' else data_path.split('/')[-1]
    with open(osp.join(data_path, f'{session}.json')) as f:
        calib = json.load(f)
    time_list = calib['timestamp']
    pcd_list = read_file_list(data_path + '/lidar', 'pcd')
    can_list = read_file_list(data_path, 'can')
    imu_info = parse_imu_info(can_list)

    assert len(time_list) == len(pcd_list)

    print(f'data_path: {data_path}')
    print(f'{session} contains {len(pcd_list)} pcd files')
    print(f'{session} contains {len(imu_info)} imu infos')

    tic = rospy.Time.now()
    bag = rosbag.Bag(osp.join(data_path, f'{session}.bag'), 'w')
    print(f'Start Converting {session}.bag')

    write_imu_msg(bag, imu_info, imu_topic, gps_fix_topic, gps_vel_topic, imu_frame_id)
    write_lidar_msg(bag, pcd_list, time_list, lidar_topic, lidar_frame_id)

    bag.close()


if __name__ == '__main__':
    # Usage:
    #   python bst2bag/bst2bag.py /workspace/work/data/output_20230109/data/6v_img_lidar_20230109/20230109_084712

    args = parse_config()
    lidar_topic = '/points_raw'
    lidar_frame_id = 'velodyne'
    imu_topic = '/imu_raw'
    imu_frame_id = 'imu_link'
    gps_fix_topic = '/gps/fix'
    gps_fix_correct_topic = '/gps/fix/correct'
    gps_vel_topic = '/gps/vel'
    rospy.init_node('bst2bag', anonymous=True)

    bst2bag()
    print('Done!')
