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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Imu, NavSatFix, PointField


lidar_frame_id = 'velodyne'
imu_frame_id = 'imu_link'
gps_frame_id = 'navsat_link'

lidar_topic = '/points_raw'
imu_raw_topic = '/imu_raw'
imu_correct_topic = '/imu_correct'
gps_fix_topic = '/gps/fix'
gps_fix_correct_topic = '/gps/fix/correct'
gps_vel_topic = '/gps/vel'


def parse_imu_info(can_list):
    imu_info = {}
    for can_file in can_list:
        if '.txt' in can_file:
            imu_info = read_gpsimu_from_txt(can_file)
            break
    return imu_info

def write_imu_msg(bag, imu_info):
    print('Writing IMU msgs ...')
    for info in tqdm(imu_info):
        if not ('timestamp' in info.keys() and 'HeadingPitchRoll' in info.keys() and \
           'HeadingPitchRollSigma' in info.keys() and 'Accel_Vehicle' in info.keys() and \
           'AngRateVehicle' in info.keys() and 'PoseSigma' in info.keys() and \
           'VelocityLevel' in info.keys() and 'Latitude' in info.keys() and \
           'Longitude' in info.keys() and 'Altitude' in info.keys()) and \
           'InsStatus' in info.keys():
            continue

        # IMU raw
        heading = radians(info['HeadingPitchRoll'][0])
        pitch   = radians(info['HeadingPitchRoll'][1])
        roll    = radians(info['HeadingPitchRoll'][2])
        yaw = -heading
        if heading > np.pi:
            yaw = 2*np.pi - heading
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        sigma_yaw, sigma_pitch, sigma_roll = info['HeadingPitchRollSigma']
        orient_covariance = np.diag([sigma_roll**2, sigma_pitch**2, sigma_yaw**2])
        imu_raw = Imu()
        imu_raw.header.frame_id = imu_frame_id
        imu_raw.header.stamp = rospy.Time.from_sec(info['timestamp'])
        imu_raw.orientation.x = q[0]
        imu_raw.orientation.y = q[1]
        imu_raw.orientation.z = q[2]
        imu_raw.orientation.w = q[3]
        imu_raw.orientation_covariance = orient_covariance.flatten().tolist()
        imu_raw.linear_acceleration.x = info['Accel_Vehicle'][0]
        imu_raw.linear_acceleration.y = info['Accel_Vehicle'][1]
        imu_raw.linear_acceleration.z = info['Accel_Vehicle'][2]
        imu_raw.angular_velocity.x = radians(info['AngRateVehicle'][0])
        imu_raw.angular_velocity.y = radians(info['AngRateVehicle'][1])
        imu_raw.angular_velocity.z = radians(info['AngRateVehicle'][2])
        bag.write(imu_raw_topic, imu_raw, t=imu_raw.header.stamp)

        # IMU correct which in ROS REP-105:
        #       https://docs.ros.org/en/melodic/api/robot_localization/html/integrating_gps.html
        roll_c = roll
        pitch_c = -pitch
        yaw_c = -heading + np.pi/2
        if yaw_c < -np.pi:
            yaw_c = 2*np.pi + yaw_c
        q = tf.transformations.quaternion_from_euler(roll_c, pitch_c, yaw_c)
        imu_correct = Imu()
        imu_correct.header.frame_id = imu_frame_id
        imu_correct.header.stamp = rospy.Time.from_sec(info['timestamp'])
        imu_correct.orientation.x = q[0]
        imu_correct.orientation.y = q[1]
        imu_correct.orientation.z = q[2]
        imu_correct.orientation.w = q[3]
        imu_correct.orientation_covariance = orient_covariance.flatten().tolist()
        imu_correct.linear_acceleration.x =  info['Accel_Vehicle'][1]
        imu_correct.linear_acceleration.y = -info['Accel_Vehicle'][0]
        imu_correct.linear_acceleration.z =  info['Accel_Vehicle'][2]
        imu_correct.angular_velocity.x =  radians(info['AngRateVehicle'][1])
        imu_correct.angular_velocity.y = -radians(info['AngRateVehicle'][0])
        imu_correct.angular_velocity.z =  radians(info['AngRateVehicle'][2])
        bag.write(imu_correct_topic, imu_correct, t=imu_correct.header.stamp)
        

        # # GPS
        # x, y, z = gps2ecef([info["Latitude"], info['Longitude'], info['Altitude']])
        # sigma_x, sigma_y, sigma_z = info['PoseSigma']
        # pose_covariance = np.diag([sigma_x**2,    sigma_y**2,     sigma_z**2, 
        #                            sigma_roll**2, sigma_pitch**2, sigma_yaw**2])
        # gps = Odometry()
        # gps.header.frame_id = frame_id
        # gps.header.stamp = rospy.Time.from_sec(info['timestamp'])
        # gps.pose.pose.position.x = x
        # gps.pose.pose.position.y = y
        # gps.pose.pose.position.z = z
        # gps.pose.pose.orientation = imu.orientation
        # gps.pose.covariance = pose_covariance.flatten().tolist()
        # gps.twist.twist.linear.x = info['VelocityLevel'][0]
        # gps.twist.twist.linear.y = info['VelocityLevel'][1]
        # gps.twist.twist.linear.z = info['VelocityLevel'][2]
        # gps.twist.twist.angular = imu.angular_velocity
        # bag.write(gps_topic, gps, t=gps.header.stamp)

        # GPS
        sigma_x, sigma_y, sigma_z = info['PoseSigma']
        pose_covariance = np.diag([sigma_x**2, sigma_y**2, sigma_z**2])
        gps = NavSatFix()
        gps.header.frame_id = gps_frame_id
        gps.header.stamp = rospy.Time.from_sec(info['timestamp'])
        gps.latitude  = info['Latitude']
        gps.longitude = info['Longitude']
        gps.altitude  = info['Altitude']
        gps.position_covariance_type = 2 # covariance in ENU coord
        gps.position_covariance = pose_covariance.flatten().tolist()
        gps.status.service = 4
        gps.status.status = -1
        if info['InsStatus'][2] == 4 or info['InsStatus'][2] == 8:
            gps.status.status = 0
        bag.write(gps_fix_topic, gps, t=gps.header.stamp)

        # Velocity
        vel = TwistStamped()
        vel.header.frame_id = gps_frame_id
        vel.header.stamp = rospy.Time.from_sec(info['timestamp'])
        vel.twist.linear.x = info['VelocityLevel'][0]
        vel.twist.linear.y = info['VelocityLevel'][1]
        vel.twist.linear.z = info['VelocityLevel'][2]
        # vel.twist.angular = imu_correct.angular_velocity
        bag.write(gps_vel_topic, vel, t=vel.header.stamp)


def write_lidar_msg(bag, pcd_list, time_list):
    lidar2ego = np.array([
        [-1,  0,  0,  1.170],
        [ 0, -1,  0, -0.002],
        [ 0,  0,  1,  2.194],
        [ 0,  0,  0,  1    ]
    ])
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
        header = Header()
        header.seq = i
        header.stamp = rospy.Time.from_sec(time_list[i])
        header.frame_id = lidar_frame_id
        pc_data = pypcd.PointCloud.from_path(pcd_list[i]).pc_data
        # lidar to ROS REP-105
        pc_data['x'] = -pc_data['x']
        pc_data['y'] = -pc_data['y']
        if 'ring' in pc_data.dtype.names and 'timestamp' in pc_data.dtype.names:
            points = pc_data[['x', 'y', 'z', 'intensity', 'ring', 'timestamp']]
        else:
            print('Error: Should contain ring and timestamp in pcd.dtype!')
            exit(1)
        pc2 = point_cloud2.create_cloud(header, fields, points)
        pc2.is_dense = True
        bag.write(lidar_topic, pc2, header.stamp)


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
    if len(imu_info) == 0:
        print(f'Error: No can file in {data_path}')
        exit(1)

    print(f'data_path: {data_path}')
    print(f'{session} contains {len(pcd_list)} pcd files')
    print(f'{session} contains {len(imu_info)} imu infos')

    tic = rospy.Time.now()
    bag = rosbag.Bag(osp.join(data_path, f'{session}.bag'), 'w')
    print(f'Start Converting {session}.bag')

    write_imu_msg(bag, imu_info)
    write_lidar_msg(bag, pcd_list, time_list)

    bag.close()


def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('data_path', type=str, default='', 
                        help='specify the data path including LiDAR and CAN')
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    # Usage:
    #   python bst2bag/bst2bag.py /workspace/work/data/output_20230109/data/6v_img_lidar_20230109/20230109_084712

    args = parse_config()
    rospy.init_node('bst2bag', anonymous=True)

    bst2bag()
    print('Done!')
