#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geoid.hpp>
#include <deque>
#include <mutex>

#include "utility.h"

class GpsOdometry : public ParamServer {
public:
    GpsOdometry(ros::NodeHandle &nh) {
        subGps = nh.subscribe<sensor_msgs::NavSatFix>("/gps/fix", 1000, &GpsOdometry::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 1000, &GpsOdometry::imuHandler, this, ros::TransportHints().tcpNoDelay());
        pubGpsOdom = nh.advertise<nav_msgs::Odometry>("/odometry/gps", 100, false);
        pubGpsPath = nh.advertise<nav_msgs::Path>("lio_sam/gps/path", 100);
    }

private:
    void gpsHandler(const sensor_msgs::NavSatFixConstPtr &msg) {
        // std::cout << "gps status: " << (int)msg->status.status << std::endl;
        if (std::isnan(msg->latitude + msg->longitude + msg->altitude)) {
            //ROS_ERROR("POS LLA NAN...");
            return;
        }
        Eigen::Vector3d lla(msg->latitude, msg->longitude, msg->altitude);
        //std::cout << "LLA: " << lla.transpose() << std::endl;
        if (!initXyz) {
            ROS_INFO("Init Orgin GPS LLA  %f, %f, %f", msg->latitude, msg->longitude,
                     msg->altitude);
            geo_converter.Reset(lla[0], lla[1], lla[2]);
            initXyz = true;
        }

        // if you have some satellite info or rtk status info, put it here
        int status = -1;
        int satell_num = -1;
        // LLA->ENU, better accuacy than gpsTools especially for z value
        double x, y, z;
        geo_converter.Forward(lla[0], lla[1], lla[2], x, y, z);
        std::cout << "enu: " << x << ", " << y << ", " << z << std::endl;
        Eigen::Vector3d enu(x, y, z);
        if (abs(enu.x()) > 10000 || abs(enu.y()) > 10000 || abs(enu.z()) > 10000) {
            ROS_INFO("Error ogigin : %f, %f, %f", enu(0), enu(1), enu(2));
            return;
        }

        // use Imu orientation
        std::lock_guard<std::mutex> lock(mtx);

        // pub gps odometry
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = odometryFrame;
        odom_msg.child_frame_id = "gps";
        odom_msg.pose.pose.position.x = enu(0);
        odom_msg.pose.pose.position.y = enu(1);
        odom_msg.pose.pose.position.z = enu(2);
        odom_msg.pose.covariance[0] = msg->position_covariance[0];
        odom_msg.pose.covariance[7] = msg->position_covariance[4];
        odom_msg.pose.covariance[14] = msg->position_covariance[8];
        odom_msg.pose.covariance[1] = lla[0];
        odom_msg.pose.covariance[2] = lla[1];
        odom_msg.pose.covariance[3] = lla[2];
        odom_msg.pose.covariance[4] = status;
        odom_msg.pose.covariance[5] = satell_num;
        odom_msg.pose.covariance[6] = true;
        odom_msg.pose.pose.orientation = currImu.orientation;
        pubGpsOdom.publish(odom_msg);

        /** for gps visualization */
        // publish gps path
        gps_path.header.frame_id = odometryFrame;
        gps_path.header.stamp = msg->header.stamp;
        geometry_msgs::PoseStamped pose;
        pose.header = gps_path.header;
        pose.pose.position.x = enu(0);
        pose.pose.position.y = enu(1);
        pose.pose.position.z = enu(2);
        pose.pose.orientation = currImu.orientation;
        gps_path.poses.push_back(pose);
        pubGpsPath.publish(gps_path);
    }

    void imuHandler(const sensor_msgs::ImuConstPtr &msg) {
        std::lock_guard<std::mutex> lock(mtx);
        currImu = imuConverter(*msg);
    }


    ros::Publisher pubGpsOdom, pubGpsPath, init_origin_pub;
    ros::Subscriber subGps, subImu;

    std::mutex mtx;
    std::deque<sensor_msgs::NavSatFixConstPtr> gpsBuf;

    bool initXyz = false;
    nav_msgs::Path gps_path;
    GeographicLib::LocalCartesian geo_converter;
    sensor_msgs::Imu currImu;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lio_sam");
    ros::NodeHandle nh;
    GpsOdometry gps(nh);
    ROS_INFO("\033[1;32m----> Simple GPS Odmetry Started.\033[0m");
    ros::spin();
    return 1;
}