#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

using namespace std;


class SensorListener
{
public:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub;
    ros::Subscriber orb_sub;
    ros::Subscriber alt_sub;
    ros::Publisher odometry_pub;

    nav_msgs::Odometry alt_stored;
    sensor_msgs::Imu imu_stored;
    nav_msgs::Odometry relative_odom;
    nav_msgs::Odometry odom_filtered;
    nav_msgs::Odometry odom_filtered_previous;
    bool ran_once;
    ros::Time previous_time;
    SensorListener()
    {
        odom_filtered.header.seq = 1;
        odom_filtered.header.frame_id = "odom";
        odom_filtered.child_frame_id = "base_link";
        ran_once = false;
        imu_sub = nh_.subscribe("/mavros/imu/data", 1, &SensorListener::imuCb, this);
        orb_sub = nh_.subscribe("/odom", 1, &SensorListener::orbCb, this);
        alt_sub = nh_.subscribe("/mavros/global_position/local", 1, &SensorListener::altCb, this);
        odometry_pub = nh_.advertise<nav_msgs::Odometry>("/odometry/filtered", 1000);
    }

    ~SensorListener()
    {
    }
 

    void imuCb(const sensor_msgs::ImuConstPtr& msg)
    {
        imu_stored = *msg;
    }
    void orbCb(const nav_msgs::OdometryConstPtr& msg)
    {
        ros::Time current_time = ros::Time::now();
        double t1 = current_time.toNSec()/1000000000.0;
        double t2 = previous_time.toNSec()/1000000000.0;
        double time_diff = t1 - t2;
        odom_filtered.pose.pose.position.x = msg->pose.pose.position.x - relative_odom.pose.pose.position.x;
        odom_filtered.pose.pose.position.y = msg->pose.pose.position.y - relative_odom.pose.pose.position.y;
        odom_filtered.pose.pose.position.z = alt_stored.pose.pose.position.z;
        odom_filtered.pose.pose.orientation = imu_stored.orientation;
        odom_filtered.header.seq += 1;
        odom_filtered.header.stamp = current_time;
        previous_time = current_time;
        odom_filtered.twist.twist.linear.x = (odom_filtered.pose.pose.position.x - odom_filtered_previous.pose.pose.position.x)/time_diff;
        odom_filtered.twist.twist.linear.y = (odom_filtered.pose.pose.position.y - odom_filtered_previous.pose.pose.position.y)/time_diff;
        odom_filtered.twist.twist.linear.z = (odom_filtered.pose.pose.position.z - odom_filtered_previous.pose.pose.position.z)/time_diff;
        odom_filtered.twist.twist.angular = imu_stored.angular_velocity;

        odom_filtered_previous = odom_filtered;
        if (!ran_once)
        {
            ran_once = true;
            //relative odom:
            relative_odom = *msg;
            odom_filtered.pose.pose.position.x = 0;
            odom_filtered.pose.pose.position.y = 0;
            odom_filtered_previous = odom_filtered;
            return;
        }
        odometry_pub.publish(odom_filtered);
    }
    void altCb(const nav_msgs::OdometryConstPtr& msg)
    {
        alt_stored = *msg;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_fusion_node");
    SensorListener sl;
    ros::spin();
    ros::Rate r(10); // 10 hz
    /*while (1)
    {      
      ros::spinOnce();
      r.sleep();
    }*/
    return 0;
}
