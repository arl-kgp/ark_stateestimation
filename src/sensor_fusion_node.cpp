#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <ark_stateestimation/VariablesConfig.h>

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

    typedef struct {
      double q; //process noise covariance
      double r; //measurement noise covariance
      double x; //value
      double p; //estimation error covariance
      double k; //kalman gain
    } kalman_state;

    kalman_state filter_data_y;
    kalman_state filter_data_x;
    double kalman_q;
    double kalman_r;

    double previous_estimate_x;
    double previous_estimate_y;

    SensorListener()
    {
        odom_filtered.header.seq = 1;
        odom_filtered.header.frame_id = "odom";
        odom_filtered.child_frame_id = "base_link";
        ran_once = false;
        kalman_q = 4;
        kalman_r = 4;
        imu_sub = nh_.subscribe("/mavros/imu/data", 1, &SensorListener::imuCb, this);
        orb_sub = nh_.subscribe("/odom", 1, &SensorListener::orbCb, this);
        alt_sub = nh_.subscribe("/mavros/global_position/local", 1, &SensorListener::altCb, this);
        odometry_pub = nh_.advertise<nav_msgs::Odometry>("/odometry/filtered", 1000);
    }

    ~SensorListener()
    {
    }


    kalman_state kalman_init(double q, double r, double p, double intial_value)
    {
      kalman_state result;
      result.q = q;
      result.r = r;
      result.p = p;
      result.x = intial_value;

      return result;
    }

    void kalman_update(kalman_state* state, double measurement)
    {
      //prediction update
      //omit x = x
      state->p = state->p + state->q;

      //measurement update
      state->k = state->p / (state->p + state->r);
      state->x = state->x + state->k * (measurement - state->x);
      state->p = (1 - state->k) * state->p;
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
        //ROS_INFO("%.2f", time_diff);
        odom_filtered.pose.pose.position.z = alt_stored.pose.pose.position.z;
        odom_filtered.pose.pose.orientation = imu_stored.orientation;
        odom_filtered.header.seq += 1;
        odom_filtered.header.stamp = current_time;
        previous_time = current_time;
        odom_filtered.twist.twist.angular = imu_stored.angular_velocity;

        if (!ran_once)
        {
            ran_once = true;
            //relative odom:
            relative_odom = *msg;
            odom_filtered.pose.pose.position.x = 0;
            odom_filtered.pose.pose.position.y = 0;
            odom_filtered.twist.twist.linear.x = 0;
            odom_filtered.twist.twist.linear.y = 0;
            odom_filtered.twist.twist.linear.z = 0;
            filter_data_y = kalman_init(kalman_q, kalman_r, 0, 0);
            filter_data_x = kalman_init(kalman_q, kalman_r, 0, 0);
            odom_filtered_previous = odom_filtered;
            return;
        }

        kalman_update(&filter_data_y, msg->pose.pose.position.y - relative_odom.pose.pose.position.y);
        kalman_update(&filter_data_x, msg->pose.pose.position.x - relative_odom.pose.pose.position.x);

        odom_filtered.pose.pose.position.y = filter_data_y.x;
        odom_filtered.pose.pose.position.x = filter_data_x.x;

        odom_filtered.twist.twist.linear.x = - (odom_filtered.pose.pose.position.x - odom_filtered_previous.pose.pose.position.x)/time_diff;
        odom_filtered.twist.twist.linear.y = - (odom_filtered.pose.pose.position.y - odom_filtered_previous.pose.pose.position.y)/time_diff;
        odom_filtered.twist.twist.linear.z = (odom_filtered.pose.pose.position.z - odom_filtered_previous.pose.pose.position.z)/time_diff;
        cout<<(float)odom_filtered.twist.twist.linear.x<<","<<(float)odom_filtered.twist.twist.linear.y<<endl;

        odom_filtered_previous = odom_filtered;

        odometry_pub.publish(odom_filtered);
    }
    void altCb(const nav_msgs::OdometryConstPtr& msg)
    {
        alt_stored = *msg;
    }
};

void dr_callback(ark_stateestimation::VariablesConfig &config, uint32_t level, SensorListener *sl) {
    sl->kalman_q = config.kalman_q;
    sl->kalman_r = config.kalman_r;
    sl->filter_data_y.q = config.kalman_q;
    sl->filter_data_y.r = config.kalman_r;
    sl->filter_data_x.q = config.kalman_q;
    sl->filter_data_x.r = config.kalman_r;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_fusion_node");
    SensorListener sl;
    //ros::spin();
    ros::Rate r(10); // 10 hz

    dynamic_reconfigure::Server<ark_stateestimation::VariablesConfig> server;
    dynamic_reconfigure::Server<ark_stateestimation::VariablesConfig>::CallbackType f;

    f = boost::bind(&dr_callback, _1, _2, &sl);
    server.setCallback(f);

    while (1)
    {      
      ros::spinOnce();
      r.sleep();
    }
    return 0;
}
