#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <map>
#include <cmath> 
#include <vector>
#include <std_msgs/Empty.h>
#include <time.h>
#include <fstream>
#include <sstream>
#include <string>


// ros::Publisher marker_pub; // パブリッシャーをグローバルで宣言

class GoalJudge
{
public:
    GoalJudge(){
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        goal_judge_pub_ = nh_.advertise<std_msgs::Int16>("goal_judge", 1);

        odom_sub_ = nh_.subscribe("ypspur_ros/odom", 1000, &GoalJudge::odomCallback, this);
        amcl_sub_ = nh_.subscribe("/amcl_pose", 1000, &GoalJudge::amclPoseCallback, this);
        scan_sub_ = nh_.subscribe("/scan", 10, &GoalJudge::scanCallback, this);
        waypoint_sub_ = nh_.subscribe("/waypoint", 1000, &GoalJudge::waypointCallback, this);
        timer_callback_ = nh_.createTimer(ros::Duration(1.0), &GoalJudge::timerCallback, this);
        
        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_odom_x_ = 0.0;
        robot_odom_y_ = 0.0;
        robot_r_.x = 0.0;
        robot_r_.y = 0.0;
        robot_r_.z = 0.0;
        robot_r_.w = 1.0;

        pnh_.getParam("current_location", current_location_);

        }


private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string current_location_;

    // pnh_.getParam("current_location", current_location_);

    ros::Subscriber odom_sub_, amcl_sub_, scan_sub_, waypoint_sub_;
    ros::Publisher marker_pub_, goal_judge_pub_;
    ros::Timer timer_callback_;
    ros::Time timer_start_;
    ros::Time timer_now_;
    std_msgs::Int16 goal_judge_msg_;
    geometry_msgs::Point p_;
    geometry_msgs::Quaternion robot_r_;
    sensor_msgs::LaserScan::ConstPtr scan_;
    geometry_msgs::PoseStamped goal_; // 目標地点



    double roll_, pitch_, yaw_;
    double theta_;
    double angle_rad_, angle_deg_;
    double distance_, distance_judge_;
    double robot_odom_x_, robot_odom_y_;
    double robot_x_, robot_y_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_);
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent&);
    void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat);
    int nearPosition(geometry_msgs::PoseStamped goal);
    void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
};

    void GoalJudge::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {//near_waypointなどはこっちでやる
        robot_odom_x_ = msg->pose.pose.position.x;
        robot_odom_y_ = msg->pose.pose.position.y;
        // std::cout << "robot_odom_x_:" << robot_odom_x_ << "robot_odom_y_;" << robot_odom_y_ << std::endl;
    }

    void GoalJudge::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
        scan_ = scan;
    }

    void GoalJudge::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_r_ = msg->pose.pose.orientation;
        // ROS_INFO("Current estimated pose: [robot_x_: %f, robot_y_: %f, theta: %f]", robot_x_, robot_y_, tf::getYaw(msg->pose.pose.orientation));
    }


    void GoalJudge::timerCallback(const ros::TimerEvent&) {
        std::cout << "nearposition result" << nearPosition(goal_) << std::endl;
        if(nearPosition(goal_)){
            goal_judge_msg_.data = 1;
            // goal_judge_pub_.publish(goal_judge_msg_);
        }
        else{
            goal_judge_msg_.data = 0;
        }
        goal_judge_pub_.publish(goal_judge_msg_);


    }

    void GoalJudge::geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
    {
        tf::Quaternion quat;
        quaternionMsgToTF(geometry_quat, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    }

    int GoalJudge::nearPosition(geometry_msgs::PoseStamped goal){
        // double difx = robot_odom_x_ - goal.pose.position.x;
        // double dify = robot_odom_y_ - goal.pose.position.y;
        double difx = robot_x_ - goal.pose.position.x;
        double dify = robot_y_ - goal.pose.position.y;
        std::cout << "sqrt(difx * difx + dify * dify)" << sqrt(difx * difx + dify * dify) << std::endl;
        return (sqrt(difx * difx + dify * dify) < 0.2);
    }

    void GoalJudge::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        goal_.pose.position.x = msg->pose.position.x;
        goal_.pose.position.y = msg->pose.position.y;
        goal_.pose.position.z = msg->pose.position.z;
        std::cout << "msg->pose.position.x" << msg->pose.position.x << std::endl;
        std::cout << "goal_.pose.position.x:" << goal_.pose.position.x << std::endl;
        // std::cout << "waypointCallback" << std::endl;
        // std::cout << "msg->pose.position.x:" << msg->pose.position.x << std::endl;
        // std::cout << "msg->pose.position.y:" << msg->pose.position.y << std::endl;
        // std::cout << "msg->pose.orientation.x:" << msg->pose.orientation.x << std::endl;
        // std::cout << "msg->pose.orientation.y:" << msg->pose.orientation.y << std::endl;
        // std::cout << "msg->pose.orientation.z:" << msg->pose.orientation.z << std::endl;
        // std::cout << "msg->pose.orientation.w:" << msg->pose.orientation.w << std::endl;
        // std::cout << "msg->header.frame_id:" << msg->header.frame_id << std::endl;
        // std::cout << "msg->header.stamp:" << msg->header.stamp << std::endl;
        // std::cout << "msg->header.seq:" << msg->header.seq << std::endl;
        // std::cout << "msg->header.stamp:" << msg->header.stamp << std::endl;
        // std::cout << "msg->header.stamp:" << msg->header.stamp << std::endl;
        // std::cout << "msg->header.stamp:" << msg->header.stamp << std::endl;
        // std::cout << "msg->header.stamp:" << msg->header.stamp << std::endl;
        // std::cout << "msg->header.stamp:" << msg->header.stamp << std::endl;
        // std::cout << "msg->header.stamp:" << msg->header.stamp << std::endl;
        // std::cout << "msg->header.stamp:" << msg->header.stamp << std::endl;
        // std::cout << "msg->header.stamp:" << msg->header.stamp << std::endl;
        // std::cout << "msg->header.stamp:" << msg->header.stamp << std::endl;
    }

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_judge");

    GoalJudge cm;

    ros::Rate loop_rate(100);

    while(ros::ok()) {
        ros::spinOnce();
        // std::cout << "sub_waypoint_flag_:" << sub_waypoint_flag_ << std::endl;
        // std::cout << "state:" << state_ << std::endl;
        // twist_pub.publish(twist);

        loop_rate.sleep();
    }
    return 0;
}