#include <ros/ros.h>
#include <chrono>
#include <thread>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/BatteryState.h>

class MMPUGMavInterface {
public:
    MMPUGMavInterface(
        ros::NodeHandle& nh,
        ros::NodeHandle& nhp
    ):
        nh_(nh),
        nhp_(nhp)
    {
        nhp.param<double>("manual_mode_timeout", manual_mode_timeout_, 0.2);
        nhp.param<double>("max_steering_angle", max_steering_angle_, 0.524);
        nhp.param<double>("forward_steering_trim", forward_steering_trim_, -90.);
        nhp.param<double>("reverse_steering_trim", reverse_steering_trim_, 75.);
        nhp.param<double>("forward_scale_a", forward_scale_a_, 143.);
        nhp.param<double>("forward_scale_b", forward_scale_b_, 27.);
        nhp.param<double>("reverse_scale_a", reverse_scale_a_, 143.);
        nhp.param<double>("reverse_scale_b", reverse_scale_b_, -27.);
        nhp.param<bool>("invert_throttle", invert_throttle_, false);
        nhp.param<bool>("invert_steering", invert_steering_, true);

        manual_override_engaged_ = false;
        arm_status_ = false;


        manual_override_sub_ = nh_.subscribe("low_level_control/manual_override", 1, &MMPUGMavInterface::manualOverrideCallback, this);
        twist_control_sub_ = nh_.subscribe("low_level_control/cmd_vel", 1, &MMPUGMavInterface::twistControlCallback, this);
        mavros_state_sub_ = nh_.subscribe("mavros/state", 1, &MMPUGMavInterface::mavStateCallback, this);

        armed_status_pub_ = nh.advertise<std_msgs::Bool>("low_level_control/arm_status", 1);
        manual_override_status_pub_ = nh.advertise<std_msgs::Bool>("low_level_control/manual_override_status", 1);
        control_pub_ = nh.advertise<mavros_msgs::ManualControl>("mavros/manual_control/send", 1);

        arming_server_ = nh.advertiseService("low_level_control/arming", &MMPUGMavInterface::armDisarmCallback, this);
        
        mavros_arming_service_ = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    };

    void run() {
        ros::Rate r(100);

        while(ros::ok()) {
            ros::spinOnce();

            // Print current state.
            if(std::fabs((ros::Time::now() - last_manual_mode_).toSec()) > manual_mode_timeout_) {
                manual_override_engaged_ = false;
            }
            ROS_INFO_THROTTLE(4, "Armed: %d, Manual: %d",
                arm_status_,
                static_cast<int>(manual_override_engaged_)
            );

            // Publish Arming Status
            std_msgs::Bool arm_msg;
            arm_msg.data = arm_status_;
            armed_status_pub_.publish(arm_msg);

            // Publish Manual Override Status
            std_msgs::Bool manual_override_msg;
            manual_override_msg.data = manual_override_engaged_;
            manual_override_status_pub_.publish(manual_override_msg);
            r.sleep();
        }

    }
    
    void mavStateCallback(const mavros_msgs::State::ConstPtr& msg) {
        arm_status_ = msg->armed;
    }

    void twistControlCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        if(manual_override_engaged_) {
            ROS_WARN_THROTTLE(5, "[Command] Unable to use command, manual override engaged.");
            return;
        }

        publishCommand(msg->twist.linear.x, msg->twist.angular.z);
    };


    void manualOverrideCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        last_manual_mode_ = ros::Time::now();
        manual_override_engaged_ = true;

        publishCommand(msg->twist.linear.x, msg->twist.angular.z);
    };

    void publishCommand(double linear_x, double angular_z) {
        if(!arm_status_) {
            ROS_WARN_THROTTLE(5, "[Publish Command] disarmed");
            return;
        }

        double throttle_candidate = 0.0;
        double steering_candidate = 1000.0 * angular_z / max_steering_angle_;
        // If linear_x is greater than 0.0, after scaling
        // the command should also be greater than 0.0
        bool force_positive = linear_x > 0.0;
        if(linear_x > 0.0) {
            steering_candidate += forward_steering_trim_;
            throttle_candidate = linear_x * forward_scale_a_ + forward_scale_b_;
        } else if(linear_x < 0.0) {
            steering_candidate += reverse_steering_trim_;
            throttle_candidate = linear_x * reverse_scale_a_ + reverse_scale_b_;
        }
        // If we invert the throttle candidate, we also
        // invert the positive/negative requirement
        if(invert_throttle_) {
            throttle_candidate = -throttle_candidate;
            force_positive = !force_positive;
        }

        // Make positive / negative limits
        if(force_positive && throttle_candidate < 0.0) {
            ROS_WARN_THROTTLE(1, "[Publish Command] Recieved velocity command [%3.2f -> %3.2f > 0.0] High Cap", linear_x, throttle_candidate);
            throttle_candidate = 0.0;
        } else if(!force_positive && throttle_candidate > 0.0) {
            ROS_WARN_THROTTLE(1, "[Publish Command] Recieved velocity command [%3.2f -> %3.2f < 0.0] Low Cap", linear_x, throttle_candidate);
            throttle_candidate = 0.0;
        // No matter what we enforce the throttle candidate
        } else if(throttle_candidate < -1000.0) {
            ROS_WARN_THROTTLE(1, "[Publish Command] Recieved velocity command [%3.2f -> %3.2f < -1000.0] Low cap", linear_x, throttle_candidate);
            throttle_candidate = -1000.0;
        } else if(throttle_candidate > 1000.0) {
            ROS_WARN_THROTTLE(1, "[Publish Command] Recieved velocity command [%3.2f -> %3.2f > 1000.0] High Cap", linear_x, throttle_candidate);
            throttle_candidate = 1000.0;
        }

        if(invert_steering_) {
            steering_candidate = -steering_candidate;
        }
        if(steering_candidate > 1000.0) {
            steering_candidate = 1000.0;
        } else if(steering_candidate < -1000.0) {
            steering_candidate = -1000.0;
        }

        mavros_msgs::ManualControl control_msg;
        control_msg.y = steering_candidate;
        control_msg.z = throttle_candidate;
        control_pub_.publish(control_msg);
    }

    bool armDisarmCallback(
        std_srvs::SetBool::Request& request,
        std_srvs::SetBool::Response& response
    ) {
        response.message = arm_status_ ? "Was armed." : "Was disarmed.";
        response.success = true;

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = request.data;
        mavros_arming_service_.call(arm_cmd);
        
        return true;
    }

private:
    bool manual_override_engaged_;
    bool arm_status_;

    double manual_mode_timeout_;
    double max_steering_angle_;
    double forward_steering_trim_;
    double reverse_steering_trim_;
    double forward_scale_a_;
    double forward_scale_b_;
    double reverse_scale_a_;
    double reverse_scale_b_;
    bool invert_throttle_;
    bool invert_steering_;

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ros::Subscriber manual_override_sub_;
    ros::Subscriber twist_control_sub_;
    ros::Subscriber mavros_state_sub_;

    ros::Publisher armed_status_pub_;
    ros::Publisher manual_override_status_pub_;
    ros::Publisher control_pub_;

    ros::ServiceServer arming_server_;
    ros::ServiceClient mavros_arming_service_;

    ros::Time last_manual_mode_;
    ros::Time last_spot_feedback_;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "mmpug_mav_interface");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    MMPUGMavInterface node(nh, nhp);
    node.run();

    return 0;
}