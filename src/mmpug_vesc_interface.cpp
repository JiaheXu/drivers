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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <cansend.h>
#include <unistd.h>
#include <boost/thread/thread.hpp>
#include <mutex> 

std::mutex velo_lock;
std::mutex servo_lock;
std::mutex arm_status_lock;

class MMPUGVescInterface {
public:
    static int steering_cmd;
    static int velo_cmd;
    static int velo_cmd_ratio;
    static int servo_cmd_ratio;

    static int rpm_scale;
    static bool arm_status_;
    static int publish_ratio;

    MMPUGVescInterface(
        ros::NodeHandle& nh,
        ros::NodeHandle& nhp
    ):
        nh_(nh),
        nhp_(nhp)
    {
        nhp.param<double>("manual_mode_timeout", manual_mode_timeout_, 1);
        nhp.param<double>("max_steering_angle", max_steering_angle_, 0.524);
        nhp.param<double>("forward_steering_trim", forward_steering_trim_, -90.);
        nhp.param<double>("reverse_steering_trim", reverse_steering_trim_, 75.);
        nhp.param<double>("forward_scale_a", forward_scale_a_, 143.);
        nhp.param<double>("forward_scale_b", forward_scale_b_, 27.);
        nhp.param<double>("reverse_scale_a", reverse_scale_a_, 143.);
        nhp.param<double>("reverse_scale_b", reverse_scale_b_, -27.);
        nhp.param<bool>("invert_throttle", invert_throttle_, false);
        nhp.param<bool>("invert_steering", invert_steering_, false);
        nhp.param<bool>("arm_status", arm_status_, false);

        nhp.param<int>("servo_upper_bound", servo_upper_bound,950);
        nhp.param<int>("servo_lower_bound", servo_lower_bound,50); 

        nhp.param<bool>("DEBUG", DEBUG, false);
        nhp.param<int>("velo_cmd_ratio", velo_cmd_ratio, 100);
        nhp.param<int>("servo_cmd_ratio", servo_cmd_ratio, 100);
        nhp.param<int>("publish_ratio", publish_ratio, 100);
        nhp.param<int>("rpm_scale", rpm_scale, 5000);

        manual_override_engaged_ = false;

        memset(cmd, '0', sizeof(cmd));

    	cmd[6] = '3';
    	cmd[7] = '3';
        cmd[8] = '#';
        cmd[17] = '\0';

        manual_override_sub_ = nh_.subscribe("low_level_control/manual_override", 1, &MMPUGVescInterface::manualOverrideCallback, this);
        twist_control_sub_ = nh_.subscribe("low_level_control/cmd_vel", 1, &MMPUGVescInterface::twistControlCallback, this);

        armed_status_pub_ = nh.advertise<std_msgs::Bool>("low_level_control/arm_status", 1);
        manual_override_status_pub_ = nh.advertise<std_msgs::Bool>("low_level_control/manual_override_status", 1);

        arming_server_ = nh.advertiseService("low_level_control/arming", &MMPUGVescInterface::armDisarmCallback, this);

	status_timer = nh.createTimer(ros::Duration(1.f/(float)publish_ratio), &MMPUGVescInterface::status_timer_callback, this);
    };

    ~MMPUGVescInterface(){

    }

    template <typename I>
    static std::string n2hexstr(I w, size_t hex_len = sizeof(I)<<1 )
    {
        static const char* digits = "0123456789ABCDEF";
        std::string rc(hex_len, '0');
        for(size_t i=0,j=(hex_len-1)*4; i<hex_len;++i,j-=4 )
        {
            rc[i] = digits[(w>>j) & 0x0f];
        }
        return rc;
    }

  void status_timer_callback(const ros::TimerEvent& e){
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

    arm_status_lock.lock();
    arm_msg.data = arm_status_;
    arm_status_lock.unlock();
            
    armed_status_pub_.publish(arm_msg);

    // Publish Manual Override Status
    std_msgs::Bool manual_override_msg;
    manual_override_msg.data = manual_override_engaged_;
    manual_override_status_pub_.publish(manual_override_msg);
  }
  
  /*
    void run() {
        ros::Rate r(publish_ratio);

        while(ros::ok()) 
        {
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

            arm_status_lock.lock();
            arm_msg.data = arm_status_;
            arm_status_lock.unlock();
            
            armed_status_pub_.publish(arm_msg);

            // Publish Manual Override Status
            std_msgs::Bool manual_override_msg;
            manual_override_msg.data = manual_override_engaged_;
            manual_override_status_pub_.publish(manual_override_msg);
            r.sleep();
        }
    }
  */

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

    double servo_convert(double& angular)
    {
        return ( -1.0*angular + max_steering_angle_ ) / (2.0 * max_steering_angle_);
    }

    void send_velo_cmd(int velo_cmd)
    {
        std::string st = n2hexstr(velo_cmd,8);
        // cmd ID
        cmd[4] = '0';
        cmd[5] = '3';

    	cmd[9] = st[0];
    	cmd[10] = st[1];
        cmd[11] = st[2];
	    cmd[12] = st[3];
        cmd[13] = st[4];
        cmd[14] = st[5];
	    cmd[15] = st[6];
    	cmd[16] = st[7];
    	cansend(cmd);
    }
    void send_steering_cmd(int steering_cmd)
    {
        std::string st = n2hexstr(steering_cmd, 4);
        
        // cmd ID
        cmd[4] = '3';
    	cmd[5] = 'F';

    	cmd[9] = '0';
    	cmd[10] = '0';
        cmd[11] = '0';
	    cmd[12] = '0';
    	cmd[13] = st[0];
    	cmd[14] = st[1];
    	cmd[15] = st[2];
    	cmd[16] = st[3];
    	cansend(cmd);
    }
    void publishCommand(double linear_x, double angular_z) {

        steering_cmd = ( int(servo_convert(angular_z) * servo_scale) + servo_bias );

        if(steering_cmd < servo_lower_bound)
                steering_cmd = servo_lower_bound;
        if(steering_cmd > servo_upper_bound)
                steering_cmd = servo_upper_bound;
        
        velo_cmd = int( (linear_x * rpm_scale) );
        
        if(DEBUG)
        {
            std::cout << "linear_x: " <<( linear_x ) << std::endl;
            std::cout << "velo_cmd : " <<( velo_cmd  ) << "\n\n";  

            std::cout << "angular_z: " <<( angular_z ) << std::endl;
            std::cout << "steering_cmd : " <<( steering_cmd ) << "\n\n";  
        }

        if( !arm_status_ )
        {
            return;
        }

        send_steering_cmd(steering_cmd);
        send_velo_cmd(velo_cmd);

    }

    bool armDisarmCallback(
        std_srvs::SetBool::Request& request,
        std_srvs::SetBool::Response& response
    ) {
        response.message = arm_status_ ? "Was armed." : "Was disarmed.";
        response.success = true;

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = request.data;

        arm_status_lock.lock();
        arm_status_ = !arm_status_ ;
        if(DEBUG)
        {
            std::cout<< "vesc arm: "<< arm_status_ << "\n";
        }
        arm_status_lock.unlock();
        
        return true;
    }

private:
    bool manual_override_engaged_;

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

    int servo_scale = 900;
    int servo_bias = 50;
    int servo_upper_bound = 950;
    int servo_lower_bound = 50;
    bool DEBUG;

    char cmd[18];

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ros::Subscriber manual_override_sub_;
    ros::Subscriber twist_control_sub_;
    ros::Subscriber mavros_state_sub_;
    //ros::Subscriber arming_sub_;

    ros::Publisher armed_status_pub_;
    ros::Publisher manual_override_status_pub_;
    ros::Publisher control_pub_;

    ros::ServiceServer arming_server_;
    //ros::ServiceClient mavros_arming_service_;

    ros::Timer status_timer;

    ros::Time last_manual_mode_;
    ros::Time last_spot_feedback_;
    

};

int MMPUGVescInterface::steering_cmd = 500;
int MMPUGVescInterface::velo_cmd = 0;

int MMPUGVescInterface::velo_cmd_ratio = 100;
int MMPUGVescInterface::servo_cmd_ratio = 100;
int MMPUGVescInterface::publish_ratio = 10;
int MMPUGVescInterface::rpm_scale = 8000;
bool MMPUGVescInterface::arm_status_ = false;

int main(int argc, char** argv) {

    ros::init(argc, argv, "mmpug_vesc_interface");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    MMPUGVescInterface node(nh, nhp);
    //node.run();
    ros::spin();
    
    return 0;
}
