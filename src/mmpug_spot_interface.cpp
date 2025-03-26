#include <ros/ros.h>
#include <chrono>
#include <thread>
#include <math.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <spot_msgs/Feedback.h>
#include <spot_msgs/LeaseArray.h>
#include <spot_msgs/PowerState.h>
#include <spot_msgs/BatteryStateArray.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <spot_msgs/TrajectoryAction.h>
#include <spot_msgs/TrajectoryActionGoal.h>
#include <spot_msgs/TrajectoryActionResult.h>
#include <spot_msgs/TrajectoryActionFeedback.h>
#include <mmpug_msgs/NavigateStairRequest.h>

class MMPUGSpotInterface {
public:
    enum class arming_state_t {
        none = -1,
        claim = 0,
        power = 1,
        stand = 2,
        walking = 3
    };


    MMPUGSpotInterface(
        ros::NodeHandle& nh,
        ros::NodeHandle& nhp
    ):
        nh_(nh),
        nhp_(nhp),
        manual_mode_timeout_(0.2),
        spot_feedback_timeout_(1.5),
        arm_attempt_timeout_(3.0)
    {
        nhp.getParam("manual_mode_timeout", manual_mode_timeout_);
        nhp.getParam("spot_feedback_timeout", spot_feedback_timeout_);
        nhp.getParam("arm_attempt_timeout", arm_attempt_timeout_);
        nhp.param<bool>("DEBUG", DEBUG, true);

        arm_request_status_ = arming_state_t::none;
        arm_disarm_stage_ = arming_state_t::none;
        arm_status_ = false;
        manual_override_engaged_ = false;
        spot_leased_ = false;
        spot_sitting_ = false;
        spot_standing_ = false;
        spot_powered_ = false;
        state_transitioned_ = false;
        try_count_ = 2;
        sendGoal = false;
        sendStairGoal = false;
        spot_stair_mode_enabled_ = false;
        joystick_mode_override_ = false;
        stair_controller_override_ = false;

        manual_override_sub_ = nh_.subscribe("low_level_control/manual_override", 1, &MMPUGSpotInterface::manualOverrideCallback, this);
        twist_control_sub_ = nh_.subscribe("low_level_control/cmd_vel", 1, &MMPUGSpotInterface::twistControlCallback, this);
        spot_feedback_sub_ = nh_.subscribe("spot/status/feedback", 1, &MMPUGSpotInterface::spotFeedbackCallback, this);
        spot_lease_sub_ = nh_.subscribe("spot/status/leases", 1, &MMPUGSpotInterface::spotLeasesCallback, this);
        spot_power_sub_ = nh_.subscribe("spot/status/power_state", 1, &MMPUGSpotInterface::spotPowerCallback, this);
        spot_waypoint_sub_ = nh_.subscribe("low_level_control/goal_pose", 5, &MMPUGSpotInterface::spotGoalCallback, this);
        spot_battery_sub_ = nh_.subscribe("spot/status/battery_states", 1, &MMPUGSpotInterface::spotBatteryCallback, this);
        spot_stair_mode_sub_ = nh_.subscribe("low_level_control/stair_mode", 1, &MMPUGSpotInterface::spotStairModeCallback, this);
        spot_stair_waypoint_sub_ = nh_.subscribe("low_level_control/stair_goal", 1, &MMPUGSpotInterface::spotStairGoalCallback, this);
        spot_stair_controller_override_sub_ = nh.subscribe("low_level_control/stair_control_override", 1, &MMPUGSpotInterface::stairOverrideCallback, this);

        armed_status_pub_ = nh.advertise<std_msgs::Bool>("low_level_control/arm_status", 1);
        manual_override_status_pub_ = nh.advertise<std_msgs::Bool>("low_level_control/manual_override_status", 1);
        spot_control_pub_ = nh.advertise<geometry_msgs::Twist>("spot/cmd_vel", 1);
        // spot_voltage_pub_ = nh.advertise<std_msgs::Float64>("low_level_control/voltage", 1);
        // spot_current_pub_ = nh.advertise<std_msgs::Float64>("low_level_control/current", 1);
        spot_battery_percent_pub_ = nh.advertise<std_msgs::Float32>("battery_percentage", 1);
        spot_stair_mode_pub_ = nh.advertise<std_msgs::Bool>("low_level_control/stair_mode_status",1);
        stair_navigate_request_pub_ = nh.advertise<mmpug_msgs::NavigateStairRequest>("stair_controller/request",1);


        arming_server_ = nh.advertiseService("low_level_control/arming", &MMPUGSpotInterface::armDisarmCallback, this);
        
        spot_claim_service_ = nh.serviceClient<std_srvs::Trigger>("spot/claim");
        spot_release_service_ = nh.serviceClient<std_srvs::Trigger>("spot/release");
        spot_stand_service_ = nh.serviceClient<std_srvs::Trigger>("spot/stand");
        spot_sit_service_ = nh.serviceClient<std_srvs::Trigger>("spot/sit");
        spot_power_on_service_ = nh.serviceClient<std_srvs::Trigger>("spot/power_on");
        spot_power_off_service_ = nh.serviceClient<std_srvs::Trigger>("spot/power_off");
        spot_stair_mode_service_ = nh.serviceClient<std_srvs::SetBool>("spot/stair_mode");
        if(DEBUG)
        {
            std::cout << "initialization finished !!!!!!!!!!!!!!!!!!!!! \n";
            std::cout << "initialization finished !!!!!!!!!!!!!!!!!!!!! \n";
            std::cout << "initialization finished !!!!!!!!!!!!!!!!!!!!! \n";
        }
    };

    void run() {
        ros::Rate r(100);
        actionlib::SimpleActionClient<spot_msgs::TrajectoryAction> spotWaypointClient_("spot/trajectory", true);

        while(ros::ok()) {
            ros::spinOnce();

            // Print current state.
            if(std::fabs((ros::Time::now() - last_manual_mode_).toSec()) > manual_mode_timeout_) {
                manual_override_engaged_ = false;
            }

            if(std::fabs((ros::Time::now() - last_joystick_mode_).toSec()) > manual_mode_timeout_) {
                joystick_mode_override_ = false;
            }

            
            ROS_INFO_THROTTLE(2, "Armed: %d, Manual: %d, Lease: %d, Standing: %d, Sitting: %d, StairMode: %d, StairOverride: %d, Powered: %d, State: %d",
                arm_status_,
                manual_override_engaged_,
                spot_leased_,
                spot_standing_,
                spot_sitting_, 
                spot_stair_mode_enabled_,
                stair_controller_override_,
                spot_powered_,
                static_cast<int>(arm_disarm_stage_)
            );

            // Publish Arming Status
            std_msgs::Bool arm_msg;
            arm_msg.data = arm_status_;
            armed_status_pub_.publish(arm_msg);


            // Publish Manual Override Status
            std_msgs::Bool manual_override_msg;
            manual_override_msg.data = manual_override_engaged_;
            manual_override_status_pub_.publish(manual_override_msg);
            
            std_msgs::Bool stair_mode_msg;
            stair_mode_msg.data = spot_stair_mode_enabled_;
            spot_stair_mode_pub_.publish(stair_mode_msg);

            std_srvs::Trigger trigger_srv;

            if(!spotWaypointClient_.isServerConnected()) {
              ROS_WARN("Waiting for Spot ROS Driver to boot!");
              spotWaypointClient_.waitForServer(ros::Duration(2)); 
            }

            if((sendGoal) && !manual_override_engaged_ && arm_status_){
                float dist_x = spotGoal.target_pose.pose.position.x;
                float dist_y = spotGoal.target_pose.pose.position.y;
                float dist = sqrt((dist_x * dist_x) + (dist_y * dist_y));
                if(dist > 5) spotGoal.duration.data = ros::Duration(10,0);
                else spotGoal.duration.data = ros::Duration(4,0);
                
                if(spotGoal.target_pose.header.frame_id == "body"){
                    actionlib::SimpleClientGoalState state = spotWaypointClient_.getState();
                    if(state == actionlib::SimpleClientGoalState::ACTIVE || state == actionlib::SimpleClientGoalState::PENDING){
                        spotWaypointClient_.cancelGoal();
                    }
                    spotWaypointClient_.sendGoal(spotGoal);
                }
                else{
                    actionlib::SimpleClientGoalState state = spotWaypointClient_.getState();
                    spotGoal.target_pose.header.frame_id = "body";
                    if(state == actionlib::SimpleClientGoalState::ABORTED || state == actionlib::SimpleClientGoalState::PREEMPTED || state == actionlib::SimpleClientGoalState::RECALLED || state == actionlib::SimpleClientGoalState::LOST || state == actionlib::SimpleClientGoalState::SUCCEEDED){
                        spotWaypointClient_.sendGoal(spotGoal);
                        ROS_INFO_THROTTLE(1, "[Waypoint Mode] Resending the same goal");
                    }
                }
                sendGoal = false;
            }

            if(sendStairGoal && !manual_override_engaged_ && arm_status_){
                if(!spot_stair_mode_enabled_){
                    ROS_WARN("[Staircase Waypoint] Stair Mode disabled, enabling ..");
                    switchStairMode(true);
                }
                
                if(spot_stair_mode_enabled_){
                    float dist_x = spotStairGoal.target_pose.pose.position.x;
                    float dist_y = spotStairGoal.target_pose.pose.position.y;
                    float dist = sqrt((dist_x * dist_x) + (dist_y * dist_y));
                    if(dist > 5) spotStairGoal.duration.data = ros::Duration(10,0);
                    else spotStairGoal.duration.data = ros::Duration(4,0);
                
                    if(spotStairGoal.target_pose.header.frame_id == "body"){
                        actionlib::SimpleClientGoalState state = spotWaypointClient_.getState();
                        if(state == actionlib::SimpleClientGoalState::ACTIVE || state == actionlib::SimpleClientGoalState::PENDING){
                            spotWaypointClient_.cancelGoal();
                        }
                        spotWaypointClient_.sendGoal(spotStairGoal);
                    }
                    else if(spotStairGoal.target_pose.header.frame_id == "old"){
                        actionlib::SimpleClientGoalState state = spotWaypointClient_.getState();
                        spotStairGoal.target_pose.header.frame_id = "body";
                        if(state == actionlib::SimpleClientGoalState::ABORTED || state == actionlib::SimpleClientGoalState::PREEMPTED || state == actionlib::SimpleClientGoalState::RECALLED || state == actionlib::SimpleClientGoalState::LOST || state == actionlib::SimpleClientGoalState::SUCCEEDED){
                            spotWaypointClient_.sendGoal(spotStairGoal);
                            ROS_INFO_THROTTLE(1, "[Staircase Waypoint] Resending staircase goal");
                        }
                    }
                    else if(spotStairGoal.target_pose.header.frame_id == "cancel"){
                         if(!(spotWaypointClient_.getState().isDone()))
                            spotWaypointClient_.cancelAllGoals();
                    }
                    sendStairGoal = false;
                }
                else{
                    ROS_ERROR("[Staircase Waypoint] Couldnot enable stair_mode, denied staircase navigation request");
                }
            }

            if(manual_override_engaged_ || joystick_mode_override_){
                if(!(spotWaypointClient_.getState().isDone()))
                    spotWaypointClient_.cancelAllGoals();
                joystick_mode_override_ = false;
            }
            /**
             * Elevator state machine :) 
             * Base -> Claim -> Power -> Stand -> Walk
             **/

            // Base state
            if(arm_disarm_stage_ == arming_state_t::none) {
                arm_status_ = false;
                if(arm_disarm_stage_ < arm_request_status_) {
                    arm_disarm_stage_ = arming_state_t::claim;
                    state_transitioned_ = true;
                }
            } else
            // Claim State
            if(arm_disarm_stage_ == arming_state_t::claim) 
            {
                if(DEBUG)
                {
                    std::cout<< "arm_disarm_stage_: "<< static_cast<int>(MMPUGSpotInterface::arm_disarm_stage_) << "\n";
                    std::cout<< "arm_request_status_: "<< static_cast<int>(MMPUGSpotInterface::arm_request_status_) <<"\n"; 
                    std::cout<< "try_count_: " << try_count_ << "\n";
                    std::cout<< "spot_leased_: " << spot_leased_ << "\n"; 
                }   
                // We must claim
                if(arm_disarm_stage_ < arm_request_status_) 
                {
                    if(try_count_ > 0 && !spot_leased_) 
                    {
                        state_transitioned_ = false;
                        spot_claim_service_.call(trigger_srv);
                        try_count_--;
                        if(!trigger_srv.response.success) 
                        {
                            ROS_ERROR("[Arming::Claim] Unable to claim spot: %s", trigger_srv.response.message.c_str());
                            arm_request_status_ = arming_state_t::none;
                            state_transitioned_ = true;
                        }
                        else
                        {
                            ROS_WARN("[Arming::Claim] claimed spot success.");
                        }
                    }
                    if(spot_leased_) 
                    {
                        ROS_WARN("[Arming::Claim] Claimed spot.");
                        arm_disarm_stage_ = arming_state_t::power;
                        state_transitioned_ = true;
                    }
                } 
                
                else
                // We must release
                if(arm_disarm_stage_ > arm_request_status_) 
                {
                    if(!spot_leased_) 
                    {
                        spot_release_service_.call(trigger_srv);
                        if(!trigger_srv.response.success) 
                        {
                            ROS_ERROR("[Arming::Claim] Unable to release spot: %s", trigger_srv.response.message.c_str());
                        }
                        else
                        {
                            ROS_WARN("[Arming::Claim] Released spot.");
                        }
                    } 
                    else 
                    {
                        ROS_WARN("[Arming::Claim] Released spot.");
                        arm_disarm_stage_ = arming_state_t::none;
                    }
                }
            } 
            
            else
            // Power State
            if(arm_disarm_stage_ == arming_state_t::power) {
                // We must power on!
                if(arm_disarm_stage_ < arm_request_status_) {
                    if(state_transitioned_ && !spot_powered_) {
                        state_transitioned_ = false;
                        spot_power_on_service_.call(trigger_srv);
                        if(!trigger_srv.response.success) {
                            ROS_ERROR("[Arming::Power] Unable to power spot on: %s", trigger_srv.response.message.c_str());
                            arm_request_status_ =  arming_state_t::none;
                            state_transitioned_ = true;
                        }
                    }
                    if(spot_powered_) {
                        ROS_WARN("[Arming::Power] Powered on spot.");
                        arm_disarm_stage_ = arming_state_t::stand;
                        state_transitioned_ = true;
                    }
                } else
                // We must power off!
                if(arm_disarm_stage_ > arm_request_status_) {
                    if(spot_powered_) {
                        ros::Duration(3).sleep();
                        spot_power_off_service_.call(trigger_srv);
                        if(!trigger_srv.response.success) {
                            ROS_ERROR("[Arming::Power] Unable to power off spot: %s", trigger_srv.response.message.c_str());
                        }
                    } else {
                        ROS_WARN("[Arming::Power] Powered off spot.");
                        arm_disarm_stage_ = arming_state_t::claim;
                        state_transitioned_ = true;
                    }
                }
            } else
            // Stand
            if(arm_disarm_stage_ == arming_state_t::stand) {
                // We must stand
                if(arm_disarm_stage_ < arm_request_status_) {
                    if(state_transitioned_ && (spot_sitting_ || !spot_standing_)) {
                        state_transitioned_ = false;
                        spot_stand_service_.call(trigger_srv);
                        if(!trigger_srv.response.success) {
                            ROS_ERROR("[Arming::Stand] Unable to stand spot up: %s", trigger_srv.response.message.c_str());
                            arm_request_status_ =  arming_state_t::none;
                            state_transitioned_ = true;
                        }
                    }
                    if(spot_standing_ && !spot_sitting_) {
                        ROS_WARN("[Arming::Stand] Spot is standing.");
                        arm_disarm_stage_ = arming_state_t::walking;
                        state_transitioned_ = true;
                        arm_status_ = true;
                    }
                } else
                // We must sit
                if(arm_disarm_stage_ > arm_request_status_) {
                    if(spot_standing_ || !spot_sitting_) {
                        spot_sit_service_.call(trigger_srv);
                        if(!trigger_srv.response.success) {
                            ROS_ERROR("[Arming::Stand] Unable to make spot sit: %s", trigger_srv.response.message.c_str());
                        }
                    } else {
                        ROS_WARN("[Arming::Stand] Spot is sitting.");
                        arm_disarm_stage_ = arming_state_t::power;
                    }

                }
            } else
            // Walk!
            if(arm_disarm_stage_ == arming_state_t::walking) {
                if(arm_disarm_stage_ > arm_request_status_) {
                    arm_status_ = false;
                    arm_disarm_stage_ = arming_state_t::stand;
                }
            }
            r.sleep();
        }

    }

    void spotFeedbackCallback(const spot_msgs::Feedback::ConstPtr& msg) {
        if(msg->sitting && !spot_sitting_) {
            ROS_WARN("[Feedback] Spot is now sitting.");
        } else if(!msg->sitting && spot_sitting_) {
            ROS_WARN("[Feedback] Spot is no longer sitting.");
        }
        if(msg->standing && !spot_standing_) {
            ROS_WARN("[Feedback] Spot is now standing.");
        } else if(!msg->standing && spot_standing_) {
            ROS_WARN("[Feedback] Spot is no longer standing.");
        }
        spot_sitting_ = msg->sitting;
        spot_standing_ = msg->standing;
    }

    void spotLeasesCallback(const spot_msgs::LeaseArray::ConstPtr& msg) {
        for(const auto& r : msg->resources) {
            if(r.resource == "body" && r.lease_owner.client_name.find("ros") != std::string::npos) {
                if(!spot_leased_) {
                    ROS_WARN("[Leases] Spot lease gained.");
                }
                spot_leased_ = true;
                return;
            }
        }
        if(spot_leased_) {
            ROS_WARN("[Leases] Spot lease lost.");
        }
        spot_leased_ = false;
    }

    void spotPowerCallback(const spot_msgs::PowerState::ConstPtr& msg) {
        auto np = msg->motor_power_state == spot_msgs::PowerState::STATE_ON;
        if(np && !spot_powered_) {
            ROS_WARN("[Power] Spot has powered on.");
        } else if(!np && spot_powered_) {
            ROS_WARN("[Power] Spot has powered off.");
        }
        spot_powered_ = np;
    }


    void spotGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        spotGoal.target_pose.pose = msg->pose;
        spotGoal.target_pose.header = msg->header;
        sendGoal = true;
    }

    void spotStairGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        spotStairGoal.target_pose.pose = msg->pose;
        spotStairGoal.target_pose.header = msg->header;
        sendStairGoal = true;
    }

    void spotBatteryCallback(const spot_msgs::BatteryStateArray::ConstPtr& msg){
        // std_msgs::Float64 current_msg;
        // current_msg.data = msg->battery_states[0].current;
        // std_msgs::Float64 voltage_msg; 
        // voltage_msg.data = msg->battery_states[0].voltage;
        std_msgs::Float32 percent_msg;
        percent_msg.data = msg->battery_states[0].charge_percentage;

        // spot_voltage_pub_.publish(voltage_msg);
        // spot_current_pub_.publish(current_msg);
        spot_battery_percent_pub_.publish(percent_msg);
    }

    void twistControlCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        if(!arm_status_) {
            ROS_WARN_THROTTLE(5, "[Command] Unable to use command, disarmed.");
            return;
        }

        if((std::fabs((ros::Time::now() - last_manual_mode_).toSec()) < manual_mode_timeout_ ) || stair_controller_override_) {
            ROS_WARN_THROTTLE(5, "[Command] Unable to use command, manual override engaged/stair controller override engaged");
            return;
        }
        last_joystick_mode_ = ros::Time::now();
        joystick_mode_override_ = true;
        spot_control_pub_.publish(msg->twist);
    };

    void manualOverrideCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        if(!arm_status_) {
            ROS_WARN_THROTTLE(5, "[Manual Override] Unable to use command, disarmed.");
            return;
        }
        last_manual_mode_ = ros::Time::now();
        manual_override_engaged_ = true;
        if(stair_controller_override_){
            mmpug_msgs::NavigateStairRequest cancel_msg;
            cancel_msg.stair_id = 0;
            cancel_msg.stair_count = 0;
            cancel_msg.stair_size = 0;
            cancel_msg.direction = true;
            stair_navigate_request_pub_.publish(cancel_msg);
        }
        stair_controller_override_ = false;
        spot_control_pub_.publish(msg->twist);
    };

    bool armDisarmCallback(
        std_srvs::SetBool::Request& request,
        std_srvs::SetBool::Response& response
    ) {
        response.message = arm_status_ ? "Was armed." : "Was disarmed.";
        response.success = true;
        if(DEBUG)
        {
            std::cout << "get an arm/disarm request !!!!!!!!!!!!!!!!!!!!! \n";
            std::cout << "current arm/disarm: "<< arm_status_ <<"\n";
            std::cout << "request.data: " << request.data << "\n\n";
        }

        if(request.data) {
            arm_request_status_ = arming_state_t::walking;
        } else {
            arm_request_status_ = arming_state_t::none;
        }
        return true;
    }

    void spotStairModeCallback(const std_msgs::Bool::ConstPtr& msg){
        ROS_INFO("[Stair Mode] Received request to toggle stair_mode: %d", msg->data);
        switchStairMode(msg->data);
    }

    void switchStairMode(bool data){
        std_msgs::Bool status;
        std_srvs::SetBool srvMsg;
        if(data){
            srvMsg.request.data = true;
            spot_stair_mode_service_.call(srvMsg);
            if(srvMsg.response.success){
                spot_stair_mode_enabled_ = true;
            }
            else{   
                ROS_WARN("[Stair Mode] Could not enable stair_mode");
                spot_stair_mode_enabled_ = false;
            }
        }
        else{
            srvMsg.request.data = false;
            spot_stair_mode_service_.call(srvMsg);
            if(srvMsg.response.success){
                spot_stair_mode_enabled_ = false;
            }
            else {   
                ROS_WARN("[Stair Mode] Could not disable stair_mode");
                spot_stair_mode_enabled_ = true;
            }
        }
    }

    void stairOverrideCallback(const std_msgs::Bool::ConstPtr& msg){
        ROS_INFO("[Stair Controller] Received Stair Controller Override: %d", msg->data);
        stair_controller_override_ = msg->data;
    }

private:
    double manual_mode_timeout_;
    double spot_feedback_timeout_;
    double arm_attempt_timeout_;

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ros::Subscriber manual_override_sub_;
    ros::Subscriber twist_control_sub_;
    ros::Subscriber spot_feedback_sub_;
    ros::Subscriber spot_lease_sub_;
    ros::Subscriber spot_power_sub_;
    ros::Subscriber spot_waypoint_sub_;
    ros::Subscriber spot_battery_sub_;
    ros::Subscriber spot_stair_waypoint_sub_;
    ros::Subscriber spot_stair_mode_sub_;
    ros::Subscriber spot_stair_controller_override_sub_;

    ros::Publisher armed_status_pub_;
    ros::Publisher manual_override_status_pub_;
    ros::Publisher spot_control_pub_;
    ros::Publisher spot_voltage_pub_;
    ros::Publisher spot_current_pub_;
    ros::Publisher spot_battery_percent_pub_;
    ros::Publisher spot_stair_mode_pub_;
    ros::Publisher stair_navigate_request_pub_;

    ros::ServiceServer arming_server_;
    ros::ServiceClient spot_claim_service_;
    ros::ServiceClient spot_release_service_;
    ros::ServiceClient spot_stand_service_;
    ros::ServiceClient spot_sit_service_;
    ros::ServiceClient spot_power_on_service_;
    ros::ServiceClient spot_power_off_service_;
    ros::ServiceClient spot_stair_mode_service_;

    arming_state_t arm_request_status_;
    arming_state_t arm_disarm_stage_;
    bool state_transitioned_;
    bool arm_status_;
    bool manual_override_engaged_;
    bool spot_leased_;
    bool spot_sitting_;
    bool spot_standing_;
    bool spot_powered_;
    bool spot_stair_mode_enabled_;
    bool joystick_mode_override_;
    bool stair_controller_override_;
    int try_count_;

    bool DEBUG;

    ros::Time last_manual_mode_;
    ros::Time last_spot_feedback_;
    ros::Time last_joystick_mode_;

    spot_msgs::TrajectoryGoal spotGoal, spotStairGoal;
    bool sendGoal;
    bool sendStairGoal;

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "mmpug_spot_interface");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    MMPUGSpotInterface node(nh, nhp);
    node.run();

    return 0;
}