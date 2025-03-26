#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <boost/circular_buffer.hpp>
#include <sensor_msgs/BatteryState.h>

class MMPUGBatteryEstimator {
public:
    MMPUGBatteryEstimator(
        ros::NodeHandle& nh,
        ros::NodeHandle& nhp
    ):
        nh_(nh),
        nhp_(nhp)
    {
        odometry_sub_ = nh_.subscribe("integrated_to_init", 1, &MMPUGBatteryEstimator::odometryCallback, this);
        battery_status_sub_ = nh_.subscribe("mavros/battery", 1, &MMPUGBatteryEstimator::batteryStateCallback, this);
        rc_battery_percent_pub_ =  nh.advertise<std_msgs::Float32>("battery_percentage", 1);
        velocity = 0;
        voltage_offset = 0;
        buffer = boost::circular_buffer<double>{10};
    };

    void run() {
        ros::Rate r(5);

        while(ros::ok()) {
            ros::spinOnce();
            r.sleep();
        }

    }
    
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
        velocity = abs(msg->twist.twist.linear.x);

        if(velocity <= 0.5) 
            voltage_offset = 0;
        else if (velocity <= 1)
            voltage_offset = 0.1;
        else if (velocity <= 1.5)
            voltage_offset = 0.3;
        else if (velocity <= 2)
            voltage_offset = 0.4;
        else if (velocity <= 2.5)
            voltage_offset = 0.5;
        else if (velocity <= 3)
            voltage_offset = 0.6;
        else if (velocity <= 3.5)
            voltage_offset = 0.7;
        else if (velocity <= 4)
            voltage_offset = 0.8;
        else if (velocity <= 4.5)
            voltage_offset = 0.9;
        else if (velocity <= 5)
            voltage_offset = 1.0;
        else if (velocity <= 5.5)
            voltage_offset = 1.1;
        else if (velocity <= 6)
            voltage_offset = 1.2;
        else if (velocity < 6.5)
            voltage_offset = 1.2;

        // ROS_INFO_STREAM("V: " << velocity << "O: " << voltage_offset);
    }

    void batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg){
        float voltage = (msg->voltage + voltage_offset)/8;
        float percent;
        
        buffer.push_back(voltage);
        float curr_voltage = 0;
        for(int i = 0; i < buffer.size(); i++){
            curr_voltage = curr_voltage + buffer[i];
        }
        curr_voltage = curr_voltage/buffer.size();
        // ROS_INFO_STREAM("Voltage: " << curr_voltage);

        if(curr_voltage > 3.93){
            percent = 85 + (15*(curr_voltage - 3.93)/(4.2 - 3.93));

        }
        else if(curr_voltage < 3.65){
            percent = 10*(curr_voltage - 3.6)/(3.65 - 3.6);
        }
        else{
            percent = 10 + (75*(curr_voltage - 3.65)/(3.93 - 3.7));
        }

        if(percent > 100)
            percent = 100;
        if(percent < 0)
            percent = 0;

        std_msgs::Float32 battery_msg;
        battery_msg.data = percent;
        rc_battery_percent_pub_.publish(battery_msg);
    }



private:

    double velocity;
    double voltage_offset;
    boost::circular_buffer<double> buffer;
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ros::Subscriber odometry_sub_;
    ros::Subscriber battery_status_sub_;

    ros::Publisher rc_battery_percent_pub_;
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "mmpug_battery_estimator");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    MMPUGBatteryEstimator node(nh, nhp);
    node.run();

    return 0;
}