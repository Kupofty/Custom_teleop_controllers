#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

class PS3Twist {

    public:
        PS3Twist(){ROS_INFO("Node created");}
        ~PS3Twist(){ROS_INFO("Node destroyed");}

        void init(ros::NodeHandle *nh)
        {
            twistPub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
            joySub = nh->subscribe("joy", 1, &PS3Twist::joyCallback, this);
            ros::spin();
        }

    private:
        //Variables
        float linear_speed = 1.0;
        float angular_speed = 1.0;

        //Subscribers
        geometry_msgs::Twist twist_msg;
        ros::Subscriber joySub;

        //Publishers
        ros::Publisher twistPub;

        //Callbacks
        void joyCallback(const sensor_msgs::Joy &msg)
        {
            //[1 : -1]
            //axes[6] : [ Left joystick L/R ; Left joystick U/D ; L2 F/P ; Right joystick L/R ; Right joystick U/D ; R2 F/P ;

            //[1 ; 0]
            //buttons[17] : [ CROSS ; CIRCLE ; TRIANGLE ; SQUARE ; L1 ; R1 ; L2 ; R2 ; SELECT ; START ; PS ; LEFT CLICK ; RIGHT CLICK ; UP ; DOWN; LEFT ; RIGHT ]


            //Change linear speed
            if(msg.buttons[2] == 1 || msg.buttons[1] == 1)
            {
                linear_speed += msg.buttons[2]; //triangle
                linear_speed -= msg.buttons[1]; // Circle
                ROS_INFO("Linear speed: [%f]", linear_speed);
            }
            if(msg.buttons[11]==1) //reset linear speed
            {
                linear_speed = 1;
                ROS_INFO("Linear speed: [%f]", linear_speed);
            }

            //Change angular speed
            if(msg.buttons[3] == 1 || msg.buttons[0] == 1)
            {
                angular_speed += msg.buttons[3]; //square
                angular_speed -= msg.buttons[0]; //cross
                ROS_INFO("Angular speed: [%f]", angular_speed);
            }
            if(msg.buttons[12]==1) //reset angular speed
            {
                angular_speed = 1;
                ROS_INFO("Angular speed: [%f]", angular_speed);
            }

            twist_msg.linear.x  = linear_speed  *  msg.axes[1]; //forward / backward
            twist_msg.linear.y  = linear_speed  *  msg.axes[0]; //straff
            twist_msg.linear.z  = linear_speed  * (msg.axes[2] - msg.axes[5])/2; //up down
            twist_msg.angular.x = angular_speed * (msg.buttons[4] - msg.buttons[5]); //roll
            twist_msg.angular.y = angular_speed *  msg.axes[4]; //pitch
            twist_msg.angular.z = angular_speed *  msg.axes[3]; //z rotation

            twistPub.publish(twist_msg);
        }
};

int main(int argc, char **argv)
{
    try
    {
        //ROS Node
        ros::init(argc, argv, "ps3_twist_node");
        ros::NodeHandle nh;

        //ROS Object
        PS3Twist ps3twist = PS3Twist();
        ps3twist.init(&nh);
    }

    catch(ros::Exception &e)
    {
        ROS_ERROR("Error occured: %s ", e.what());
    }

}
