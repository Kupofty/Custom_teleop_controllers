#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

class X638VTwist {

    public:
        X638VTwist(){ROS_INFO("Node created");}
        ~X638VTwist(){ROS_INFO("Node destroyed");}

        void init(ros::NodeHandle *nh)
        {
            twistPub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
            joySub = nh->subscribe("joy", 1, &X638VTwist::joyCallback, this);
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
            //axes[6] : [ Left joystick L/R ; Left joystick U/D ;  Right joystick U/D ;  Right joystick L/R ; Button L/R ; Button U/D]

            //[1 ; 0]
            //buttons[12] : [1 ; 2; 3; 4; 5; 6; 7; 8; 9; 10; Left Joystick click ; Right joystick click ]


            //Change linear speed
            if(msg.buttons[1] == 1 || msg.buttons[3] == 1)
            {
                linear_speed += msg.buttons[1]; //button 2
                linear_speed -= msg.buttons[3]; // button 4
                ROS_INFO("Linear speed: [%f]", linear_speed);
            }
            if(msg.buttons[10]==1) //reset linear speed
            {
                linear_speed = 1;
                ROS_INFO("Linear speed: [%f]", linear_speed);
            }

            //Change angular speed
            if(msg.buttons[0] == 1 || msg.buttons[2] == 1)
            {
                angular_speed += msg.buttons[0]; //button 1
                angular_speed -= msg.buttons[2]; //button3
                ROS_INFO("Angular speed: [%f]", angular_speed);
            }
            if(msg.buttons[11]==1) //reset angular speed
            {
                angular_speed = 1;
                ROS_INFO("Angular speed: [%f]", angular_speed);
            }

            twist_msg.linear.x  = linear_speed  *  msg.axes[1]; //forward / backward
            twist_msg.linear.y  = linear_speed  *  msg.axes[0]; //straff
            twist_msg.linear.z  = linear_speed  * (msg.buttons[9]- msg.axes[7]); //up down
            twist_msg.angular.x = angular_speed * (msg.buttons[8] - msg.buttons[6]); //roll
            twist_msg.angular.y = angular_speed *  msg.axes[2]; //pitch
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
        X638VTwist X638V_twist = X638VTwist();
        X638V_twist.init(&nh);
    }

    catch(ros::Exception &e)
    {
        ROS_ERROR("Error occured: %s ", e.what());
    }

}



