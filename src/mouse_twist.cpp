#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"


class MouseTwist {

    public:
        MouseTwist(){ROS_INFO("Node created");}
        ~MouseTwist(){ROS_INFO("Node destroyed");}

        void init(ros::NodeHandle *nh)
        {
            twistPub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
            joySub = nh->subscribe("joy", 1, &MouseTwist::joyCallback, this);
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
        {   //2 axes : [left/right ; up/down]
            //1 button : ?
            twist_msg.linear.x  = linear_speed  *  msg.axes[1]; //forward / backward
            twist_msg.angular.z = angular_speed *  msg.axes[0]; //z rotation
            twistPub.publish(twist_msg);
        }
};

int main(int argc, char **argv)
{
    try
    {
        //ROS Node
        ros::init(argc, argv, "mouse_twist_node");
        ros::NodeHandle nh;

        //ROS Object
        MouseTwist mousetwist = MouseTwist();
        mousetwist.init(&nh);
    }

    catch(ros::Exception &e)
    {
        ROS_ERROR("Error occured: %s ", e.what());
    }

}
