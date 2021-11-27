#include "ros/ros.h" //include the header for using ROS functionalities -> every command or type of data that begins with ros:: is defined in this header
// #include "turtlesim/Pose.h" //include the header corresponding to the message this node uses
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h" //include the header corresponding to the message this node uses

ros::Publisher pub; //define the global variable pub of type Publisher

void robotCallback(const nav_msgs::Odometry::ConstPtr& msg) //function that is executed every time that something is published in the topic
{
// Subscriber part
ROS_INFO("Odometry subscriber@[%f, %f, %f]",
msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z); // print the field x, y and theta of the message msg (->: dereferencing operator)

// Publisher part
geometry_msgs::Twist my_vel; //define the message variable my_vel of type Twist (type of message retrieved from the geometry_msgs package)
my_vel.linear.x = 1.0; //assign to the field x of the field linear of the message my_vel of type Twist the value 1.0
// my_vel.angular.z = 1.0; //assign to the field x of the field linear of the message my_vel of type Twist the value 1.0
pub.publish(my_vel); //publish the message my_vel of type Twist on the topic defined in the initializationn of the publisher (turtle1/cmd_vel)
}



int main (int argc, char **argv) //argc is an integer, argv is a pointer to a pointer to a character
{
// Initialize the node, setup the NodeHandle for handling the communication with the ROS
//system
ros::init(argc, argv, "robot_subscriber"); //initialize the node passed as argument in the rosrun call/command from terminal (argv contains: nameofthepackage nameofthenode),
// argc instead is not given as an argument from the terminal, it is just the number of elements of argv
ros::NodeHandle nh; //kind of a node manager (present only in C++ implemented nodes): it implements the ROS functionalities to create a publisher-subscriber paradigm
//or a server-client paradigm
// Setup the publisher
pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1); //initialize and define the (node as a) publisher: name of the variable publisher = node handle object
//<type of message that it will publish>("name of the topic",buffer dimension in bytes)
// Define the subscriber to turtle's position
ros::Subscriber sub = nh.subscribe("/odom", 1,robotCallback); //initialize and define the (node as a) subscriber: type of the varaible subscriber
// name of the variable subscriber = node handle object ("name of the topic",buffer dimension in bytes, name of the callback function that will be executed every time that
// something is published in the topic)
ros::spin(); //like a while(1) loop: it is used to continuously check if there is some published message on the topic
return 0; //since the main is a function that returns an integers
}

