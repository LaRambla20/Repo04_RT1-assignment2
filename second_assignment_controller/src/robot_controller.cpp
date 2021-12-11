#include "ros/ros.h" //include the header for using ROS functionalities -> every command or type of data that begins with ros:: is defined in this header
#include "sensor_msgs/LaserScan.h" //include the header corresponding to the message of type "sensor_msgs/LaserScan" that this node reads
// from the /base_scan topic
#include "geometry_msgs/Twist.h" //include the header corresponding to the message of type "geometry_msgs/Twist" that this node publishes
// on the /cmd_vel topic
#include "second_assignment_controller/ChangeVel.h" //include the header corresponding to the custom service of type 
// "second_assignment_controller/ChangeVel" that this node uses 
#include <iostream>  // include in order to use std::cout and std::cin commands
#include <vector> // include in order to define dynamic vectors with the notation std::vector

#define DAN_DISTANCE 0.65
#define DEFAULT_LINEAR_VEL 0.8
#define DEFAULT_ANGULAR_VEL 0.4

float coeff_l = 0.0; // coefficient that is modified inside the call-back function of the service (called whenever a request is sent by
// a client), used to increment/decrement the robot linear velocity
float coeff_a = 0.0; // coefficient that is modified inside the call-back function of the service (called whenever a request is sent by
// a client), used to increment/decrement the robot angular velocity

// Define the publisher as a global variable
ros::Publisher pub;

float find_minimum (std::vector<float> vector, int size){
    int i;
    float minimum = 10.0;
    for (i = 0; i < size; i++){
        if (vector[i] < minimum){
            minimum = vector[i];
        }
    }
    return minimum;
}

std::vector<std::string> change_direction (std::vector<float> dist_vector, float * linear_x, float * angular_z){

    std::string state_description;
    std::string substate_description  = "/";
    std::vector<std::string> state_vector;

    if (dist_vector[2] > DAN_DISTANCE && dist_vector[3] > DAN_DISTANCE && dist_vector[1] > DAN_DISTANCE){
        state_description = "case 1 - nothing";
        *linear_x = coeff_l*DEFAULT_LINEAR_VEL;
        *angular_z = 0;
    }
    else if (dist_vector[2] < DAN_DISTANCE && dist_vector[3] > DAN_DISTANCE && dist_vector[1] > DAN_DISTANCE){
        state_description = "case 2 - front";
        if (dist_vector[0] < DAN_DISTANCE && dist_vector[4] > DAN_DISTANCE){
            substate_description = "case 2a - right";
            *linear_x = 0;
            *angular_z = coeff_a*DEFAULT_ANGULAR_VEL;
        }
        else if (dist_vector[0] > DAN_DISTANCE && dist_vector[4] < DAN_DISTANCE){
            substate_description = "case 2b - left";
            *linear_x = 0;
            *angular_z = -coeff_a*DEFAULT_ANGULAR_VEL;
        }
        else{
            substate_description = "case 2c - right and left";
            *linear_x = 0;
            *angular_z = coeff_a*DEFAULT_ANGULAR_VEL;
        }
    }
    else if (dist_vector[2] > DAN_DISTANCE && dist_vector[3] > DAN_DISTANCE && dist_vector[1] < DAN_DISTANCE){
        state_description = "case 3 - fright";
        *linear_x = 0;
        *angular_z = coeff_a*DEFAULT_ANGULAR_VEL;
    }
    else if (dist_vector[2] > DAN_DISTANCE && dist_vector[3] < DAN_DISTANCE && dist_vector[1] > DAN_DISTANCE){
        state_description = "case 4 - fleft";
        *linear_x = 0;
        *angular_z = -coeff_a*DEFAULT_ANGULAR_VEL;
    }
    else if (dist_vector[2] < DAN_DISTANCE && dist_vector[3] > DAN_DISTANCE && dist_vector[1] < DAN_DISTANCE){
        state_description = "case 5 - front and fright";
        *linear_x = 0;
        *angular_z = coeff_a*DEFAULT_ANGULAR_VEL;
    }
    else if (dist_vector[2] < DAN_DISTANCE && dist_vector[3] < DAN_DISTANCE && dist_vector[1] > DAN_DISTANCE){
        state_description = "case 6 - front and fleft";
        *linear_x = 0;
        *angular_z = -coeff_a*DEFAULT_ANGULAR_VEL;
    }
    else if (dist_vector[2] < DAN_DISTANCE && dist_vector[3] < DAN_DISTANCE && dist_vector[1] < DAN_DISTANCE){
        state_description = "case 7 - front and fleft and fright";
        if (dist_vector[0] < DAN_DISTANCE && dist_vector[4] > DAN_DISTANCE){
            substate_description = "case 7a - right";
            *linear_x = 0;
            *angular_z = coeff_a*DEFAULT_ANGULAR_VEL;
        }
        else if (dist_vector[0] > DAN_DISTANCE && dist_vector[4] < DAN_DISTANCE){
            substate_description = "case 7b - left";
            *linear_x = 0;
            *angular_z = -coeff_a*DEFAULT_ANGULAR_VEL;
        }
        else{
            substate_description = "case 7c - right and left";
            *linear_x = 0;
            *angular_z = coeff_a*DEFAULT_ANGULAR_VEL;
        }
    }
    else if (dist_vector[2] > DAN_DISTANCE && dist_vector[3] < DAN_DISTANCE && dist_vector[1] < DAN_DISTANCE){
        state_description = "case 8 - fleft and fright";
        *linear_x = coeff_l*DEFAULT_LINEAR_VEL/2;
        *angular_z = 0;
    }
    else{
        state_description = "unknown case";
    }

    state_vector = {
        state_description,
        substate_description
    };

    return state_vector;
}


bool obtaincoeffCallback (second_assignment_controller::ChangeVel::Request &req, second_assignment_controller::ChangeVel::Response &res){ // function that is
// executed every time that a request message of the /change_vel custom service

// SERVER PART

    if (req.command == 's'){
        if (coeff_l == 0.0 && coeff_a == 0.0){
            coeff_l = 1.0;
            coeff_a = 1.0;
            res.action = "motion started";
        }
        else{
            res.action = "WARNING: cannot start the motion: motion already started";
        }
    }

    else if (req.command == 'd'){
        if (coeff_l == 0.0){
            res.action = "WARNING: start the motion first";
        }
        else{
            coeff_l = coeff_l*1.5;
            res.action = "linear velocity incremented";
        }
    }
    else if (req.command == 'a'){
        if (coeff_l == 0.0){
            res.action = "WARNING: start the motion first";
        }
        else{
            coeff_l = coeff_l/1.5;
            res.action = "linear velocity decremented";
        }
    }
    else if (req.command == 'c'){
        if (coeff_a == 0.0){
            res.action = "WARNING: start the motion first";
        }
        else{
            coeff_a = coeff_a*1.2;
            res.action = "angular velocity incremented";
        }
    }
    else if (req.command == 'z'){
        if (coeff_a == 0.0){
            res.action = "WARNING: start the motion first";
        }
        else{
            coeff_a = coeff_a/1.2;
            res.action = "angular velocity decremented";
        }
    }
    else if (req.command == 'r'){
        coeff_l = 0.0;
        coeff_a = 0.0;
        res.action = "position and velocities reset";
    }
    return true;
}

void robotCallback(const sensor_msgs::LaserScan::ConstPtr& msg) //function that is executed every time that a message is published on the /base_scan topic
{

// N.B. The laser scanners output a vector of 720 elements, that is a field of the sensor_msgs/LaserScan message published on the /base_scan topic.
//      This vector contains the distances from the walls in an angular range that goes from 0 degrees (on the right of the robot) to 180 degrees. So the element 
//      ranges[0] is nothing more than the distance from the wall that is at 0 degrees with respect to the robot; while ranges[719] is the distance from the wall
//      at 180 degrees with respect to the robot. If this vector is divided in 5 subevectors, 5 regions will be identified:
//      - region1 - right region: region that ranges from 0 to 36 degrees (that is: from ranges[0] to ranges[143])
//      - region2 - front-right region: region that ranges from 36 to 72 degrees (that is: from ranges[144] to ranges[287])
//      - region3 - front region: region that ranges from 72 to 108 degrees (that is: from ranges[288] to ranges[431])
//      - region4 - front-left region: region that ranges from 108 to 144 degrees (that is: from ranges[432] to ranges[575])
//      - region5 - left region: region that ranges from 144 to 180 degrees (that is: from ranges[576] to ranges[719])
//      By taking the minimum elements of these subvectors, 5 representative distances will be obtained: each of those is the most dangerous distances within the
//      region


// SUBSCRIBER PART

int i;

std::vector<float> right_region(msg->ranges.begin(), msg->ranges.begin()+msg->ranges.size()/5);
std::vector<float> fright_region(msg->ranges.begin()+msg->ranges.size()/5, msg->ranges.begin()+2*(msg->ranges.size()/5));
std::vector<float> front_region(msg->ranges.begin()+2*(msg->ranges.size()/5), msg->ranges.begin()+3*(msg->ranges.size()/5));
std::vector<float> fleft_region(msg->ranges.begin()+3*(msg->ranges.size()/5), msg->ranges.begin()+4*(msg->ranges.size()/5));
std::vector<float> left_region(msg->ranges.begin()+4*(msg->ranges.size()/5), msg->ranges.begin()+5*(msg->ranges.size()/5));

float smallest_wall_dist_right;
float smallest_wall_dist_fright;
float smallest_wall_dist_front;
float smallest_wall_dist_fleft;
float smallest_wall_dist_left;

smallest_wall_dist_right = find_minimum (right_region, right_region.size());
smallest_wall_dist_fright = find_minimum (fright_region, fright_region.size());
smallest_wall_dist_front = find_minimum (front_region, front_region.size());
smallest_wall_dist_fleft = find_minimum (fleft_region, fleft_region.size());
smallest_wall_dist_left = find_minimum (left_region, left_region.size());


std::vector<float> smallest_wall_dist = {
    smallest_wall_dist_right,
    smallest_wall_dist_fright,
    smallest_wall_dist_front,
    smallest_wall_dist_fleft,
    smallest_wall_dist_left
};

for(i=0; i<smallest_wall_dist.size(); i++){
    ROS_INFO("smallest_wall_dist %d-th element: [%f]", i, smallest_wall_dist[i]);
}

// PUBLISHER PART

float linear_x = 0.0; 
float angular_z = 0.0;
std::vector<std::string> state;
geometry_msgs::Twist my_vel;


state = change_direction(smallest_wall_dist, &linear_x, &angular_z);

std::cout << "\x1b[34m" "((" << state[0] << "))" "\x1b[0m" << std::endl;
std::cout << "\x1b[34m" "((" << state[1] << "))" "\x1b[0m" << std::endl;

my_vel.linear.x = linear_x;
my_vel.angular.z = angular_z;

pub.publish(my_vel);
}



int main (int argc, char **argv)
{
// Initialize the node passed as argument in the rosrun call/command from the terminal and setup the NodeHandle for handling the communication with the ROS system
ros::init(argc, argv, "robot_controller_node"); 
ros::NodeHandle nh; 
// Initialize the publisher that publishes on the "/cmd_vel" topic
pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
// Initialize and define the subscriber that subscribes to the "base_scan" topic and assign the corresponding call-back function
ros::Subscriber sub = nh.subscribe("/base_scan", 1, robotCallback);
// Initialize and define the server that answers to requests belonging to the "/change_vel" service and assign the corresponding call-back function
ros::ServiceServer server1 = nh.advertiseService("/change_vel", obtaincoeffCallback);
ros::spin(); // continuously check if there is some published message on the topic
return 0;
}