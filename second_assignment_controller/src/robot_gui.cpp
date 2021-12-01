#include "ros/ros.h" //include the header for using ROS functionalities -> every command or type of data that begins with ros:: is defined in this header
#include "geometry_msgs/Twist.h" //include the header corresponding to the message this node uses
#include "second_assignment_controller/ChangeVel.h"  //include the header corresponding to the custom service messge this node uses
#include "std_srvs/Empty.h"
#include <iostream>     // std::cout


int main (int argc, char **argv)
{
// Initialize the node, setup the NodeHandle for handling the communication with the ROS
//system
ros::init(argc, argv, "robot_client"); //initialize the node passed as argument in the rosrun call/command from terminal (argv contains: nameofthepackage nameofthenode),
// argc instead is not given as an argument from the terminal, it is just the number of elements of argv
ros::NodeHandle nh; //kind of a node manager (present only in C++ implemented nodes): it implements the ROS functionalities to create a publisher-subscriber paradigm
//or a server-client paradigm
// Setup the publisher
ros::ServiceClient client1 = nh.serviceClient<second_assignment_controller::ChangeVel>("/change_vel"); //initialize and define the (node as a) client: type of the variable client
// name of the variable client = node handle object <type of service that it will use> ("name of the service")
second_assignment_controller::ChangeVel srv1; //define the service variable srv1 of type Spawn (type of service retrieved from the turtlesim package)

ros::ServiceClient client2 = nh.serviceClient<std_srvs::Empty>("/reset_positions");

std_srvs::Empty srv2;

std::string command;
int iterations_count = 0;

std::cout << "=================================================================" << std::endl;
std::cout << "COMMAND LEGEND:" << std::endl;
std::cout << "- 'd': increment the robot velocities by multiplying the linear\nvelocity by 1.5 and the angular velocity by 1.2" << std::endl;
std::cout << "- 'a': decrement the robot velocities by dividing the linear\nvelocity by 1.5 and the angular velocity by 1.2" << std::endl;
std::cout << "- 'r': reset the robot position to the initial one" << std::endl;
std::cout << "- 'q': quit this GUI node" << std::endl;
std::cout << "=================================================================" << std::endl;

while(1){

    std::cout << "Please enter a command:" << std::endl;
    std::cin >> command;

    if (command == "d"){
        client1.waitForExistence();  //to check if the server is running and active
        srv1.request.command = 'd';
        client1.call(srv1);

        std::cout << "\x1b[32m" << srv1.response.action << "\x1b[0m" << std::endl;
        iterations_count = iterations_count + 1;
    }

    else if (command == "a"){
        client1.waitForExistence();  //to check if the server is running and active
        srv1.request.command = 'a';
        client1.call(srv1);

        if (srv1.response.action == "cannot decrement the velocity"){
            std::cout << "\x1b[31m" << srv1.response.action << "\x1b[0m" << std::endl;
            iterations_count = 0;
        }
        else{
            std::cout << "\x1b[32m" << srv1.response.action << "\x1b[0m" << std::endl;
            iterations_count = iterations_count - 1;
        }
    }

    else if (command == "r"){
        client1.waitForExistence();  //to check if the server is running and active
        srv1.request.command = 'r';
        client1.call(srv1);

        std::cout << "\x1b[32m" << srv1.response.action << "\x1b[0m" << std::endl;
        iterations_count = 0;


        client2.waitForExistence();  //to check if the server is running and active
        client2.call(srv2); // since the call is empty, nothing should be assigned to the request field of srv2
    }

    else if (command == "q"){
        std::cout << "\x1b[32m" "exiting" "\x1b[0m" << std::endl;
        std::cout << "\n-----------------------------------------------------------------\n" << std::endl;
        return 0;
    }

    else{
        std::cout << "\x1b[31m" "Invalid command" "\x1b[0m" << std::endl;
    }


    if (iterations_count > 4){
        std::cout << "\x1b[31m" "WARNING: high velocity -> control might fail" "\x1b[0m" << std::endl;
    }

    std::cout << "\n-----------------------------------------------------------------\n" << std::endl;
    
}

return 0; //since the main is a function that returns an integer
}

