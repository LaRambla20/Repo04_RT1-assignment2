#include "ros/ros.h" //include the header for using ROS functionalities -> every command or type of data that begins with ros:: is defined in this header
#include "second_assignment_controller/ChangeVel.h" //include the header corresponding to the custom service of type 
// "second_assignment_controller/ChangeVel" that this node uses 
#include "std_srvs/Empty.h" //include the header corresponding to the service of type "std_srvs/Empty" that this node uses
#include <iostream> // include in order to use std::cout and std::cin commands


int main (int argc, char **argv)
{
// Initialize the node passed as argument in the rosrun call/command from the terminal and setup the NodeHandle for handling the communication with the ROS system
ros::init(argc, argv, "robot_gui_node"); 
ros::NodeHandle nh;
// Initialize and define the client that sends requests belonging to the /change_vel service
ros::ServiceClient client1 = nh.serviceClient<second_assignment_controller::ChangeVel>("/change_vel");
second_assignment_controller::ChangeVel srv1; //define the custom service variable srv1 of type ChangeVel (type of service retrieved from the second_assignment_controller)

ros::ServiceClient client2 = nh.serviceClient<std_srvs::Empty>("/reset_positions");
std_srvs::Empty srv2;

std::string command;
int iterations_count_l = 0;
int iterations_count_a = 0;

std::cout << "====================================================================" << std::endl;
std::cout << "COMMAND LEGEND:" << std::endl;
std::cout << "\x1b[33m" "- 's': start the motion" "\x1b[0m" << std::endl;
std::cout << "- 'd': increment the robot linear velocity by multiplying it by 1.5" << std::endl;
std::cout << "- 'a': decrement the robot linear velocity by dividing it by 1.5" << std::endl;
std::cout << "- 'c': increment the robot angular velocity by multiplying it by 1.2" << std::endl;
std::cout << "- 'z': decrement the robot angular velocity by dividing it by 1.2" << std::endl;
std::cout << "- 'r': reset the robot position and velocities" << std::endl;
std::cout << "- 'q': quit this GUI node" << std::endl;
std::cout << "====================================================================" << std::endl;

while(1){

    std::cout << "Please enter a command:" << std::endl;
    getline (std::cin, command);

    if (command == "s"){
        client1.waitForExistence();  // check if the server is running and active
        srv1.request.command = 's';
        client1.call(srv1);

        if (srv1.response.action == "WARNING: cannot start the motion: motion already started"){
            std::cout << "\x1b[31m" << srv1.response.action << "\x1b[0m" << std::endl;
        }
        else{
            std::cout << "\x1b[32m" << srv1.response.action << "\x1b[0m" << std::endl;
        }
    }

    else if (command == "d"){
        client1.waitForExistence();
        srv1.request.command = 'd';
        client1.call(srv1);
        
        if (srv1.response.action == "WARNING: start the motion first"){
            std::cout << "\x1b[31m" << srv1.response.action << "\x1b[0m" << std::endl;
            iterations_count_l = 0;
        }
        else{
            std::cout << "\x1b[32m" << srv1.response.action << "\x1b[0m" << std::endl;
            iterations_count_l = iterations_count_l + 1;
        }
    }

    else if (command == "a"){
        client1.waitForExistence();
        srv1.request.command = 'a';
        client1.call(srv1);

        if (srv1.response.action == "WARNING: start the motion first"){
            std::cout << "\x1b[31m" << srv1.response.action << "\x1b[0m" << std::endl;
            iterations_count_l = 0;
        }
        else{
            std::cout << "\x1b[32m" << srv1.response.action << "\x1b[0m" << std::endl;
            iterations_count_l = iterations_count_l - 1;
        }
    }

    else if (command == "c"){
        client1.waitForExistence();
        srv1.request.command = 'c';
        client1.call(srv1);

        if (srv1.response.action == "WARNING: start the motion first"){
            std::cout << "\x1b[31m" << srv1.response.action << "\x1b[0m" << std::endl;
            iterations_count_a = 0;
        }
        else{
            std::cout << "\x1b[32m" << srv1.response.action << "\x1b[0m" << std::endl;
            iterations_count_a = iterations_count_a + 1;
        }
    }

    else if (command == "z"){
        client1.waitForExistence();
        srv1.request.command = 'z';
        client1.call(srv1);

        if (srv1.response.action == "WARNING: start the motion first"){
            std::cout << "\x1b[31m" << srv1.response.action << "\x1b[0m" << std::endl;
            iterations_count_a = 0;
        }
        else{
            std::cout << "\x1b[32m" << srv1.response.action << "\x1b[0m" << std::endl;
            iterations_count_a = iterations_count_a - 1;
        }
    }

    else if (command == "r"){
        client1.waitForExistence();
        srv1.request.command = 'r';
        client1.call(srv1);

        std::cout << "\x1b[32m" << srv1.response.action << "\x1b[0m" << std::endl;
        iterations_count_l = 0;
        iterations_count_a = 0;


        client2.waitForExistence();
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


    if (iterations_count_l > 3){
        std::cout << "\x1b[31m" "WARNING: high linear velocity -> control might fail" "\x1b[0m" << std::endl;
    }
    if (iterations_count_a > 3){
        std::cout << "\x1b[31m" "WARNING: high angular velocity -> control might fail" "\x1b[0m" << std::endl;
    }

    std::cout << "\n-----------------------------------------------------------------\n" << std::endl;
    
}

return 0;
}