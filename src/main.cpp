#include "../include/usar.h"
#include <ctime>
int main(int argc, char **argv){
    ros::init(argc, argv, "search_rescue_node"); //initialize the ros node
    ros::NodeHandle nh("~"); //initialize the node handle
    Urban_Search_And_Rescue sar(&nh); //create an object of Urban_Search_And_Rescue
    time_t start_time = time(NULL); //time object to know the start time of the simulation
    sar.get_targets(); //calls the get_targets function of sar object to store the target locations from rosparam server.
    sar.search(); //initiates the search operation
    sar.rescue(); //initiates the rescue operation
    time_t end_time = time(NULL); //time object to know the end time of the simuation
    std::cout << "Total Simulation Time {minutes}: " << (end_time - start_time) / 60.0 <<"\n"; //calculates the total simulation time in minutes and displays that in console
    return 0;
}