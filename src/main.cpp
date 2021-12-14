#include "../include/usar.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"search_rescue_node");
    ros::NodeHandle nh("~");
    Urban_Search_And_Rescue sar(&nh);
    sar.get_targets();
    sar.search();
    sar.rescue();
    return 0;
}