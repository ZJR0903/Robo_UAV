
#include  "unitree_nav_ros/libUnitreeNav.h"

int main(int argc,  char ** argv) 
{
    ros::init(argc,  argv,  "unitree_nav_node");

    UnitreeNav MyUnitreeNav;
    MyUnitreeNav.Manager();

    return 0;
}
