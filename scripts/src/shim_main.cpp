#include "test_depth.h"

int main(int argc, char** argv)
{
        ros::init(argc, argv, "test_depth");
        ros::NodeHandle nh("test_depth");
        test_depth driver(nh);
        try
        {
                driver.init();
                ros::spin();
        }
        catch(...) {
                ros::shutdown();
        }


        return 0;
}
