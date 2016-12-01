#include "camera.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lsd_alignment");
    Camera ic;
    //ros::NodeHandle nh;
    ros::spin();
    return 0;
}
