/**
 *  Image alignment using LSD's direct photometric error
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/highgui/highgui.hpp>
#include "lsd.h"

using namespace std;

class Camera
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr_depth;
    const std::string OPENCV_WINDOW = "Image window";
    MyLSD lsd;
    ros::Subscriber sub;          
    ros::Subscriber cam_info_sub_;          
    cv::Mat mono8_depth;
    cv::Mat depth;
    
    image_geometry::PinholeCameraModel model_;

    double baseline = 0.0;
    unsigned int height = 640;
    unsigned int width = 512;
    double cx_, cy_, fx_, fy_;
    bool got_cam_info;

    public:
        Camera(): it_(nh_)
        {
            // Subscrive to input video feed and publish output video feed
            cam_info_sub_ = nh_.subscribe("/narrow_stereo/left/camera_info", 1,&Camera::getCamInfo,this);
            image_sub_ = it_.subscribe("/narrow_stereo/left/image_rect", 1, 
                                &Camera::imageCb, this);
            sub = nh_.subscribe("/narrow_stereo/disparity", 1, &Camera::dispCb, this);
            image_pub_ = it_.advertise("/image_converter/output_video", 1);
            cv::namedWindow(OPENCV_WINDOW);              

            // TODO: FIXME
            cx_ = 317.20617294311523;
            cy_ = 233.2914752960205;
            fx_ = fy_ = 307.4838344732113;

        }

        ~Camera()
        {
            cv::destroyWindow(OPENCV_WINDOW);
        }

        void dispCb(const stereo_msgs::DisparityImage& msg)
        {
            try
                {
                    cv_ptr_depth = cv_bridge::toCvCopy(msg.image);
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return;
                }
            mono8_depth;
            depth = cv_ptr_depth->image;
            depthToCV8UC1(depth,mono8_depth);
            lsd.get_imdepth(cv_ptr_depth);
            cv::imshow("4", mono8_depth);

            image_pub_.publish(cv_ptr_depth->toImageMsg());
         }

        void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
              //Process images
            if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
                    mono8_img = cv::Mat(float_img.size(), CV_8UC1);
            }
              cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
                //The following doesn't work due to NaNs
                //double minVal, maxVal; 
                //minMaxLoc(float_img, &minVal, &maxVal);
                //ROS_DEBUG("Minimum/Maximum Depth in current image: %f/%f", minVal, maxVal);
                //mono8_img = cv::Scalar(0);
                //cv::line( mono8_img, cv::Point2i(10,10),cv::Point2i(200,100), cv::Scalar(255), 3, 8 );
        }

        void getCamInfo(const sensor_msgs::CameraInfo::ConstPtr& msg_info)
        {
            double downsample_scale = 1.0;
            if(got_cam_info)
                    return;
            model_.fromCameraInfo ( msg_info  );
            ROS_INFO_ONCE("Cam Info Recvd Fx Fy Cx Cy: %f %f , %f %f",model_.fx(),model_.fy(),model_.cx(),model_.cy());
            cx_ = model_.cx()/downsample_scale;
            cy_ = model_.cy()/downsample_scale;
            fx_ = fy_ = model_.fx()/downsample_scale;
            width = msg_info->width/downsample_scale;
            height = msg_info->height/downsample_scale;
            double baseline_temp = -msg_info->P[3]/msg_info->P[0];
            if(baseline_temp != 0.0)
                   baseline = baseline_temp;
            baseline *=downsample_scale;
            ROS_WARN("Transformed Cam Info Recvd Fx Fy Cx Cy: %f %f , %f %f Baseline: %f with downsamplescale: %f",model_.fx(),model_.fy(),model_.cx(),model_.cy(),baseline,downsample_scale);
            got_cam_info = true;
        }

        void imageCb(const sensor_msgs::ImageConstPtr& msg)
        {
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                                        
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
                                  
            lsd.get_im(cv_ptr);
            cv::Mat result = lsd_alignment();
            cv::imshow(OPENCV_WINDOW, result);
            cv::imshow("2", lsd.get_imj());
            cv::imshow("3", lsd.get_imi());
            // Update GUI Window
            cv::waitKey(3);

            // Output modified video stream
            image_pub_.publish(cv_ptr->toImageMsg());
        }

        cv::Mat lsd_alignment()
        {
            return lsd.get_gradient();
        }
};

void disparityImageCallback(const stereo_msgs::DisparityImage& msg)
{
    cout << "HI\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lsd_alignment");
    Camera ic;
    //ros::NodeHandle nh;
    ros::spin();
    return 0;
}
