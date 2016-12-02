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
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>


using namespace std;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

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
    ros::Subscriber depth_sub;          
    ros::Subscriber cam_info_sub_;          
    ros::Publisher expansion_cloud_pub;

    cv::Mat mono8_depth;
    cv::Mat depth;
    cv::Mat depth_mat;
    image_geometry::PinholeCameraModel model_;

    double baseline = 17.5;
    unsigned int height = 640;
    unsigned int width = 512;
    double cx_, cy_, fx_, fy_;
    bool got_cam_info = false;

    public:
        Camera(): it_(nh_)
        {
            image_sub_ = it_.subscribe("/narrow_stereo/left/image_rect", 1, &Camera::imageCb, this);
            depth_sub = nh_.subscribe("/narrow_stereo/points2",1, &Camera::getDepthCb, this);
            cam_info_sub_ = nh_.subscribe("/narrow_stereo/left/camera_info", 1,&Camera::getCamInfo,this);
            
            expansion_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/output/cloud", 1);
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

        /*void dispCb(const stereo_msgs::DisparityImage::ConstPtr& msg)
        {
            try
                {
                    cv_ptr_depth = cv_bridge::toCvCopy(msg->image);
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
            //getDepth(msg);

            image_pub_.publish(cv_ptr_depth->toImageMsg());
         }

        void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
              //Process images
            if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
                    mono8_img = cv::Mat(float_img.size(), CV_8UC1);
            }
              cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
        }*/

        void getCamInfo(const sensor_msgs::CameraInfo::ConstPtr& msg_info)
        {
            double downsample_scale = 1.0;
            if(got_cam_info)
                    return;
            model_.fromCameraInfo ( msg_info  );
            ROS_INFO_ONCE("Cam Info Recvd Fx Fy Cx Cy: \n%f %f , %f %f",model_.fx(),model_.fy(),model_.cx(),model_.cy());
            cx_ = model_.cx()/downsample_scale;
            cy_ = model_.cy()/downsample_scale;
            fx_ = fy_ = model_.fx()/downsample_scale;
            width = msg_info->width/downsample_scale;
            height = msg_info->height/downsample_scale;
            double baseline_temp = -msg_info->P[3]/msg_info->P[0];
            if(baseline_temp != 0.0)
                   baseline = baseline_temp;
            baseline *=downsample_scale;
            ROS_WARN("Transformed Cam Info Recvd Fx Fy Cx Cy: \n%f %f , %f %f \nBaseline: %f with downsamplescale: %f",model_.fx(),model_.fy(),model_.cx(),model_.cy(),baseline,downsample_scale);
            got_cam_info = true;
        }

        void getDepthCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_in)
        {
            cout << "EHRE\n";
            sensor_msgs::PointCloud2 cloud_msg = *cloud_msg_in;

            unsigned int _width=cloud_msg.width;
            unsigned int _height=cloud_msg.height;
            unsigned int _depth_id = cloud_msg.header.frame_id;

            depth_mat = cv::Mat::zeros(_height,_width, CV_16UC1);

            uint16_t* pd = (uint16_t*) depth_mat.data;        

            int ind = 0;
                                
            string s1("z");
            // // this call also resizes the data structure according to the given width, height and fields
            sensor_msgs::PointCloud2Iterator<float>   iter_z(cloud_msg, s1);
            for(; iter_z != iter_z.end(); ++iter_z)
            {
                int u = ind/_width;
                int v = ind%_width;

                // get z
                float pz = *iter_z;
                *pd = (uint16_t)(1000.00*pz);

                pd++;
                ind++;
            }
            cout << depth_mat.rows << "," << depth_mat.cols << endl;
            lsd.get_depth_im(depth_mat, _depth_id);
            cv::imshow( "depth", lsd.get_depth()  ); 
            cv::waitKey(3);

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
                                  
            lsd.get_im(cv_ptr->image);
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

