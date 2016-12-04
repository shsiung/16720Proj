/**
 *  Image alignment using LSD's direct photometric error
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
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
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>

using namespace std;

class Camera
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr_depth;
    MyLSD lsd;
    ros::Subscriber depth_sub;          
    ros::Subscriber cam_info_sub_;          
    ros::Publisher cloud_pub_;
    ros::Publisher state_pub;

    tf::TransformBroadcaster state_broadcaster;

    cv::Mat mono8_depth;
    cv::Mat depth;
    cv::Mat depth_mat;
    image_geometry::PinholeCameraModel model_;

    double baseline = 17.5;
    unsigned int height = 640;
    unsigned int width = 480;
    double cx_, cy_, fx_, fy_;
    bool got_cam_info = false;

    public:
        Camera(): it_(nh_)
        {


            image_sub_ = it_.subscribe("/narrow_stereo/left/image_rect_color", 1, &Camera::imageCb, this);
            depth_sub = nh_.subscribe("/narrow_stereo/points2",1, &Camera::getDepthCb, this);
            cam_info_sub_ = nh_.subscribe("/narrow_stereo/left/camera_info", 1,&Camera::getCamInfo,this);
            
            cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/output/cloud", 1);
            image_pub_ = it_.advertise("/image_converter/output_video", 1);
            state_pub = nh_.advertise<nav_msgs::Odometry>("state", 50);

            // TODO: FIXME
            cx_ = 317.20617294311523;
            cy_ = 233.2914752960205;
            fx_ = fy_ = 307.4838344732113;
        }

        ~Camera()
        {
        }

        void publishState(ros::Publisher& state_pub, tf::TransformBroadcaster& state_broadcaster)
        {

          // Set a tf quaternion first, and then convert to ROS geometry quaternion message
          ros::Time current_time = ros::Time::now(); 
          tf::Quaternion quat;
          geometry_msgs::Quaternion state_quat;
          quat.setRPY(lsd.current_rpy(0), lsd.current_rpy(1), lsd.current_rpy(2));
          tf::quaternionTFToMsg(quat, state_quat);  
            
          //first, we'll publish the transform over tf tree
          geometry_msgs::TransformStamped state_trans;
          state_trans.header.stamp = current_time;
          state_trans.header.frame_id = "world";
          state_trans.child_frame_id = "odom";

          state_trans.transform.translation.x = lsd.current_pose(0);
          state_trans.transform.translation.y = lsd.current_pose(1);
          state_trans.transform.translation.z = lsd.current_pose(2);
          state_trans.transform.rotation = state_quat;

          //send the transform
          state_broadcaster.sendTransform(state_trans);

          //next, we'll publish the state tree (using an odometry message) over ROS
          nav_msgs::Odometry state;
          state.header.stamp = current_time;
          state.header.frame_id = "world";

          //set the position
          state.pose.pose.position.x = lsd.current_pose(0);
          state.pose.pose.position.y = lsd.current_pose(1);
          state.pose.pose.position.z = lsd.current_pose(2);
          state.pose.pose.orientation = state_quat;

          //set the velocity
          state.child_frame_id = "odom";

          //publish the message
          state_pub.publish(state); 
        }


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
            //From ROS to PCL cloud
            sensor_msgs::PointCloud2 cloud_msg = *cloud_msg_in;
            cloud_msg.fields[3].name = "intensity";
            PointCloud orig_cloud;
            pcl::fromROSMsg(cloud_msg, orig_cloud);

            unsigned int _width=cloud_msg.width;
            unsigned int _height=cloud_msg.height;

            depth_mat = cv::Mat::zeros(_height,_width, CV_16UC1);

            uint16_t* pd = (uint16_t*) depth_mat.data;        
            int ind = 0;
            string s1("z");
            // this call also resizes the data structure according to the given width, height and fields
            sensor_msgs::PointCloud2Iterator<float>   iter_z(cloud_msg, s1);
            for(; iter_z != iter_z.end(); ++iter_z)
            {
                // get z
                float pz = *iter_z;
                if ((uint16_t)(1000*pz) < 30000)
                    *pd = (uint16_t)(3000*pz);
                else
                    *pd = 0;
                pd++;
                ind++;
            }
            lsd.add_depth(depth_mat, orig_cloud, cloud_msg.header.seq);
        }

        void imageCb(const sensor_msgs::ImageConstPtr& msg)
        {
            sensor_msgs::Image im_msg = *msg;
            sensor_msgs::PointCloud2 cloud_PC2;
            PointCloud cloud; 
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                                        
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
                                  
            lsd.add_frame(cv_ptr->image, im_msg.header.seq);
            if (im_msg.header.seq > 2)
            {   
                cv::imshow("Key Frame", lsd.key_frame.frame);
                cv::imshow("Current Frame", cv_ptr->image);
                //cv::imshow("depth", lsd.key_frame.depth); 
                //cv::imshow("Current Frame Tracking Pixels", lsd.current_frame.grad_mask); 
                //cv::imshow("Key Frame Tracking Pixels", lsd.key_frame.grad_mask);
                //cv::imshow("Key Frame Tracking Pixels", lsd.key_frame.interest_depth_region);
                cloud = lsd.key_frame.cloud;
                cloud.header.frame_id = im_msg.header.frame_id;
                //ROS_WARN("%d", cloud.width);
               /* if (im_msg.header.seq > 200 && im_msg.header.seq % 10 == 0)
                {
                    string frame_title = "../frame_" + boost::lexical_cast<std::string>(im_msg.header.seq)+".jpg";
                    string track_title = "../tracking_" + boost::lexical_cast<std::string>(im_msg.header.seq)+".jpg";
                    string grad_tittle = "../grad_" + boost::lexical_cast<std::string>(im_msg.header.seq)+".jpg";
                    cv::imwrite( frame_title, lsd.key_frame.frame);
                    cv::imwrite( grad_tittle, lsd.key_frame.grad_mask);
                    cout <<frame_title;
                    cv::imwrite( track_title, lsd.key_frame.interest_depth_region);
                }*/
//                ROS_WARN("%d", lsd.current_frame.orig_cloud);
                pcl::toROSMsg(cloud,cloud_PC2);
                cloud_pub_.publish(cloud_PC2);
            }
            // Update GUI Window
            cv::waitKey(3);

            // Output modified video stream
            image_pub_.publish(cv_ptr->toImageMsg());
            publishState(state_pub, state_broadcaster); //Publish current state using an odometry msg

        }

};

