/**
 *  Image alignment using LSD's direct photometric error
 */

#ifndef LSD
#define LSD

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Eigen>

using namespace std;

class MyLSD
{

    typedef Eigen::Matrix<double,  3,  3> Matrix_3X3;   

    private:
        cv::Mat* im_i;
        cv::Mat* im_j;
        cv::Mat* key_frame;
        cv::Mat* key_depth;
        unsigned int im_width;
        unsigned int im_height;
        
        double cx_, cy_, fx_, fy_;

    public:
    
    MyLSD():
        im_width(640), im_height(512)
    {
        im_i = new cv::Mat(im_height, im_width, CV_64F, 0.0);
        im_j = new cv::Mat(im_height, im_width, CV_64F, 0.0);
        key_frame = new cv::Mat(im_height, im_width, CV_64F, 0.0);
        key_depth = new cv::Mat(im_height, im_width, CV_64F, 0.0);
    
        cx_ = 317.20617294311523;
        cy_ = 233.2914752960205;
        fx_ = fy_ = 307.4838344732113;
    }

    ~MyLSD(){}
    void get_im(cv_bridge::CvImagePtr& cv_ptr);
    void get_imdepth(cv_bridge::CvImagePtr& cv_ptr_depth);

    cv::Mat compute_jacob();
    cv::Mat update_xi();
    cv::Mat gn_update();
    Eigen::Quaterniond SO3_exp(const Eigen::Vector3d &v);
    Eigen::Vector3d SO3_log(const Eigen::Quaterniond &v);
    Eigen::Vector3d delta_R(const Matrix_3X3 &R);

    cv::Mat get_gradient();
    cv::Mat get_imi() {return *im_i;}
    cv::Mat get_imj() {return *im_j;}
    void set_imsize(unsigned int w, unsigned int h)
    {
        im_width = w;
        im_height = h;
    }
};
#endif
