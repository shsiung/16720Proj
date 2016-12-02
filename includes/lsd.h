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

typedef Eigen::Matrix<double,  3,  3> Matrix_3X3;   
typedef Eigen::Matrix<double,  3,  1> Matrix_3X1;
typedef Eigen::Matrix<double,  4,  4> Matrix_4X4;

class MyLSD
{

private:
    bool take_keyframe_;

    cv::Mat im_i_mat;
    cv::Mat im_j_mat;
    cv::Mat key_frame_mat;
    cv::Mat key_depth_mat;
    unsigned int im_width;
    unsigned int im_height;
    
public:
    
    struct frame_struct{
        unsigned int id;
        cv::Mat frame;
        cv::Mat depth;
        cv::Mat gradient;
        cv::Mat mask;
    } key_frame, current_frame; 

    const Matrix_3X3 I3 = Eigen::MatrixXd::Identity(3,3);
    Matrix_3X3 R;
    Matrix_3X1 T;
  
    MyLSD(): im_width(640), im_height(512), take_keyframe_(true)
    {}

    ~MyLSD(){}
    void add_frame(cv::Mat& im, unsigned int id);
    void add_depth(cv::Mat& depth, unsigned int id);
    
    string type2str(int type);

    cv::Mat get_gradient(cv::Mat& im);
    cv::Mat get_region(cv::Mat& im,double thresh, double scale);
    
    cv::Mat compute_jacob();
    cv::Mat warp_im(cv::Mat im_ref, Eigen::Vector4d SE3);
    cv::Mat update_xi();
    cv::Mat gn_update();
    Eigen::Quaterniond SO3_exp(const Eigen::Vector3d &v);
    Eigen::Vector3d SO3_log(const Eigen::Quaterniond &v);
    Eigen::Vector3d delta_R(const Matrix_3X3 &R);

    cv::Mat get_depth() {return key_depth_mat;}
    cv::Mat get_imi() {return im_i_mat;}
    cv::Mat get_imj() {return im_j_mat;}
    void set_imsize(unsigned int w, unsigned int h)
    {
        im_width = w;
        im_height = h;
    }
};
#endif
