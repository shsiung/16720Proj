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
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>


using namespace std;

typedef Eigen::Matrix<double,  3,  3> Matrix_3X3;   
typedef Eigen::Matrix<double,  3,  1> Matrix_3X1;
typedef Eigen::Matrix<double,  4,  4> Matrix_4X4;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

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
        cv::Mat grad_mask;
        cv::Mat interest_region;
        cv::Mat interest_depth_region;
        Matrix_4X4 xi;
        PointCloud cloud;
        PointCloud::Ptr orig_cloud;
    } key_frame, current_frame; 

    const Matrix_3X3 I3 = Eigen::MatrixXd::Identity(3,3);
    Matrix_3X3 R;
    Matrix_3X1 T;
  
    MyLSD(): im_width(640), im_height(512), take_keyframe_(true) {}

    ~MyLSD(){}
    void add_frame(cv::Mat& im, unsigned int id);
    void add_depth(cv::Mat& depth, PointCloud& orig_cloud, unsigned int id);
    
    string type2str(int type);

    cv::Mat get_gradient(cv::Mat& im);
    cv::Mat get_mask(cv::Mat& im,double thresh, double scale, bool inv);
    cv::Mat get_region(frame_struct& frame);
    cv::Mat get_depth_region(frame_struct& frame);
    
    cv::Mat compute_jacob();
    double compute_pt_residual(cv::Point& p, Matrix_4X4& xi);
    cv::Point warp_im(cv::Point& p, Matrix_4X4& SE3);
    Matrix_4X4 update_xi(Matrix_4X4& delta_xi);
    cv::Mat gn_update();

    PointCloud get_key_cloud(cv::Mat frame);

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
