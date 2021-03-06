/**
 *  Image alignment using LSD's direct photometric error
 */
#ifndef LSD
#define LSD

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

using namespace std;

typedef Eigen::Matrix<double,  3,  3> Matrix_3X3;   
typedef Eigen::Matrix<double,  3,  1> Matrix_3X1;
typedef Eigen::Matrix<double,  4,  4> Matrix_4X4;
typedef Eigen::Matrix<double,  6,  1> Vector6d;


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
        Matrix_3X3 R;
        Matrix_3X1 T;
    } key_frame, current_frame; 


    list<Vector6d> poses;
    Eigen::Vector3d current_pose;
    Eigen::Vector3d current_rpy;

    const Matrix_3X3 I3 = Eigen::MatrixXd::Identity(3,3);
    Matrix_3X3 R;
    Matrix_3X1 T;
  
    MyLSD(): im_width(640), im_height(512), take_keyframe_(true)
    {
        current_pose = Eigen::Vector3d(0,0,0);
        current_rpy = Eigen::Vector3d(0,0,0);
    }

    ~MyLSD(){}
    void add_frame(cv::Mat& im, unsigned int id);
    void add_depth(cv::Mat& depth, PointCloud& orig_cloud, unsigned int id);
    
    string type2str(int type);

    cv::Mat get_gradient(cv::Mat& im);
    cv::Mat get_mask(cv::Mat& im,double thresh, double scale, bool inv);
    cv::Mat get_region(frame_struct& frame);
    cv::Mat get_depth_region(frame_struct& frame);
    PointCloud get_key_cloud(cv::Mat frame);
    
    cv::Mat compute_jacob();
    double compute_loss(cv::Mat& ref_im, cv::Mat& depth_im, 
                        cv::Mat& new_im, Vector6d& xi, Matrix_4X4& SE3);
    double compute_pt_residual(cv::Point& p, Matrix_4X4& xi);
    cv::Point warp_im(cv::Point& p, Matrix_4X4& SE3);
    Matrix_4X4 update_xi(Matrix_4X4& delta_xi);
    cv::Mat gn_update();
    void compute_new_pose();

    Eigen::Quaterniond SO3_exp(const Eigen::Vector3d &v);
    Eigen::Vector3d SO3_log(const Eigen::Quaterniond &q);
    Vector6d SE3_log(const Matrix_4X4 &q);
    Matrix_4X4 SE3_exp(const Vector6d &v);
    Matrix_3X3 skew(const Matrix_3X1 &v);

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
