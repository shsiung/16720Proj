
#include "lsd.h"

void MyLSD::add_frame(cv::Mat& im, unsigned int id)
{
    im_i_mat = im_j_mat;
    im_j_mat = (im.clone());

    if (id-key_frame.id >= 10)
    {
        take_keyframe_=true;
    }

    if (take_keyframe_)
    {
        ROS_WARN("Keyframe Im Added! Seq: %d", id);
        key_frame.frame = im_j_mat.clone();
        key_frame.gradient = get_gradient(im_j_mat).clone();
        key_frame.grad_mask = get_mask(key_frame.gradient, 60, 1, false);
        key_frame.interest_region = get_region(key_frame);
        key_frame.id = id;

        take_keyframe_= false;
    }
    else
    {
        current_frame.frame = im_j_mat.clone();
        current_frame.gradient = get_gradient(im_j_mat).clone();
        current_frame.grad_mask = get_mask(current_frame.gradient, 60, 1, false);
        current_frame.id = id;
    }
    get_new_pose();
}


//Running LSD match to get new pose (alignment)
void MyLSD::get_new_pose()
{
    current_pose = current_pose + Eigen::Vector3d(0.01,0,0);
}

void MyLSD::add_depth(cv::Mat& depth, PointCloud& orig_cloud, unsigned int id)
{

    key_depth_mat = depth.clone();
    if (id == key_frame.id){
        key_frame.depth = key_depth_mat;
        key_frame.interest_depth_region = get_depth_region(key_frame).clone();
        key_frame.orig_cloud = orig_cloud.makeShared();
        key_frame.cloud = get_key_cloud(key_frame.frame);
    }
}

PointCloud MyLSD::get_key_cloud(cv::Mat frame_in)
{
    
    PointCloud cloud = *key_frame.orig_cloud;
        
    double minVal, maxVal;
    minMaxLoc( key_frame.depth, &minVal, &maxVal  );

    double pt_counter = 0;
    pcl::PointXYZI pt;

    cv::Mat mask_8;
    cv::Mat out;
    key_frame.interest_depth_region.convertTo(mask_8, CV_8UC1, 255/65536.0);
    cv::Mat image;
    cvtColor(mask_8,image,CV_RGB2GRAY);

    int val = 0;
    for ( int x = 0; x < frame_in.rows; x++  )
    {
        for ( int y = 0; y < frame_in.cols ; y++  )
        {
            val = (int)image.at<unsigned char>(x,y);
            if (val == 0)
            {
                cloud.at(y,x).z = -50;
            }
        }
    }
    //ROS_WARN("%d", cloud.width);
    return cloud;
}

cv::Mat MyLSD::get_region(frame_struct& frame)
{
    cv::Mat masked_out;
    frame.frame.copyTo(masked_out,frame.grad_mask);
    return masked_out;
}

cv::Mat MyLSD::get_depth_region(frame_struct& frame)
{
    cv::Mat image;
    cvtColor(frame.depth,image,CV_GRAY2RGB);
    cv::Mat masked_depth_out;
    image.copyTo(masked_depth_out,frame.grad_mask);

    return masked_depth_out;
}
cv::Mat MyLSD::get_gradient(cv::Mat& im)
{
    cv::Mat grad_x;
    cv::Sobel(im, grad_x, CV_8UC1, 1,0,3);
    return grad_x;
}

cv::Mat MyLSD::get_mask(cv::Mat& im,double thresh, double scale, bool inv)
{
    cv::Mat mask_8;
    cv::Mat out;
    im.convertTo(mask_8, CV_8UC1, scale);
    if (inv)
        cv::threshold(mask_8, out, thresh, 255, CV_THRESH_BINARY_INV);
    else
        cv::threshold(mask_8, out, thresh, 255, CV_THRESH_BINARY);
    return out;
}

cv::Mat MyLSD::compute_jacob()
{
    cv::Mat grad_x;
    return grad_x;
}

Matrix_4X4 MyLSD::update_xi(Matrix_4X4& delta_xi)
{
    Matrix_4X4 new_xi;
    new_xi.block<3,3>(0,0) = current_frame.xi.block<3,3>(0,0) * delta_xi.block<3,3>(0,0);
    new_xi.block<3,1>(0,3) = current_frame.xi.block<3,3>(0,0) * delta_xi.block<3,1>(0,3) 
                           + current_frame.xi.block<3,1>(0,3);
    new_xi(3,3) = 1;
    return new_xi;
}

double MyLSD::compute_pt_residual(cv::Point& p, Matrix_4X4& xi)
{
    cv::Point warped_p = warp_im(p,xi);
    return current_frame.frame.at<double>(p.y,p.x) 
         - key_frame.frame.at<double>(warped_p.y,warped_p.x);
}

cv::Point MyLSD::warp_im(cv::Point& p, Matrix_4X4& SE3)
{
    double depth = key_frame.depth.at<double>(p.x, p.y);
    Eigen::Vector4d P  (p.x*depth, 
                        p.y*depth, 
                            depth, 
                                1);
    Eigen::Vector4d warped_3d_pt = SE3 * P;
    return cv::Point(warped_3d_pt(0)/warped_3d_pt(2),
                     warped_3d_pt(1)/warped_3d_pt(2));
}

double compute_loss(cv::Mat& ref_im, cv::Mat& depth_im, cv::Mat& new_im, Vector6d& xi)
{
    double a = 0;
    return a;
}

Eigen::Quaterniond MyLSD::SO3_exp(const Eigen::Vector3d &v)
{
    Eigen::Quaterniond q;
    double angle = v.norm();
    if (angle < 0.01*3.1415926/180){
	q.setIdentity();
	return q;
    }	
    Eigen::Vector3d axis = v/angle;
    q.vec() = axis * sin(angle/2);
    q.w() = cos(angle/2);
    q.normalize();
    return q;
}

Eigen::Vector3d MyLSD::SO3_log(const Eigen::Quaterniond &q)
{
    Matrix_3X3 R = q.toRotationMatrix();       
    double d =  0.5*(R(0,0)+R(1,1)+R(2,2)-1); 
    Eigen::Vector3d omega;
    Eigen::Vector3d dR = delta_R(R);
    if (d > 0.99999){
        omega = 0.5 * dR;
    }
    else{
        double theta = acos(d);
        omega = theta/(2*sqrt(1-d*d))*dR;
    }  
    return omega;
}

Vector6d MyLSD::SE3_log(const Matrix_4X4 &q)
{
    Matrix_3X3  R = q.block<3,3>(0,0);    
    Matrix_3X1  Vu = q.block<3,1>(0,3);
    Vector6d    xi;
    double      theta = acos((R.trace() - 1) / 2.0);
    Matrix_3X1  w = (theta / (2 * sin(theta)) * (R - R.transpose())).diagonal();
    Matrix_3X3  wx = skew(w);

    double      A = sin(theta)/theta; 
    double      B = (1 - cos(theta))/(theta*theta);
    double      C = (1-A) / (theta*theta);

    Matrix_3X3 V = Eigen::MatrixXd::Identity(3,3) + B * wx + C * wx * wx;

    Matrix_3X1 u = V.inverse() * Vu;
    xi.block<3,1>(0,0) = u;
    xi.block<3,1>(3,0) = w; 

    return xi;
}

Matrix_4X4 MyLSD::SE3_exp(const Vector6d &v)
{
    Matrix_4X4 T;
    double x = v(0);
    double y = v(1);
    double z = v(3);
    double w1 = v(4);
    double w2 = v(5);
    double w3 = v(6);
    Matrix_3X1 u = Eigen::Vector3d(x,y,z);
    Matrix_3X1 w = Eigen::Vector3d(w1,w2,w3);
    Matrix_3X3 wx = skew(w);
    Matrix_3X3 wx2 = wx*wx;
    double theta = sqrt(w1*w1 + w2*w2 + w3*w3);
    if (abs(theta) > 1e-20)
    {
        double      A = sin(theta)/theta; 
        double      B = (1 - cos(theta))/(theta*theta);
        double      C = (1-A) / (theta*theta);
        Matrix_3X3 R = Eigen::MatrixXd::Identity(3,3) + A * wx + C*wx2;
        Matrix_3X3 V = Eigen::MatrixXd::Identity(3,3) + B * wx + C*wx2;
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = V*u;
        T.block<1,3>(3,0) = Eigen::Matrix<double,  1,  3>(0,0,0);
        T(3,3) = 1;
    }
    else
    {
        T.block<3,1>(0,0) = Eigen::Vector3d::Zero();
        T.block<3,1>(0,1) = Eigen::Vector3d::Zero();
        T.block<3,1>(0,2) = Eigen::Vector3d::Zero();
        T.block<3,1>(0,3) = u;
        T.block<1,3>(3,0) = Eigen::Matrix<double,  1,  3>(0,0,0);
        T(3,3) = 1;
    }
    return T;
}

Matrix_3X3 MyLSD::skew(const Matrix_3X1 &v)
{
  Matrix_3X3 out;
  out <<     0, -v[2],  v[1],
          v[2],     0, -v[0],
         -v[1],  v[0],     0;

  return out;
}

Eigen::Vector3d MyLSD::delta_R(const Matrix_3X3 &R)
{
    Eigen::Vector3d v;
    v(0)=R(2,1)-R(1,2);
    v(1)=R(0,2)-R(2,0);
    v(2)=R(1,0)-R(0,1);
    return v;
}

cv::Mat MyLSD::gn_update()
{
    cv::Mat grad_x;
    return grad_x;
}

string MyLSD::type2str(int type) {
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth  ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }
    r += "C";
    r += (chans+'0');
    return r;
}


