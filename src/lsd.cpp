
#include "lsd.h"

void MyLSD::add_frame(cv::Mat& im, unsigned int id)
{
    im_i_mat = im_j_mat;
    im_j_mat = (im.clone());
    //ROS_WARN("Current frame id: %d", id);
    //TODO: THIS IS JUST TEMPORARY
    if (id-key_frame.id >= 10)
    {
        take_keyframe_=true;
    }

    if (take_keyframe_)
    {
        ROS_WARN("Keyframe Im Added! Seq: %d", id);
        key_frame.frame = im_j_mat.clone();
        key_frame.gradient = get_gradient(im_j_mat).clone();
        key_frame.grad_mask = get_mask(key_frame.gradient, 125, 1, false);
        key_frame.interest_region = get_region(key_frame);
        key_frame.id = id;

        take_keyframe_= false;

        cout << type2str(key_frame.frame.type());
    }
    else
    {
        current_frame.frame = im_j_mat.clone();
        current_frame.gradient = get_gradient(im_j_mat).clone();
        current_frame.grad_mask = get_mask(current_frame.gradient, 125, 1, false);
        current_frame.id = id;
    }
}

void MyLSD::add_depth(cv::Mat& depth, unsigned int id)
{
    key_depth_mat = depth.clone();
    if (id == key_frame.id){
        key_frame.depth = key_depth_mat;
    }
    key_frame.interest_depth_region = get_depth_region(key_frame);
}


PointCloud MyLSD::get_key_cloud(cv::Mat& frame)
{
    
    PointCloud cloud;
    cloud.height = 1;
    cloud.width =1;
    cloud.is_dense = false;
    
    double pt_counter = 0;
    pcl::PointXYZI pt;
    for ( int x = 0; x < frame.rows; x++  )
    {
        for ( int y = 0; y < frame.cols ; y++  )
        {
            double val = frame.at<double>(y,x);
            if (val > 0)
            {
                pt_counter ++;
                pt.x = y;
                pt.y = x;
                pt.z = frame.at<double>(y,x);
                pt.intensity = 220;
                cloud.points.push_back(pt);
            }
        }
    }
    cloud.width = pt_counter;

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
    return current_frame.frame.at<double>(p.x,p.y) 
         - key_frame.frame.at<double>(warped_p.x,warped_p.y);
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


