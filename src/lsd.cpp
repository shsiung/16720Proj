
#include "lsd.h"

void MyLSD::add_frame(cv::Mat& im, unsigned int id)
{
    im_i_mat = im_j_mat;
    im_j_mat = (im.clone());
    //ROS_WARN("Current frame id: %d", id);
    
    //TODO: THIS IS JUST TEMPORARY
    if (id-key_frame.id >= 20)
    {
        take_keyframe_=true;
    }

    if (take_keyframe_)
    {
        ROS_WARN("Keyframe Im Added! Seq: %d", id);
        key_frame.frame = im_j_mat.clone();
        key_frame.gradient = get_gradient(im_j_mat).clone();
        key_frame.mask = get_region(key_frame.gradient, 125, 1);
        key_frame.id = id;

        take_keyframe_= false;
    }
    else
    {
        current_frame.frame = im_j_mat.clone();
        current_frame.gradient = get_gradient(im_j_mat).clone();
        current_frame.mask = get_region(new_frame.gradient, 125, 1);
        current_frame.id = id;
    }

    //cout << "I_size = " << endl << " " << im_temp.rows << "," << im_temp.cols <<endl;
    //    cout << "I = "<< endl << " "  << im_i_mat << endl << endl;
    //    cout << "J_size = " << endl << " " << im_j_mat->rows << "," << im_j->cols <<endl;
    //    cout << "J = "<< endl << " "  << im_j_mat << endl << endl;
}
void MyLSD::add_depth(cv::Mat& depth, unsigned int id)
{
    key_depth_mat = depth.clone();
    if (id == key_frame.id){
        ROS_WARN("Keyframe Depth Added! Seq: %d", key_frame.id);
        key_frame.depth = key_depth_mat;
    }
    // cout << "depth_size = " << endl << " " << key_depth_mat->rows << "," << key_depth->cols <<endl;
    //cout << "depth"<< endl << " "  << *key_depth_mat << endl << endl;
}

cv::Mat MyLSD::get_gradient(cv::Mat& im)
{
    cv::Mat grad_x;
    cv::Sobel(im, grad_x, CV_8UC1, 1,0,3);
    return grad_x;
}

cv::Mat MyLSD::get_region(cv::Mat& im,double thresh, double scale)
{
    cv::Mat mask_8;
    cv::Mat out;
    im.convertTo(mask_8, CV_8UC1, scale);
    cv::threshold(mask_8, out, thresh, 255, CV_THRESH_BINARY);
    return out;
}

cv::Mat MyLSD::compute_jacob()
{
    cv::Mat grad_x;
    return grad_x;
}

cv::Mat MyLSD::update_xi()
{
    cv::Mat grad_x;
    return grad_x;
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


