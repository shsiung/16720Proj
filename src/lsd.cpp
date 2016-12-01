
#include "lsd.h"

void MyLSD::get_im(cv_bridge::CvImagePtr& cv_ptr)
{
    im_i = im_j;
    *im_j = (cv_ptr->image.clone());
    //cout << "I_size = " << endl << " " << im_temp.rows << "," << im_temp.cols <<endl;
    //    cout << "I = "<< endl << " "  << im_i << endl << endl;
    //    cout << "J_size = " << endl << " " << im_j->rows << "," << im_j->cols <<endl;
    //    cout << "J = "<< endl << " "  << im_j << endl << endl;
}
void MyLSD::get_imdepth(cv_bridge::CvImagePtr& cv_ptr_depth)
{
    *key_depth = (cv_ptr_depth->image.clone());
     //cout << "depth_size = " << endl << " " << key_depth->rows << "," << key_depth->cols <<endl;
    //cout << "depth"<< endl << " "  << *key_depth << endl << endl;
}

cv::Mat MyLSD::get_gradient()
{
    cv::Mat grad_x;
    cv::Sobel(*im_j, grad_x, CV_64F, 1,0,3);
    return grad_x;
}

cv::Mat MyLSD::compute_jacob()
{
    cv::Mat grad_x;
    cv::Sobel(*im_j, grad_x, CV_64F, 1,0,3);
    return grad_x;
}

cv::Mat MyLSD::update_xi()
{
    cv::Mat grad_x;
    cv::Sobel(*im_j, grad_x, CV_64F, 1,0,3);
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
    cv::Sobel(*im_j, grad_x, CV_64F, 1,0,3);
    return grad_x;
}
