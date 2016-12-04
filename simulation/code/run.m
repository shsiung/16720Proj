clear;clc;
addpath('/Users/daoyuan/Dropbox/cmu/16720/project/rgbd_dataset_freiburg1_xyz/depth');
addpath('/Users/daoyuan/Dropbox/cmu/16720/project/rgbd_dataset_freiburg1_xyz/rgb');

d_ref = '1305031102.160407.png';
im_ref = '1305031102.175304.png';
im = '1305031102.211214.png';

D_ref = imread(d_ref);
I_ref = imread(im_ref);
I = imread(im);
D_ref = double(D_ref);

% 	fx	fy	cx	cy	d0	d1	d2	d3	d4
% https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats#intrinsic_camera_calibration_of_the_kinect
cam_intrinsics = {517.3,516.5,318.6,255.3,0.2624,-0.9531,-0.0054,0.0026,1.1633};
depth_scale = 5000.0;

D_ref = D_ref ./ depth_scale;

[xi, T] = sim(I_ref, D_ref, I, cam_intrinsics)
