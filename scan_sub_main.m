clear all;
close all;
global controlpub
global count
global target
global u
global K_est
global rho_est
global rho
global K
global rho_est_vector
global rho_vector
global u_vector;
global K_vector;
global K_est_vector;
tic;

u = double(u);
rho = double(rho);
K = double(K);

rho_est = double(rho_est);
K_est = double(K_est);

rho_vector = [];
rho_est_vector = [];
rho_est = 0;
K_est = 0;
count = 1;
target = [NaN;NaN];
u = 0;


controlpub = rospublisher('/head/control/scan','std_msgs/Float32');
scansub = rossubscriber('/head/kinect2/scan2DArray', @scanCallback_circle);