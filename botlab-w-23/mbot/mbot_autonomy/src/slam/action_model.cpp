#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.0005f)//for angle
, k2_(0.001f)//for distance
, min_dist_(0.00025)
, min_theta_(0.002)
, initialized_(false)
{   
    std::random_device rd;
    numberGenerator_ =  std::mt19937(rd());
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if(!initialized_){
        initialized_ = true;
        previousPose_ = odometry;
    }
    bool moved = 0;

    dx_ = odometry.x - previousPose_.x;
    dy_ = odometry.y - previousPose_.y;
    dtheta_ = odometry.theta - previousPose_.theta;

    float direction;
    dalpha_ = angle_diff(atan2(dy_,dx_),previousPose_.theta);
    dtrans_ = sqrt(dx_*dx_ + dy_*dy_);
    if(abs(dalpha_>M_PI/2)){
        dalpha_ = angle_diff(M_PI,dalpha_);
        direction = -1;
    }
    dtheta_alpha_ = angle_diff(dtheta_, dalpha_); 

    if(dtrans_>min_dist_||dtheta_>min_theta_){
        moved = true;
        xStd_ =  k2_ * dx_;
        yStd_ = k2_ * dy_;
        thetaStd_ = k1_ * dtheta_;
        sigma1_std = sqrt(k1_*abs(dalpha_));
        sigma2_std = sqrt(k2_*abs(dtrans_));
        sigma3_std = sqrt(k1_*abs(dtheta_alpha_));
    }
    //dtrans_ *= direction;
    previousPose_ = odometry;
    utime_ = odometry.utime;
    return moved;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;

    // dx_ = sample.pose.x - sample.parent_pose.x;
    // dy_ = sample.pose.y - sample.parent_pose.y;
    // dtheta_ = angle_diff(sample.pose.theta,sample.parent_pose.theta);

    // dalpha_ = atan(dy_,dx_)-sample.parent_pose.theta;
    // dtrans_ = sqrt(dx_*dx_+dy_*dy_);
    // dtheta_alpha_ = angle_diff(dtheta, dalpha_);

    // std::normal_distribution<float> dx(0, xStd_);
    // std::normal_distribution<float> dy(0, yStd_);
    // std::normal_distribution<float> dtheta(0, thetaStd_);

    std::normal_distribution<float> drot1(0, sigma1_std);
    std::normal_distribution<float> dtr(0, sigma2_std);
    std::normal_distribution<float> drot2(0, sigma3_std);

    float error1 = drot1(numberGenerator_);
    float error2 = dtr(numberGenerator_);
    float error3 = drot2(numberGenerator_);

    newSample.pose.x = sample.pose.x + (dtrans_+error2)*cos(sample.pose.theta + dalpha_+error1);
    newSample.pose.y = sample.pose.y + (dtrans_+error2)*sin(sample.pose.theta + dalpha_+error1);
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + dtheta_+error1+error3);
    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;
    return newSample;
}
