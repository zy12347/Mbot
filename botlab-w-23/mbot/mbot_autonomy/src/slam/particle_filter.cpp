#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>


//Added libraries
#include <random>
#include <cmath>
#include <common_utils/geometric/angle_functions.hpp>
#include <numeric>
#include <chrono>

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////    
    
    //Init weights to have equal value
    double initWeight = 1./(float)kNumParticles_;
    std::random_device rd;
    std::mt19937 gen(rd());

    //Setup normal distribution
    float stddev = 0.01;
    float stddev_theta = 0.01;
    std::normal_distribution<float> d(0.0, 1.0); //Sample from normal distribution with mean 0 and stddev 1

    mbot_lcm_msgs::particle_t particle = {0};
    particle.weight = initWeight;
    particle.parent_pose = pose;

    //PARENT POSE AND POSE TIME NOT INITIALIZED
    for (int i = 0; i < kNumParticles_; i++){
        particle.pose = pose;
        //Sample nearby poses
        particle.pose.x += stddev*d(gen);
        particle.pose.y += stddev*d(gen);
        particle.pose.theta +=stddev_theta*d(gen);
        posterior_.push_back(particle);
    }
}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    
    //Init weights to have equal value
    double initWeight = 1./(float)kNumParticles_;
    std::random_device rd;
    std::mt19937 gen(rd());

    //Setup normal distribution
    int x_a = map.originInGlobalFrame().x - map.widthInMeters();
    int x_b = map.originInGlobalFrame().x + map.widthInMeters();
    int y_a = map.originInGlobalFrame().y - map.heightInMeters();
    int y_b = map.originInGlobalFrame().y + map.heightInMeters();
    double th_a = 0;
    double th_b = M_PI;

    std::uniform_real_distribution<> x(x_a, x_b); //Sample from normal distribution with mean 0 and stddev 1
    std::uniform_real_distribution<> y(y_a, y_b);    
    std::uniform_real_distribution<> theta(th_a, th_b);
    mbot_lcm_msgs::particle_t particle = {0};
    particle.weight = initWeight;
    
    mbot_lcm_msgs::pose_xyt_t particlePose = {0};


    //PARENT POSE AND POSE TIME NOT INITIALIZED
    for (int i = 0; i < kNumParticles_; i++){
        particle.pose = particlePose;
        //Sample poses uniformly from map
        particle.pose.x = x(gen);
        particle.pose.y = y(gen);
        particle.pose.theta = theta(gen);
        posterior_.push_back(particle);
    }

}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose_xyt_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved){
        std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
        auto prior = resamplePosteriorDistribution(&map);
        
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        // OPTIONAL TODO: Add reinvigoration step
        posteriorPose_ = estimatePosteriorPose(posterior_);
        //std::cout << posteriorPose_.theta << "\n"; 
        posteriorPose_.utime = odometry.utime;
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> dt = t1 - t0;
        std::cout << "Runtime: " << dt.count() << " ms" << std::endl;

    }
   
    //printf("Pose: x = %7.3f, y = %7.3f, theta =  %7.3f \n",posteriorPose_.x,posteriorPose_.y,posteriorPose_.theta);
    return posteriorPose_;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = proposal;
    }

    posteriorPose_ = odometry;

    return posteriorPose_;
}



mbot_lcm_msgs::pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid* map)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    ParticleList prior;


    //Don't resample if we have enough effective particles
    // double neff = 0;
    // for (int k = 0; k < kNumParticles_; k++){
    //     neff += posterior_.at(k).weight*posterior_.at(k).weight ;
    // }
    // neff = 1/neff;
    // if (neff >= (float)kNumParticles_/3){
    //     return posterior_;
    // }

    //Low variance resampling

    //Initialize a random starting position
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> starting_pt(0, 1./kNumParticles_);

    int i = 0;
    double r = starting_pt(gen);
    double c = posterior_.at(0).weight;
    for (int m = 0; m < kNumParticles_; m++){
        double U = r + (double)m/(double)kNumParticles_;
        while (U > c){
            i++;
            //std::cout<< i << std::endl;
            c += posterior_.at(i).weight; 
        }
        //std::cout <<"U: " <<U <<"c: "<< c <<"\n"<< std::endl;

        prior.push_back(posterior_.at(i));
    }
    
   //std::cout << global_position_to_grid_position(Point<double>(posterior_[1].pose.x,posterior_[1].pose.y), *map) << "\t";

    return prior;
}

ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList proposal;

    //Pass particles through action model
    for(int i = 0 ; i < prior.size(); i++){
        //std::cout << prior.at(i).pose.x << prior.at(i).pose.y << prior.at(i).pose.theta << std::endl;
        proposal.push_back(actionModel_.applyAction(prior.at(i)));
        //std::cout << proposal.at(i).pose.x << proposal.at(i).pose.y << proposal.at(i).pose.theta << std::endl << std::endl;

    }

    return proposal;
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution

    ParticleList posterior;
    double normalizer = 0;
    //Determine p(z|x) to reweight each particle
    for(int i = 0 ; i < proposal.size(); i++){
        mbot_lcm_msgs::particle_t particle = proposal.at(i);
        
        particle.weight = sensorModel_.likelihood(particle, laser, map); //return likelihood
        
        //std::cout <<(particle.weight) <<std::endl;
        
        normalizer += particle.weight;
        posterior.push_back(particle);
    }
    //printf("Pose: x = %7.3f, y = %7.3f, theta =  %7.3f \n",posterior.at(1).pose.x,top_particles.at(1).pose.y,top_particles.at(1).pose.theta);
    //std::cout <<int(map.logOdds(100,100)) <<"\n"; 
    //Normalize weights
    for(int i = 0 ; i < proposal.size(); i++){
        posterior.at(i).weight /= normalizer;
    }
    return posterior;
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    mbot_lcm_msgs::pose_xyt_t pose;
    ParticleList ordered_particles = posterior;
    //printf("Pose: x = %7.3f, y = %7.3f, theta =  %7.3f \n",post.at(1).pose.x,top_particles.at(1).pose.y,top_particles.at(1).pose.theta);
    std::sort(ordered_particles.begin(), ordered_particles.end(), SortbyWt); //Sorts particles from largest to smallest
    //std::cout << ordered_particles.at(0).weight << " " << ordered_particles.back().weight <<std::endl;
    
    float numParticles = 0.1*(float)posterior.size();

    ParticleList top_particles;
    for (int i = 0; i<= (int)numParticles; i++){
        top_particles.push_back(ordered_particles.at(i));
        //printf("Pose: x = %7.3f, y = %7.3f, theta =  %7.3f \n",top_particles.at(i).pose.x,top_particles[i].pose.y,top_particles[i].pose.theta);
    } 
    

    pose = computeParticlesAverage(top_particles);
    return pose;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    //////// TODO: Implement your method for computing the average of a pose distribution
    mbot_lcm_msgs::pose_xyt_t avg_pose;
    
    avg_pose.x = 0;
    avg_pose.y = 0;
    avg_pose.utime = particles_to_average.at(0).pose.utime;

    double norm_weight = 0;
    double this_weight;
    double sin_sum = 0;
    double cos_sum = 0;
    for(int i = 0 ; i < particles_to_average.size(); i++){
        this_weight = particles_to_average.at(i).weight;
        avg_pose.x += this_weight*particles_to_average.at(i).pose.x;
        avg_pose.y += this_weight*particles_to_average.at(i).pose.y;
        sin_sum += this_weight*sin(particles_to_average.at(i).pose.theta);
        cos_sum += this_weight*cos(particles_to_average.at(i).pose.theta);
        norm_weight += this_weight;
    }
    avg_pose.x /= norm_weight;
    avg_pose.y /= norm_weight;
    avg_pose.theta = wrap_to_pi(atan2(sin_sum, cos_sum));

    return avg_pose;
}

bool SortbyWt(mbot_lcm_msgs::particle_t p1, mbot_lcm_msgs::particle_t p2){
    return p1.weight>p2.weight;
}