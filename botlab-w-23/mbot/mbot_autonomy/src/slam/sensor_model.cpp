#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   ray_stride_(1)
{
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    double likelihood = 0.0;
    double rayScore = 0.0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);
    //printf("x = %3.7f, y = %3.7f\t",movingScan[1].origin.x,movingScan[1].origin.y);

    //std::cout << global_position_to_grid_position(Point<double>(sample.pose.x,sample.pose.y), map) << "\t";
    for(int i = 0; i < movingScan.size(); i++){
        rayScore = scoreRay(movingScan[i], map);
        //printf("%7.3f ",rayScore);
        likelihood += rayScore; 
    }

    //printf("%7.3f\n",likelihood);
    return likelihood;
}


double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map){
    
    //Fraction of nearest occupied neighbour
    float nFrac =   0.333333; 

    Point<int> end_cell = global_position_to_grid_position(
        Point<double>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
    );

    /* if(int(map.logOdds(end_cell.x,end_cell.y)) > 0){
        return 1;
    }
    else {
        return 0;
    } */
    //std::cout << global_position_to_grid_position(ray.origin,map)<<"\n";
    //std::cout << end_cell.x <<"  " <<end_cell.y<<"\n";
    //std::cout <<"LogOdds_Cell: "<< int(map.logOdds(end_cell.x,end_cell.y))<< "\n";

    if(int(map.logOdds(end_cell.x,end_cell.y)) > 0){
        return int(map.logOdds(end_cell.x,end_cell.y));
    }
   
    float metersPerCell = map.metersPerCell();
    Point<int> test_cell;
    //Test cell in front of end cell
    test_cell = global_position_to_grid_position(Point<float>(
        ray.origin.x + (ray.range+metersPerCell) * std::cos(ray.theta),
        ray.origin.y + (ray.range+metersPerCell) * std::sin(ray.theta)
    ), map);

    if(int(map.logOdds(test_cell.x,test_cell.y)) > 0){
        return nFrac*int(map.logOdds(test_cell.x,test_cell.y));
    }

    //Test cell in behind of end cell
    test_cell = global_position_to_grid_position(Point<float>(
        ray.origin.x + (ray.range-metersPerCell) * std::cos(ray.theta),
        ray.origin.y + (ray.range-metersPerCell) * std::sin(ray.theta)
    ), map);

    if(int(map.logOdds(test_cell.x,test_cell.y)) > 0){
        return nFrac*int(map.logOdds(test_cell.x,test_cell.y));
    }

    return 0.0;
};