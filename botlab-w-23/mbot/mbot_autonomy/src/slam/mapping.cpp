#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono> 
using namespace std::chrono; 

//Added to prevent overflow and underflow of data
CellOdds safe_add_odds(CellOdds a, CellOdds b) {
    if (b > INT8_MAX - a && a > 0) {
        //overflow
        return INT8_MAX;
    } else if (b < INT8_MIN - a && a < 0) {
        //underflow
        return INT8_MIN;
    }
    return a + b;
}

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose_xyt_t& pose,
                        OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    // Take lidar scan and adjust for robot pose change while scanning
    MovingLaserScan movingScan(scan,previousPose_,pose);
    adjusted_ray_t rayInScan;

    //Loop through all rays in scan and score them to update map
    for(int i = 0; i < movingScan.size(); i++){
        rayInScan = movingScan[i];
        //printf("Index: %7.3f; Range: %7.3f", i, rayInScan.range);
        //Score
        scoreRay(rayInScan, map);
        scoreEndpoint(rayInScan,map);
    }

    previousPose_ = pose;

}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your endpoint score ///////////////////////
    Point<int> end_cell = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
        );
    

    if(ray.range >= kMaxLaserDistance_) //If ray is bigger than max laser distance; discard reading
    { 
    }
    else // If ray is within range, increase end cell occupancy 
    {
        map.setLogOdds(end_cell.x,end_cell.y, safe_add_odds(map.logOdds(end_cell.x,end_cell.y),kHitOdds_));
    }
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your ray score ///////////////////////
    
    if(ray.range >= kMaxLaserDistance_) //If ray is bigger than max laser distance; discard reading
    { 
    }
    else 
    {  
        // If ray is within range, decrease occupancy of cells leading up to end cell
        std::vector<Point<int>> cells_touched = bresenham(ray,map);
        for(auto& cell: cells_touched){
            map.setLogOdds(cell.x,cell.y, safe_add_odds(map.logOdds(cell.x,cell.y),-kMissOdds_)); 
        }
    }
}
/*
Takes the ray and map, and returns a vector of map cells to check except for end cell of ray
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    // Get global positions 
    Point<float> f_end = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y +  ray.range * std::sin(ray.theta)
            ), 
        map
        );

    // Cells
    Point<int> start_cell = global_position_to_grid_cell(ray.origin, map);
    Point<int> end_cell;
    end_cell.x = static_cast<int>(f_end.x);
    end_cell.y = static_cast<int>(f_end.y);
    std::vector<Point<int>> cells_touched;
    //////////////// TODO: Implement Bresenham's Algorithm ////////////////
    Point<int> this_cell;
    
    int dx = abs(end_cell.x - start_cell.x);
    int dy = abs(end_cell.y - start_cell.y);

    int sx  = (start_cell.x < end_cell.x) ?  1: -1;
    int sy = (start_cell.y < end_cell.y) ? 1: -1;

    int err = dx - dy;
    
    int x = start_cell.x;
    int y = start_cell.y;

    while (x != end_cell.x || y!= end_cell.y){
        this_cell.x = x;
        this_cell.y = y;
        cells_touched.push_back(this_cell);      
        
        int e2 = 2*err;
        if (e2 >= -dy){
            err -= dy;
            x += sx;
        }
        if (e2 <= dx){
            err +=dx;
            y += sy;
        }
    }
    return cells_touched;
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    auto end_cell = global_position_to_grid_cell(Point<double>(
        ray.origin.x + ray.range * std::cos(ray.theta),
        ray.origin.y + ray.range * std::sin(ray.theta)
        ), map);
    //////////////// TODO: Implement divide and step ////////////////
    
    std::vector<Point<int>> cells_touched;
    return cells_touched;
}

