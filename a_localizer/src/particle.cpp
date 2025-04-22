#include "localizer/particle.hpp"

// デフォルトコンストラクタ
Particle::Particle() : pose_(0.0, 0.0, 0.0)
{
    weight_ = 0.0;
}

// コンストラクタ
Particle::Particle(const double x, const double y, const double yaw, const double weight) : pose_(x, y, yaw)
{
    weight_ = weight;
}

// 代入演算子
Particle& Particle::operator =(const Particle& p)
{
    pose_ = p.pose_;
    weight_ = p.weight_;
    return *this;
}

// setter
void Particle::set_weight(const double weight)
{
    weight_ = weight;
}

// 尤度関数
// センサ情報からパーティクルの姿勢を尤度を算出
double Particle::likelihood(const nav_msgs::msg::OccupancyGrid& map, const sensor_msgs::msg::LaserScan& laser,
        const double sensor_noise_ratio, const int laser_step, const std::vector<double>& ignore_angle_range_list)
{
    double L = 0.0; // 尤度
    // センサ情報からパーティクルの姿勢を評価
    for(int i=0; i<laser.ranges.size(); i+=laser_step)
    {
        double angle = i*laser.angle_increment + laser.angle_min;

        if(not is_ignore_angle(angle, ignore_angle_range_list))
        {
            double dist = calc_dist_to_wall(pose_.x(), pose_.y(), pose_.yaw()+angle, 
                map, laser.ranges[i], sensor_noise_ratio);
            // printf("%f %f\n", dist, laser.ranges[i]);
            L += norm_pdf(dist, laser.ranges[i], sensor_noise_ratio);
            // printf("no ignore\n");
        }else{
            // printf("ignore\n");
        }
    }

    // printf("L %f\n", L);
    return L;
}

// 柱がある範囲か判定
bool Particle::is_ignore_angle(double angle, const std::vector<double>& ignore_angle_range_list)
{
    const int size = ignore_angle_range_list.size();

    for(int i=0; i<size; i+=2){
        if(ignore_angle_range_list[i]<angle && ignore_angle_range_list[i+1]){
            return true;
        }
    }
    return false;
}

// 与えられた座標と角度の方向にある壁までの距離を算出
// マップデータが100の場合，距離を返す
// マップデータが-1（未知）の場合，マップ範囲外の場合はsearch_limit * 2.0を返す
// いずれでもない場合は，search_limit * 5.0を返す
double Particle::calc_dist_to_wall(double x, double y, const double laser_angle, const nav_msgs::msg::OccupancyGrid& map,
        const double laser_range, const double sensor_noise_ratio)
{
    // 探索のステップサイズ
    const double search_step = map.info.resolution;
    // 最大探索距離
    const double search_limit = laser_range;

    // 探索
    for(double dist=0.0; dist<search_limit; dist+=search_step)
    {
        x += search_step * cos(laser_angle);
        y += search_step * sin(laser_angle);
        const int grid_index = xy_to_grid_index(x, y, map.info);
        // printf("%d\n", map.data[grid_index]);

        if(!in_map(grid_index, map.data.size()) || map.data[grid_index] == -1) return search_limit*2.0;
        else if(map.data[grid_index] == 100){
            // printf("dist %f\n", dist);
            return dist;
        }
    }
    // printf("error\n");
    return search_limit*5.0;
}

// 座標からグリッドのインデックスを返す
int Particle::xy_to_grid_index(const double x, const double y, const nav_msgs::msg::MapMetaData& map_info)
{
    int index_x = int(round((x - map_info.origin.position.x)/map_info.resolution));
    int index_y = int(round((y - map_info.origin.position.y)/map_info.resolution));
    int grid_index = index_x + index_y*map_info.width;
    return grid_index;
}

// マップ内か判定
bool Particle::in_map(const int grid_index, const int map_data_size)
{
    if(grid_index < map_data_size){
        return true;
    }else return false;
}

// 確率密度関数（正規分布）
double Particle::norm_pdf(const double x, const double mean, const double stddev)
{
    return exp(-0.5*pow((x - mean)/stddev, 2))/(stddev*sqrt(2.0*M_PI));
}
