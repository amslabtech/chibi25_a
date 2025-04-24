#include "local_map_creator/local_map_creator.hpp"

// コンストラクタ
LocalMapCreator::LocalMapCreator() : Node("local_map_creator")
{
    // パラメータの初期化
    this->declare_parameter()
    // パラメータの取得(hz, map_size, map_reso)
    this->get_parameter("hz", hz_);
    this->get_parameter("map_size", map_size_);
    this->get_parameter("map_reso", map_reso_);

    // Subscriberの設定
    sub_obs_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "obs_poses", 10, std::bind(&LocalMapCreator::obs_poses_callback, this, std::placeholders::_1));

    // Publisherの設定
    pub_local_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "local_map", 10);

    // --- 基本設定 ---
    // マップの基本情報(local_map_)を設定する（header, info, data）
    //   header
    local_map_.header.frame_id = "map";   // 座標フレーム名
    local_map_.header.stamp = this->now();  // タイムスタンプ

    //   info(width, height, position.x, position.y)
    // マップのメタデータ
    local_map_.info.resolution = map_reso_;
    local_map_.info.width = static_cast<uint32_t>(map_size_ / map_reso_);
    local_map_.info.height = static_cast<uint32_t>(map_size_ / map_reso_);

    // マップの原点（左下）
    local_map_.info.origin.position.x = -map_size_ / 2.0;
    local_map_.info.origin.position.y = -map_size_ / 2.0;
    local_map_.info.origin.position.z = 0.0;
    local_map_.info.origin.orientation.w = 1.0;  // 向きはデフォルト

    //   data
    local_map_.data.assign(local_map_.info.width * local_map_.info.height, -1);
}

// obs_posesのコールバック関数
void LocalMapCreator::obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    obs_poses_ = *msg;
    flag_obs_poses_ = true;
}

// 周期処理の実行間隔を取得する
int LocalMapCreator::getFreq()
{
    return hz_; // 合ってる？
}

// 障害物情報が更新された場合、マップを更新する
void LocalMapCreator::process()
{
    rclcpp::Rate rate(hz_);
    while (rclcpp::ok())
    {
        if (flag_obs_poses_)
        {
            update_map();
            flag_obs_poses_ = false;
        }
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }    
}

// 障害物の情報をもとにローカルマップを更新する
void LocalMapCreator::update_map()
{
    // マップを初期化する
    init_map();

    // 障害物の位置を考慮してマップを更新する
    for (const auto & pose : obs_poses_.poses)
    {
        int index = xy_to_grid_index(pose.position.x, pose.position.y);
        if(index >= 0 && index < local_map_.data.size()){
            local_map_.data[index] = 100;  // 障害物の個数
        }
    }

    // 更新したマップをpublishする
    pub_local_map_->publish(local_map_);
}

// マップの初期化(すべて「未知」にする)
void LocalMapCreator::init_map()
{
    local_map_.data.assign(local_map_.info.width * local_map_.info.height, -1);
}

// マップ内の場合、trueを返す
bool LocalMapCreator::in_map(const double dist, const double angle)
{
    // 指定された距離と角度がマップの範囲内か判定する
    double x = dist * std::cos(angle);
    double y = dist * std::sin(angle);

    double half_size = map_size_ / 2.0;
    return (x >= -half_size && x <= half_size && y >= -half_size && y <= half_size);
    // true返せてる？
}

// 距離と角度からグリッドのインデックスを返す
int LocalMapCreator::get_grid_index(const double dist, const double angle)
{
    double x = dist * std::cos(angle);
    double y = dist * std::sin(angle);

    return xy_to_grid_index(x, y);
}

// 座標からグリッドのインデックスを返す
int LocalMapCreator::xy_to_grid_index(const double x, const double y)
{
    int grid_x = static_cast<int>((x + (map_size_ / 2.0)) / map_reso_);
    int grid_y = static_cast<int>((y + (map_size_ / 2.0)) / map_reso_);

    int width = static_cast<int>(map_size_ / map_reso_);

    if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= width)
    {
        return -1;
    }
    
    return grid_y * width + grid_x;
}
