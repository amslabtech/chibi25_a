#include "obstacle_detector.hpp"

using namespace std::chrono_literals;

ObstacleDetector::ObstacleDetector() : Node("ObstacleDetector")
{
    // YAMLファイルからパラメータを取得
    this->declare_parameter("hz", 10);  // デフォルト値10Hz
    this->declare_parameter("laser_step", 1);  // デフォルト値 1
    this->declare_parameter("robot_frame", std::string("base_link"));
    this->declare_parameter("ignore_dist", 0.5);  // デフォルト値 0.5m

    this->get_parameter("hz", hz_);
    this->get_parameter("laser_step", laser_step_);
    this->get_parameter("robot_frame", robot_frame_);
    this->get_parameter("ignore_dist", ignore_dist_);

    RCLCPP_INFO(this->get_logger(), "Loaded Parameters: hz=%d, laser_step=%d, robot_frame=%s, ignore_dist=%.2f",
                hz_, laser_step_, robot_frame_.c_str(), ignore_dist_);

    // global変数を定義(yamlファイルからパラメータを読み込めるようにすると，パラメータ調整が楽)
    // LiDARデータのサブスクライバ
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
       "scan", 10, std::bind(&ObstacleDetector::scan_callback, this, std::placeholders::_1));
   
    // 障害物の情報をpublishするパブリッシャの作成
    obstacle_points_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("obstacle_points", 10);
}

//LiDARから障害物の情報を取得
void ObstacleDetector::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // msgを取得
    laserscan_ = *msg;
}

//一定周期で行う処理(obstacle_detectorの処理)
void ObstacleDetector::process()
{
    //scan_obstacle()を呼び出す
    scan_obstacle();  
}

//LiDARから障害物情報を取得し，障害物の座標をpublish
void ObstacleDetector::scan_obstacle()
{
    if (!laserscan_.has_value()) return;
    auto scan = laserscan_.value();
   
    geometry_msgs::msg::PointStamped obstacle_point;
    obstacle_point.header.stamp = this->get_clock()->now();
    obstacle_point.header.frame_id = robot_frame_;
   
    // 簡単な障害物検出 (最短距離の点を取得)
    float min_distance = scan.range_max;
    float min_angle = 0.0;
    for (size_t i = 0; i < scan.ranges.size(); i += laser_step_) {
        float distance = scan.ranges[i];
        float angle = scan.angle_min + i * scan.angle_increment;
       
        // 無視する範囲を判定
        if (is_ignore_scan(distance, angle)) continue;
       
        if (distance < min_distance) {
            min_distance = distance;
            min_angle = angle;
        }
    }
   
    // 障害物の座標を計算
    obstacle_point.point.x = min_distance * cos(min_angle);
    obstacle_point.point.y = min_distance * sin(min_angle);
    obstacle_point.point.z = 0.0;
   
    // Publish
    obstacle_points_pub_->publish(obstacle_point);
}


//無視するLiDAR情報の範囲の決定(LiDARがroombaの櫓の中にあり，櫓の４つの柱を障害物として検出してしまうため削除が必要)
bool ObstacleDetector::is_ignore_scan(float distance, float angle)
{
    // 無視する柱の位置（LiDAR中心から半径10cm付近）
    // const float IGNORE_RADIUS = 0.1; // 10cm
    const float TOLERANCE = 0.02; // 許容誤差 2cm

    // 4本の支柱の角度（LiDARの正面から45, 135, -45, -135度に設置）
    float pillar_angles[] = {M_PI_4, 3*M_PI_4, -M_PI_4, -3*M_PI_4};

    for (float pillar_angle : pillar_angles) {
        if (std::abs(distance - ignore_dist_) < TOLERANCE && std::abs(angle - pillar_angle) < M_PI / 18) {
            return true; // 無視する
        }
    }
    return false;
}