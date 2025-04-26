#include "obstacle_detector.hpp"
// #include "local_goal_creator/local_goal_creator.hpp"  // hz_を使うため

using namespace std::chrono_literals;

ObstacleDetector::ObstacleDetector() : Node("ObstacleDetector")
{
    // YAMLファイルからパラメータを取得
    // this->declare_parameter("hz", &LocalGoalCreator::getOdomFreq());  // デフォルト値10Hz
    this->declare_parameter("hz_", 10);
    this->declare_parameter("laser_step", 1);  // デフォルト値 1
    this->declare_parameter("robot_frame", std::string("base_link"));  // yamlファイルで定義済みなので15行目のみで良いのでは？
    this->declare_parameter("ignore_dist", 0.5);  // デフォルト値 0.5m。あとで変わる？

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
    
    // RCLCPP_INFO(this->get_logger(), "Check Point 1.");
   
    // 障害物の情報をpublishするパブリッシャの作成
    obstacle_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("obstacle_poses", 10);
    // RCLCPP_INFO(this->get_logger(), "Check Point 2.");
}

//LiDARから障害物の情報を取得
void ObstacleDetector::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // msgを取得
    laserscan_ = *msg;
    // RCLCPP_INFO(this->get_logger(), "Check Point 3.");
}

//一定周期で行う処理(obstacle_detectorの処理)
void ObstacleDetector::process()
{
    //scan_obstacle()を呼び出す
    scan_obstacle();  
    // RCLCPP_INFO(this->get_logger(), "Check Point 4.");
}

//LiDARから障害物情報を取得し，障害物の座標をpublish
void ObstacleDetector::scan_obstacle()
{
    // RCLCPP_INFO(this->get_logger(), "Check Point 5.");
    if (!laserscan_.has_value()) {
        //RCLCPP_WARN(this->get_logger(), "laserscan_ is empty");
        return;
    }
    auto scan = laserscan_.value();
    //RCLCPP_INFO(this->get_logger(), "Scan angle range: min=%.3f, max=%.3f", scan.angle_min, scan.angle_max);
   
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = this->get_clock()->now();
    pose_array.header.frame_id = robot_frame_;

    // RCLCPP_INFO(this->get_logger(), "Check Point 6.");
   
    // 簡単な障害物検出 (最短距離の点を取得)
    float min_distance = scan.range_max;
    //  RCLCPP_INFO(this->get_logger(), "min_distance = %f", min_distance);
    float min_angle = 0.0;
    for (size_t i = 0; i < scan.ranges.size(); i += laser_step_) {
        float distance = scan.ranges[i];
        float angle = scan.angle_min + i * scan.angle_increment;
        // RCLCPP_INFO(this->get_logger(), "Scan angle range: min=%.3f, max=%.3f, increment=%.5f", scan.angle_min, scan.angle_max, scan.angle_increment);
        //RCLCPP_INFO(this->get_logger(), "i: %lu, distance: %.3f, angle: %.3f", i, distance, angle);  // 柱無視デバック用
       
        // 無視すべき柱をスキップ
        if (is_ignore_scan(distance, angle)) continue;
       
        if(distance < scan.range_min || distance > scan.range_max) continue;

        // 支柱っぽい距離をログ出力
        // if(distance < 0.2){
        //     RCLCPP_INFO(this->get_logger(), "支柱かも？ index = %zu, angle = %.3f, [deg] = %.1f, dist = %.3f", i, angle, angle * 180.0 / M_PI, distance);
        // }

        geometry_msgs::msg::Pose pose;
        pose.position.x = distance * cos(angle);
        pose.position.y = distance * sin(angle);
        pose.position.z = 0.0;

        // 姿勢は未使用なので identity に設定
        pose.orientation.w = 1.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;

        pose_array.poses.push_back(pose);
    }
   
    // RCLCPP_INFO(this->get_logger(), "Check Point 7.");
    // Publish
    obstacle_array_pub_->publish(pose_array);
}


//無視するLiDAR情報の範囲の決定(LiDARがroombaの櫓の中にあり，櫓の４つの柱を障害物として検出してしまうため削除が必要)
bool ObstacleDetector::is_ignore_scan(float distance, float angle)
{
    // 無視する柱の位置（LiDAR中心から半径10cm付近）
    // const float IGNORE_RADIUS = 0.1; // 10cm
    // RCLCPP_INFO(this->get_logger(), "Checking distance=%.3f angle=%.3f", distance, angle);  // 柱無視デバック用
    const float DIST_TOLERANCE = 0.08; // 許容誤差 8cm
    const float ANGLE_TOLERANCE = M_PI / 6;  // 約30度

    // 角度の正規化（正面０を中心に）
    float angle_from_front = normalize_angle(angle);  // angle_minからではなく、正面中心で見たい

    // 4本の支柱の角度（LiDARの正面から45, 135, -45, -135度に設置）
    float pillar_angles[] = {
        M_PI_4,    // 45
        3*M_PI_4,  // 135
        -M_PI_4,   // -45
        -3*M_PI_4  // -135
    };
    // RCLCPP_INFO(this->get_logger(), "Check Point 7.5.");

    float pillar_dists[] = {0.1, 0.1, 0.1, 0.1};  // それぞれの支柱の理想距離

    for (int i = 0; i < 4; ++i) {
        // RCLCPP_WARN(this->get_logger(), "Looping pillar_angle = %.3f", pillar_angle);

        float angle_diff = std::abs(normalize_angle(angle_from_front - pillar_angles[i]));
        float dist_diff = std::abs(distance - pillar_dists[i]);

        //RCLCPP_WARN(this->get_logger(), "pillar_angle=%.3f. angle=%.3f -> angle_diff = %.3f, dist_diff = %.3f", pillar_angles[i], angle, angle_diff, dist_diff);

        if (dist_diff < DIST_TOLERANCE && angle_diff < ANGLE_TOLERANCE) {
            //RCLCPP_INFO(this->get_logger(), "Ignoring pillar at dist = %.2f angle = %.2f", distance, angle);
            return true; // 無視する
        }
    }
    // RCLCPP_INFO(this->get_logger(), "Check Point 8.");
    return false;
}

float ObstacleDetector::normalize_angle(float angle){
    // RCLCPP_INFO(this->get_logger(), "Check Point 9.");
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2* M_PI;
    return angle;
}