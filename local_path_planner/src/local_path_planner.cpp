/*
DWA: Dynamic Window Approach

速度について以下のようにする
velocity(vel) : 並進速度
yawrate       : 旋回速度
speed         : 速度の総称(vel, yawrate)
*/

#include "local_path_planner/local_path_planner.hpp"

using namespace std::chrono_literals;

// デフォルトコンストラクタ
// パラメータの宣言と取得
DWAPlanner::DWAPlanner() : Node("local_path_planner"), clock_(RCL_ROS_TIME)
{
    // ###### 並進速度のパラメータ ######
    this->declare_parameter("max_vel", 0.5);                 // 最高並進速度 [m/s]（計算用）
    this->declare_parameter("max_vel1", 0.5);                // 最高並進速度 [m/s]（平常時）
    this->declare_parameter("max_vel2", 0.3);                // 最高並進速度 [m/s]（減速時）
    this->declare_parameter("min_vel", -0.5);                // 最低並進速度 [m/s]

    // ###### 旋回速度のパラメータ ######
    this->declare_parameter("max_yawrate", 1.0);             // 最高旋回速度 [rad/s]（計算用）
    this->declare_parameter("max_yawrate1", 1.0);            // 最高旋回速度 [rad/s]（平常時）
    this->declare_parameter("max_yawrate2", 0.5);            // 最高旋回速度 [rad/s]（減速時）

    // ###### 加速度のパラメータ ######
    this->declare_parameter("max_accel", 0.2);               // 最大並進加速度 [m/s²]
    this->declare_parameter("max_dyawrate", 0.1);            // 最大旋回加速度 [rad/s²]

    // ###### 解像度のパラメータ ######
    this->declare_parameter("vel_reso", 0.1);                // 並進速度の解像度 [m/s]
    this->declare_parameter("yawrate_reso", 0.1);            // 旋回速度の解像度 [rad/s]

    // ###### 停止判定のパラメータ ######
    this->declare_parameter("stop_vel_th", 0.01);            // 停止状態か判断する閾値 [m/s²]
    this->declare_parameter("stop_yawrate_th", 0.01);        // 停止状態か判断する閾値 [rad/s]

    // ###### 軌跡生成のパラメータ ######
    this->declare_parameter("dt", 0.1);                      // シミュレーションの時間間隔 [s]
    this->declare_parameter("predict_time", 3.0);            // 軌跡予測時間 [s]（計算用）
    this->declare_parameter("predict_time1", 3.0);           // 軌跡予測時間 [s]（平常時）
    this->declare_parameter("predict_time2", 2.0);           // 軌跡予測時間 [s]（減速時）

    // ###### ロボットサイズのパラメータ ######
    this->declare_parameter("roomba_radius", 0.2);           // Roombaの半径 [m]
    this->declare_parameter("radius_margin", 0.1);           // 半径のマージン [m]（計算用）
    this->declare_parameter("radius_margin1", 0.1);          // 半径のマージン [m]（平常時）
    this->declare_parameter("radius_margin2", 0.2);          // 半径のマージン [m]（減速時）

    // ###### ゴール関連のパラメータ ######
    this->declare_parameter("goal_tolerance", 0.05);         // ゴール到達判定の許容誤差 [m]
    this->declare_parameter("search_range", 1.0);            // 評価関数 `dist` で探索する範囲 [m]

    // ###### 重み定数 ######
    this->declare_parameter("weight_heading", 1.0);          // ゴールへの向きやすさに関する重み
    this->declare_parameter("weight_heading1", 1.0);         // ゴールへの向きやすさに関する重み（平常時）
    this->declare_parameter("weight_heading2", 0.5);         // ゴールへの向きやすさに関する重み（減速時）
    this->declare_parameter("weight_dist", 1.0);             // 障害物との距離に関する重み
    this->declare_parameter("weight_dist1", 1.0);            // 障害物との距離に関する重み（平常時）
    this->declare_parameter("weight_dist2", 0.5);            // 障害物との距離に関する重み（減速時）
    this->declare_parameter("weight_vel", 1.0);              // 速度に関する重み

    // ###### tf 変換用フレーム ######
    this->declare_parameter("robot_frame", "base_link");      // ロボットのフレーム名

    // ###### 設定されたパラメータの取得 ######
    this->get_parameter("max_vel", max_vel_);
    this->get_parameter("max_vel1", max_vel1_);
    this->get_parameter("max_vel2", max_vel2_);
    this->get_parameter("min_vel", min_vel_);
    this->get_parameter("max_yawrate", max_yawrate_);
    this->get_parameter("max_yawrate1", max_yawrate1_);
    this->get_parameter("max_yawrate2", max_yawrate2_);
    this->get_parameter("max_accel", max_accel_);
    this->get_parameter("max_dyawrate", max_dyawrate_);
    this->get_parameter("vel_reso", vel_reso_);
    this->get_parameter("yawrate_reso", yawrate_reso_);
    this->get_parameter("stop_vel_th", stop_vel_th_);
    this->get_parameter("stop_yawrate_th", stop_yawrate_th_);
    this->get_parameter("dt", dt_);
    this->get_parameter("predict_time", predict_time_);
    this->get_parameter("predict_time1", predict_time1_);
    this->get_parameter("predict_time2", predict_time2_);
    this->get_parameter("roomba_radius", roomba_radius_);
    this->get_parameter("radius_margin", radius_margin_);
    this->get_parameter("radius_margin1", radius_margin1_);
    this->get_parameter("radius_margin2", radius_margin2_);
    this->get_parameter("goal_tolerance", goal_tolerance_);
    this->get_parameter("search_range", search_range_);
    this->get_parameter("weight_heading", weight_heading_);
    this->get_parameter("weight_heading1", weight_heading1_);
    this->get_parameter("weight_heading2", weight_heading2_);
    this->get_parameter("weight_dist", weight_dist_);
    this->get_parameter("weight_dist1", weight_dist1_);
    this->get_parameter("weight_dist2", weight_dist2_);
    this->get_parameter("weight_vel", weight_vel_);
    this->get_parameter("robot_frame", robot_frame_);


    // ###### tf_buffer_とtf_listenerを初期化 ######
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ####### Subscriber #######
    sub_local_goal_ = this->create_subscription<geometry_msgs::msg::PointStamped>("local_goal", 10,
    [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {this->local_goal_callback(msg);});
    sub_obs_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>("obs_poses", 10,
    [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {this->obs_poses_callback(msg);});
    // ###### Publisher ######
    pub_cmd_speed_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>("/roomba/control", rclcpp::QoS(1).reliable()); 
    pub_optimal_path_ = this->create_publisher<nav_msgs::msg::Path>("optimal_path", 10);
    pub_predict_path_ = this->create_publisher<nav_msgs::msg::Path>("predict_path", 10);
}

// local_goalのコールバック関数
// local_goalはマップ座標系(map)だが，実際の移動に合わせるためにルンバ座標系(base_link)に変換する処理を行う
void DWAPlanner::local_goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    printf("local_goal_callback\n");
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        // tfを取得，成功したらフラグを立てる
        transform = tf_buffer_->lookupTransform(
            "base_link",            // 変換先フレーム
            msg->header.frame_id,   // 変換元フレーム (map)
            msg->header.stamp);     // 受信した時刻

        flag_local_goal_ = true;
    }
    catch(tf2::TransformException& ex)
    {
        // 取得に失敗した場合，警告を出力しフラグをリセット
        RCLCPP_WARN(this->get_logger(), "Failed to transform local goal: %s", ex.what());
        flag_local_goal_ = false;
        return;

    }
    // 取得した変換を用いて，local_goal_ をロボット座標系 (base_link) に変換

     tf2::doTransform(*msg, local_goal_, transform);

}

// obs_posesのコールバック関数
void DWAPlanner::obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    obs_poses_ = *msg;
    flag_obs_poses_ = true;
}

// hzを返す関数
int DWAPlanner::get_freq()
{
    return hz_;
}

// 唯一，main文で実行する関数
// can_move()がTrueのとき，calc_final_input()を実行し，速度と旋回速度を計算
// それ以外の場合は，速度と旋回速度を0にする
void DWAPlanner::process()
{
    double velocity = 0.0;
    double yawrate = 0.0;

    if (can_move()) {
        // 最適な制御入力を計算
        auto final_input = calc_final_input();
        velocity = final_input[0];
        yawrate = final_input[1];
        
        roomba_.x += final_input[0] * dt_ * cos(roomba_.yaw);
        roomba_.y += final_input[0] * dt_ * sin(roomba_.yaw);
        roomba_.yaw += final_input[1] * dt_;
        roomba_.yaw = normalize_angle(roomba_.yaw);

    }

    // ロボットへの制御入力を実行
    roomba_control(velocity, yawrate);
}

// ゴールに着くまでTrueを返す
bool DWAPlanner::can_move()
{
    if (!flag_local_goal_) {
        return false;
    }

    // 目標地点と現在位置の距離を計算
    double dx = local_goal_.point.x - roomba_.x;
    double dy = local_goal_.point.y - roomba_.y;
    double distance_to_goal = sqrt(dx * dx + dy * dy);

    // 目標地点に到達しているか判定
    if (distance_to_goal < goal_tolerance_) {
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        return false;
    }

    return true;
}

// Roombaの制御入力を行う
void DWAPlanner::roomba_control(const double velocity, const double yawrate)
{
    cmd_speed_.mode = 11; 
    cmd_speed_.cntl.linear.x = velocity;
    cmd_speed_.cntl.angular.z = yawrate;
    pub_cmd_speed_->publish(cmd_speed_);
}

// 最適な制御入力を計算
std::vector<double> DWAPlanner::calc_final_input()
{
    // 変数を定義，初期化
    std::vector<double> input{0.0, 0.0};          // {velocity, yawrate}
    std::vector<std::vector<State>> trajectories; // すべての軌跡格納用
    double max_score = -1e6;                      // 評価値の最大値格納用
    int index_of_max_score = 0;                   // 評価値の最大値に対する軌跡のインデックス格納用

    // 旋回状況に応じた減速機能
    change_mode();

    // ダイナミックウィンドウを計算
    calc_dynamic_window();

    int i = 0; // 現在の軌跡のインデックス保持用
    // ###### 並進速度と旋回速度のすべての組み合わせを評価 ######
    for (double v = dw_.min_vel; v <= dw_.max_vel; v += vel_reso_) {
        for (double w = dw_.min_yawrate; w <= dw_.max_yawrate; w += yawrate_reso_) {

            // 軌跡生成
            std::vector<State> traj = calc_traj(v, w);
            trajectories.push_back(traj);

            // 評価関数でスコアを計算
            double score = calc_evaluation(traj);

            // 最高評価の軌跡を採用
            if (score > max_score) {
                max_score = score;
                index_of_max_score = i;
            }
            i++;
        }
    }

    //最適な軌跡の決定
    if (index_of_max_score != -1) {
        input[0] = trajectories[index_of_max_score][0].velocity; // 並進速度
        input[1] = trajectories[index_of_max_score][0].yawrate;  // 旋回速度
    }

    // 現在速度の記録
    roomba_.velocity = input[0];
    roomba_.yawrate  = input[1];

    // ###### pathの可視化 #######
    if (index_of_max_score != -1) {
        visualize_traj(trajectories[index_of_max_score], pub_optimal_path_, clock_.now());
    }

    return input;
}

// 旋回状況に応じた減速機能
// ロボットの旋回速度や速度によって減速モードに切り替える（普段よりも遅く動く）
void DWAPlanner::change_mode()
{
    if (fabs(roomba_.yawrate) > turn_thres_yawrate_ || fabs(roomba_.velocity) < avoid_thres_vel_) {
        // 減速モードに切り替え
        max_vel_ = max_vel2_;
        max_yawrate_ = max_yawrate2_;
        predict_time_ = predict_time2_;
        radius_margin_ = radius_margin2_;
        weight_heading_ = weight_heading2_;
        weight_dist_ = weight_dist2_;
    } else {
        // 通常モード
        max_vel_ = max_vel1_;
        max_yawrate_ = max_yawrate1_;
        predict_time_ = predict_time1_;
        radius_margin_ = radius_margin1_;
        weight_heading_ = weight_heading1_;
        weight_dist_ = weight_dist1_;
    }
}

// Dynamic Windowを計算
void DWAPlanner::calc_dynamic_window()
{
    // ###### 車両モデルによるWindow ######
    double Vs[] = { min_vel_, max_vel_, -max_yawrate_, max_yawrate_ };

    // ####### 運動モデルによるWindow #######
    double Vd[] = {
        roomba_.velocity - max_accel_ * dt_,
        roomba_.velocity + max_accel_ * dt_,
        roomba_.yawrate - max_dyawrate_ * dt_,
        roomba_.yawrate + max_dyawrate_ * dt_
    };

    // ###### 最終的なDynamic Window ######
    dw_.min_vel     = std::max(Vs[0], Vd[0]);
    dw_.max_vel     = std::min(Vs[1], Vd[1]);
    dw_.min_yawrate = std::max(Vs[2], Vd[2]);
    dw_.max_yawrate = std::min(Vs[3], Vd[3]);
}

// 指定された予測時間までロボットの状態を更新し，予測軌跡を生成
std::vector<State> DWAPlanner::calc_traj(const double velocity, const double yawrate)
{
    std::vector<State> traj;
    State state = roomba_; // 現在のロボット状態をコピー

    double time = 0.0;
    while (time <= predict_time_) {
        // 仮想ロボットの状態を更新
        move(state, velocity, yawrate);

        // 角度を正規化
        state.yaw = normalize_angle(state.yaw);

        // 更新した状態を格納
        traj.push_back(state);

        // 時間を更新
        time += dt_;
    }
    return traj;
}

// 予測軌跡作成時における仮想ロボットを移動
void DWAPlanner::move(State& state, const double velocity, const double yawrate)
{
    state.x += velocity * cos(state.yaw) * dt_;
    state.y += velocity * sin(state.yaw) * dt_;

    // 旋回運動による角度の更新
    state.yaw += yawrate * dt_;
}

// angleを適切な角度(-M_PI ~ M_PI)の範囲にして返す
double DWAPlanner::normalize_angle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

// 評価関数を計算
double DWAPlanner::calc_evaluation(const std::vector<State>& traj)
{
    const double heading_score  = weight_heading_ * calc_heading_eval(traj);
    const double distance_score = weight_dist_    * calc_dist_eval(traj);
    const double velocity_score = weight_vel_     * calc_vel_eval(traj);

    const double total_score = heading_score + distance_score + velocity_score;

    return total_score;
}

// headingの評価関数を計算
// 軌跡のゴール方向への向きやすさを評価する関数
double DWAPlanner::calc_heading_eval(const std::vector<State>& traj)
{
    if (traj.empty()) return 0.0;

    // 軌跡の終端状態を取得
    State last_state = traj.back();

    // ゴール方向を計算
    double goal_direction = atan2(local_goal_.point.y - last_state.y, 
                                  local_goal_.point.x - last_state.x);

    // ゴール方向と終端状態の向きの差を計算
    double angle_diff = normalize_angle(goal_direction - last_state.yaw);

    // cosで評価値を計算（1に近いほど向きが良い）
    return cos(angle_diff);
}

// distの評価関数を計算
// 軌跡の障害物回避性能を評価する関数
double DWAPlanner::calc_dist_eval(const std::vector<State>& traj)
{
    if (traj.empty()) return 0.0;

    double min_dist = 1e6; // 初期値（非常に大きな値）

    for (const auto& state : traj) {
        for (const auto& obs_pose : obs_poses_.poses) {
            double dist = sqrt(pow(state.x - obs_pose.position.x, 2) +
                               pow(state.y - obs_pose.position.y, 2));
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
    }

    // スコアは最小距離（障害物から遠いほど良い）
    return min_dist;
}

// velocityの評価関数を計算
// 軌跡の速度評価を計算する関数
double DWAPlanner::calc_vel_eval(const std::vector<State>& traj)
{
    if (traj.empty()) return 0.0;

    // 軌跡の最終状態の速度をスコアとして使用
    return traj.back().velocity;
}

// 軌跡を可視化するための関数
void DWAPlanner::visualize_traj(const std::vector<State>& traj, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_path, rclcpp::Time now)
{
    nav_msgs::msg::Path path;
    path.header.stamp = now;
    path.header.frame_id = "map"; // 座標系

    for (const auto& state : traj) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now;
        pose.header.frame_id = "map";

        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(state.yaw * 0.5), cos(state.yaw * 0.5)));

        path.poses.push_back(pose);
    }

    // トピックに発行
    pub_local_path->publish(path);
}