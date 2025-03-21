#include "localizer/localizer.hpp"

// デフォルトコンストラクタ
// パラメータの宣言と取得
// Subscriber，Publisherの設定
// frame idの設定
// パーティクルクラウドのメモリの確保
// odometryのモデルの初期化
Localizer::Localizer() : Node("team_localizer")
{ 
    // パラメータの宣言
    this->declare_parameter("hz", 10);
    this->declare_parameter("particle_num", 500);
    this->declare_parameter("max_particle_num", 1000);
    this->declare_parameter("min_particle_num", 100);
    this->declare_parameter("move_dist_th", 0.2);
    this->declare_parameter("init_x", 0.0);
    this->declare_parameter("init_y", 0.0);
    this->declare_parameter("init_yaw", 0.0);
    this->declare_parameter("init_x_dev", 0.50);
    this->declare_parameter("init_y_dev", 0.65);
    this->declare_parameter("init_yaw_dev", 0.50);
    this->declare_parameter("alpha_th", 0.0017);
    this->declare_parameter("reset_count_limit", 5);
    this->declare_parameter("expansion_x_dev", 0.05);
    this->declare_parameter("expansion_y_dev", 0.05);
    this->declare_parameter("expansion_yaw_dev", 0.1);
    this->declare_parameter("laser_step", 10);
    this->declare_parameter("sensor_noise_ratio", 0.03);
    this->declare_parameter("ff", 0.17);
    this->declare_parameter("fr", 0.0005);
    this->declare_parameter("rf", 0.13);
    this->declare_parameter("rr", 0.2);

    // パラメータの取得
    this->get_parameter("hz", hz_);
    this->get_parameter("particle_num", particle_num_);
    this->get_parameter("max_particle_num", max_particle_num_);
    this->get_parameter("min_particle_num", min_particle_num_);
    this->get_parameter("move_dist_th", move_dist_th_);
    this->get_parameter("init_x", init_x_);
    this->get_parameter("init_y", init_y_);
    this->get_parameter("init_yaw", init_yaw_);
    this->get_parameter("init_x_dev", init_x_dev_);
    this->get_parameter("init_y_dev", init_y_dev_);
    this->get_parameter("init_yaw_dev", init_yaw_dev_);
    this->get_parameter("alpha_th", alpha_th_);
    this->get_parameter("reset_count_limit", reset_count_limit_);
    this->get_parameter("expansion_x_dev", expansion_x_dev_);
    this->get_parameter("expansion_y_dev", expansion_y_dev_);
    this->get_parameter("expansion_yaw_dev", expansion_yaw_dev_);
    this->get_parameter("laser_step", laser_step_);
    this->get_parameter("sensor_noise_ratio", sensor_noise_ratio_);
    this->get_parameter("ff", ff_);
    this->get_parameter("fr", fr_);
    this->get_parameter("rf", rf_);
    this->get_parameter("rr", rr_);

    // Subscriberの設定
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(10).reliable(), std::bind(
            &Localizer::map_callback, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(10).reliable(), std::bind(
            &thirdChallenge::odom_callback, this, std::placeholders::_1));
    sub_Laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(10).reliable(), std::bind(
            &thirdChallenge::laser_callback, this, std::placeholders::_1));

    // Publisherの設定
    pub_estimated_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/estimated_pose", rclcpp::QoS(10).reliable());
    pub_particle_cloud_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/particle_cloud", rclcpp::QoS(10).reliable());

    // frame idの設定
    estimated_pose_msg_.header.frame_id = "map";
    particle_cloud_msg_.header.frame_id = "map";

    // パーティクルクラウドのメモリの確保
    particle_cloud_msg_.poses.reserve(max_particle_num_);

    // odometryのモデルの初期化
    odom_model_.OdomModel(ff_, fr_, rf_, rr_);
}

// mapのコールバック関数
void Localizer::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = *msg;
    flag_map_ = true;
}

// odometryのコールバック関数
void Localizer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    prev_odom_ = last_odom_;
    last_odom_ = *msg;
    flag_odom_ = true;
}

// laserのコールバック関数
void Localizer::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    laser_ = *msg;
    flag_laser_ = true;
}

// hz_を返す関数
int Localizer::getOdomFreq()
{
    return hz_;
}

// ロボットとパーティクルの推定位置の初期化
void Localizer::initialize()
{
    // 推定位置の初期化
    estimated_pose_.set(init_x_, init_y_, init_yaw_);
    
    Particle particle;

    // 初期位置近傍にパーティクルを配置
    for(int i=0; i<particle_num_; i++){
        const double x = norm_rv(init_x_, init_x_dev_);
        const double y = norm_rv(init_y_, init_y_dev_);
        const double yaw = norm_rv(init_yaw_, init_yaw_);
        particle.pose_.set(x, y, yaw);
        particle.pose_.normalize_angle();
        particles_.push_back(particle);
    }

    // パーティクルの重みの初期化
    reset_weight();
}

// main文のループ内で実行される関数
// tfのbroadcastと位置推定，パブリッシュを行う
void Localizer::process()
{
    if(flag_map_ && flag_odom_ && flag_laser_)
    {
        broadcast_odom_state();
        localize();
        publish_estimated_pose();
        publish_particles();
    }

}

// 適切な角度(-M_PI ~ M_PI)を返す
double Localizer::normalize_angle(double angle)
{
    while(M_PI < angle) angle -= 2*M_PI;
    while(angle < -M_PI) angle += 2*M_PI;
    return angle;
}

// ランダム変数生成関数（正規分布）
double Localizer::norm_rv(const double mean, const double stddev)
{
    std::normal_distribution<> norm_dist(mean, stddev);
    return norm_dist(engine_);
}

// パーティクルの重みの初期化
void Localizer::reset_weight()
{
    for(auto& p : particles_){
        p.set_weight(1/particles.size());
    }
}

// map座標系からみたodom座標系の位置と姿勢をtfでbroadcast
// map座標系からみたbase_link座標系の位置と姿勢，odom座標系からみたbase_link座標系の位置と姿勢から計算
void Localizer::broadcast_odom_state()
{
    if(flag_broadcast_)
    {
        // TF Broadcasterの実体化
        static std::shared_ptr<tf2_ros::TransformBroadcaster> odom_state_broadcaster;
        // broadcasterの初期化
        odom_state_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // map座標系からみたbase_link座標系の位置と姿勢の取得

        // odom座標系からみたbase_link座標系の位置と姿勢の取得

        // map座標系からみたodom座標系の位置と姿勢を計算（回転行列を使った単純な座標変換）

        // yawからquaternionを作成
        tf2::Quaternion map_to_odom_quat;

        // odom座標系odomの位置姿勢情報格納するための変数
        geometry_msgs::msg::TransformStamped odom_state;

        // 現在の時間の格納

        // 親フレーム・子フレームの指定
        odom_state.header.frame_id = map_.header.frame_id;
        odom_state.child_frame_id  = last_odom_.header.frame_id;

        // map座標系からみたodom座標系の原点位置と方向の格納

        // tf情報をbroadcast(座標系の設定)
    }

}

// 自己位置推定
// 動作更新と観測更新を行う
void Localizer::localize()
{
    motion_update();
    observation_update();
}

// 動作更新
// ロボットの微小移動量を計算し，パーティクルの位置をノイズを加えて更新
void Localizer::motion_update()
{
    // 微小移動量の計算
    const double prev_yaw = tf2::getYaw(prev_odom_.pose.pose.orientatin);
    const double last_yaw = tf2::getYaw(last_odom_.pose.pose.orientatin);
    const double dx = last_odom_.pose.pose.position.x - prev_odom_.pose.pose.position.x;
    const double dy = last_odom_.pose.pose.position.y - prev_odom_.pose.pose.position.y;
    const double dyaw = normalize_angle(last_yaw - prev_yaw);

    // 方向、移動量の計算
    const double length = hypot(dx, dy);
    const double direction = normalize_angle(atan2(dy, dx));

    // 標準偏差の設定
    odom_model_.set_dev(length, direction);

    // パーティクルの位置更新
    for(auto& p : particles_){
        p.pose_.move(length, direction, dyaw, odom_model_.get_fw_noise(), odom_model_.get_rot_noise());
    }
}

// 観測更新
// パーティクルの尤度を計算し，重みを更新
// 位置推定，リサンプリングなどを行う
void Localizer::observation_update()
{
    // パーティクル1つのレーザ1本における平均尤度を算出
    const double alpha;
}

// 周辺尤度の算出
double Localizer::calc_marginal_likelihood()
{

}

// 推定位置の決定
// 算出方法は複数ある（平均，加重平均，中央値など...）
void Localizer::estimate_pose()
{
    
}

// 重みの正規化
void Localizer::normalize_belief()
{

}

// 膨張リセット（EMCLの場合）
void Localizer::expansion_resetting()
{

}

// リサンプリング（系統サンプリング）
// 周辺尤度に応じてパーティクルをリサンプリング
void Localizer::resampling(const double alpha)
{
    // パーティクルの重みを積み上げたリストを作成
    std::vector<double> accum;

    // サンプリングのスタート位置とステップを設定
    const std::vector<Particle> old(particles_);
    int size = particles_.size();

    // particle数の動的変更

    // サンプリングするパーティクルのインデックスを保持

    // リサンプリング

    // 重みを初期化
}

// 推定位置のパブリッシュ
void Localizer::publish_estimated_pose()
{

}

// パーティクルクラウドのパブリッシュ
// パーティクル数が変わる場合，リサイズする
void Localizer::publish_particles()
{
    if(is_visible_)
    {
        
    }
}