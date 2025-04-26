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
    this->declare_parameter("ignore_angle_range_list", std::vector<double>({M_PI*0.22, M_PI*0.25, M_PI*0.72, M_PI*0.76}));

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
    this->get_parameter("ignore_angle_range_list", ignore_angle_range_list_);   

    // Subscriberの設定
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(10).reliable(), std::bind(
            &Localizer::map_callback, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(10).reliable(), std::bind(
            &Localizer::odom_callback, this, std::placeholders::_1));
    sub_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(10).reliable(), std::bind(
            &Localizer::laser_callback, this, std::placeholders::_1));

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
    OdomModel odom_model_(ff_, fr_, rf_, rr_);
}

// mapのコールバック関数
void Localizer::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = *msg;
    flag_map_ = true;
    printf("map call ok\n");
}

// odometryのコールバック関数
void Localizer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    prev_odom_ = last_odom_;
    last_odom_ = *msg;
    flag_odom_ = true;
    // printf("%f -> %f\n", prev_odom_.pose.pose.position.x, last_odom_.pose.pose.position.x);
}

// laserのコールバック関数
void Localizer::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    laser_ = *msg;
    flag_laser_ = true;
    // printf("laser call ok\n");
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

    printf("particle weight %f\n", particles_[0].weight());
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
    std::normal_distribution<double> norm_dist(mean, stddev);
    return norm_dist(engine_);
}

// パーティクルの重みの初期化
void Localizer::reset_weight()
{
    for(auto& p : particles_){
        p.set_weight(1.0/particles_.size());
    }
}

// map座標系からみたodom座標系の位置と姿勢をtfでbroadcast
// map座標系からみたbase_link座標系の位置と姿勢，odom座標系からみたbase_link座標系の位置と姿勢から計算
void Localizer::broadcast_odom_state()
{
    flag_broadcast_ = true;
    if(flag_broadcast_)
    {
        // TF Broadcasterの実体化
        static std::shared_ptr<tf2_ros::TransformBroadcaster> odom_state_broadcaster;
        // broadcasterの初期化
        odom_state_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // map座標系からみたbase_link座標系の位置と姿勢の取得
        Pose map_to_base = estimated_pose_;

        // odom座標系からみたbase_link座標系の位置と姿勢の取得
        Pose odom_to_base;
        odom_to_base.x_ = last_odom_.pose.pose.position.x;
        odom_to_base.y_ = last_odom_.pose.pose.position.y;
        odom_to_base.yaw_ = tf2::getYaw(last_odom_.pose.pose.orientation);

        // map座標系からみたodom座標系の位置と姿勢を計算（回転行列を使った単純な座標変換）
        const double map_to_odom_x = map_to_base.x() - odom_to_base.x();
        const double map_to_odom_y = map_to_base.y() - odom_to_base.y();
        const double map_to_odom_yaw = map_to_base.yaw() - odom_to_base.yaw();

        // yawからquaternionを作成
        tf2::Quaternion map_to_odom_quat;
        map_to_odom_quat.setRPY(0, 0, map_to_odom_yaw);

        // odom座標系odomの位置姿勢情報格納するための変数
        geometry_msgs::msg::TransformStamped odom_state;

        // 現在の時間の格納
        odom_state.header.stamp = get_clock()->now();

        // 親フレーム・子フレームの指定
        odom_state.header.frame_id = map_.header.frame_id;
        odom_state.child_frame_id  = last_odom_.header.frame_id;

        // map座標系からみたodom座標系の原点位置と方向の格納
        odom_state.transform.translation.x = map_to_odom_x;
        odom_state.transform.translation.y = map_to_odom_y;
        odom_state.transform.rotation.x = map_to_odom_quat.x();
        odom_state.transform.rotation.y = map_to_odom_quat.y();
        odom_state.transform.rotation.z = map_to_odom_quat.z();
        odom_state.transform.rotation.w = map_to_odom_quat.w();

        // tf情報をbroadcast(座標系の設定)
        odom_state_broadcaster->sendTransform(odom_state);

        // printf("broadcast ok\n");
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
    const double prev_yaw = tf2::getYaw(prev_odom_.pose.pose.orientation);
    const double last_yaw = tf2::getYaw(last_odom_.pose.pose.orientation);
    const double dx = last_odom_.pose.pose.position.x - prev_odom_.pose.pose.position.x;
    const double dy = last_odom_.pose.pose.position.y - prev_odom_.pose.pose.position.y;
    const double dyaw = normalize_angle(last_yaw - prev_yaw);

    // 方向、移動量の計算
    const double length = hypot(dx, dy);
    const double direction = normalize_angle(atan2(dy, dx) - prev_yaw);
    // printf("%f, %f\n", length, direction);

    // 標準偏差の設定
    odom_model_.set_dev(length, dyaw);

    // パーティクルの位置更新
    for(auto& p : particles_){
        p.pose_.move(length, direction, dyaw, odom_model_.get_fw_noise(), odom_model_.get_rot_noise());
    }
    // printf("mu ok\n");
}

// 観測更新
// パーティクルの尤度を計算し，重みを更新
// 位置推定，リサンプリングなどを行う
void Localizer::observation_update()
{
    for(auto& p : particles_){
        p.set_weight(p.weight() * p.likelihood(map_, laser_, sensor_noise_ratio_, laser_step_, ignore_angle_range_list_));
    }
    // パーティクル1つのレーザ1本における平均尤度を算出
    const double alpha = calc_marginal_likelihood()/((laser_.ranges.size()/laser_step_) * particles_.size());
    // printf("alpha %f\n", alpha);

    normalize_belief();
    if(alpha < alpha_th_ && reset_counter < reset_count_limit_){
        estimate_pose();
        expansion_resetting();
        reset_counter++;
    }
    else{
        estimate_pose();
        resampling(alpha);
        reset_counter = 0;
    }
    // printf("ou ok\n");
}

// 周辺尤度の算出
double Localizer::calc_marginal_likelihood()
{
    double sum = 0.0;
    for(const auto& p : particles_){
        sum += p.weight();
    }
    // printf("sum %f\n", sum);
    return sum;
}

// 推定位置の決定
// 算出方法は複数ある（平均，加重平均，中央値など...）
void Localizer::estimate_pose()
{
    double estimate_x = 0.0;
    double estimate_y = 0.0;
    double estimate_yaw = 0.0;
    double max_weight = 0.0;
    // printf("weight %f\n", max_weight);
    for(const auto& p : particles_){
        estimate_x += p.pose_.x() * p.weight();
        estimate_y += p.pose_.y() * p.weight();
        // printf("p %f\n", p.weight());
        // if(max_weight < p.weight()){
        //     estimate_yaw = p.pose_.yaw();
        //     max_weight = p.weight();
        //     // printf("max %f\n", max_weight);
        // }
        estimate_yaw += p.pose_.yaw() * p.weight();
    }
    estimated_pose_.set(estimate_x, estimate_y, estimate_yaw);
    // printf("yaw %f\n", estimate_yaw);
}

// 重みの正規化
void Localizer::normalize_belief()
{
    const double weight_sum = calc_marginal_likelihood();
    for(auto& p : particles_){
        p.set_weight(p.weight()/weight_sum);
    }
}

// 膨張リセット（EMCLの場合）
void Localizer::expansion_resetting()
{
    for(auto& p : particles_){
        const double x = norm_rv(p.pose_.x(), expansion_x_dev_);
        const double y = norm_rv(p.pose_.y(), expansion_y_dev_);
        const double yaw = norm_rv(p.pose_.yaw(), expansion_yaw_dev_);
        p.pose_.set(x, y, yaw);
        p.pose_.normalize_angle();
    }
    reset_weight();
}

// リサンプリング（系統サンプリング）
// 周辺尤度に応じてパーティクルをリサンプリング
void Localizer::resampling(const double alpha)
{
    // printf("resampling start\n");
    // パーティクルの重みを積み上げたリストを作成
    std::vector<double> accum;
    double sum_weight = 0.0;
    for(auto& p : particles_){
        sum_weight += p.weight();
        accum.push_back(sum_weight);
    }
    // printf("accum ok\n");

    // サンプリングのスタート位置とステップを設定
    const std::vector<Particle> old(particles_);
    int size = particles_.size();
    double step = sum_weight / size;
    double start = (double)rand()/RAND_MAX * step;
    // printf("set ok\n");

    // particle数の動的変更
    // if(alpha>0.8 && particle_num_>max_particle_num_){
    //     particle_num_ -= particle_num_/6;
    //     size = particle_num_;
    //     // printf("p_num -\n");
    // }else if(alpha<0.3 && particle_num_>min_particle_num_){
    //     particle_num_ += particle_num_/3;
    //     size = particle_num_;
    //     // printf("p_num +\n");
    // }

    // サンプリングするパーティクルのインデックスを保持
    std::vector<int> index;
    // index.reserve(particle_num_);
    int num = 0;
    int prev_num = 0;
    for(int i=0; i<size; i++)
    {
        while(accum[num] <= start + i*step){
            num += 1;
            if(num == size){
                exit(1);
            }
        }
        index.push_back(num);
    }

    // リサンプリング
    particles_.clear();
    for(int i=0; i<size; i++){
        particles_.push_back(old[index[i]]);
    }

    // 重みを初期化
    // normalize_belief();
    reset_weight();
    // printf("resample ok\n");
}

// 推定位置のパブリッシュ
void Localizer::publish_estimated_pose()
{
    estimated_pose_msg_.pose.position.x = estimated_pose_.x();
    estimated_pose_msg_.pose.position.y = estimated_pose_.y();

    tf2::Quaternion q;
    q.setRPY(0, 0, estimated_pose_.yaw());
    tf2::convert(q, estimated_pose_msg_.pose.orientation);

    pub_estimated_pose_ -> publish(estimated_pose_msg_);
    // printf("ep pub ok\n");
}

// パーティクルクラウドのパブリッシュ
// パーティクル数が変わる場合，リサイズする
void Localizer::publish_particles()
{
    is_visible_ = true;
    if(is_visible_)
    {
        particle_cloud_msg_.poses.clear();
        geometry_msgs::msg::Pose pose;

        for(const auto& p : particles_)
        {
            pose.position.x = p.pose_.x();
            pose.position.y = p.pose_.y();

            tf2::Quaternion q;
            q.setRPY(0, 0, p.pose_.yaw());
            tf2::convert(q, pose.orientation);

            particle_cloud_msg_.poses.push_back(pose);
        }
        pub_particle_cloud_ -> publish(particle_cloud_msg_);   
        // printf("par pub ok\n");     
    }
}