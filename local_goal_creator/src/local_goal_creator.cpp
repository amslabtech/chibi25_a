#include "local_goal_creator/local_goal_creator.hpp"
LocalGoalCreator::LocalGoalCreator() : Node("LocalGoalCreator")
{
    //pubやsubの定義，tfの統合
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/global_path", rclcpp::QoS(10),std::bind(&LocalGoalCreator::pathCallback, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/estimated_pose", rclcpp::QoS(10),std::bind(&LocalGoalCreator::poseCallback, this, std::placeholders::_1));

    local_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/local_goal", rclcpp::QoS(10));
    //初期値の設定
    hz_ = this->declare_parameter<int>("hz", 10);// ループ周期 [Hz]
    index_step_ = this->declare_parameter<int>("index_step", 5);// １回で更新するインデックス数
    goal_index_ = this->declare_parameter<int>("goal_index", 25);// グローバルパス内におけるローカルゴールのインデックス
    target_distance_ = this->declare_parameter<double>("target_distance", 0.0);// 現在位置-ゴール間の距離 [m]
}

void LocalGoalCreator::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)//subのコールバック関数
{
    pose_=*msg;
}

void LocalGoalCreator::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)//subのコールバック関数
{
    path_=*msg;
    is_path_ = true;
}

int LocalGoalCreator::getOdomFreq()//hzを返す関数（無くてもいい）
{
    return hz_;
}

void LocalGoalCreator::process()//main文ので実行する関数
{
    //pathが読み込めた場合にpublishGoal関数を実行
    if (!is_path_) return;
    publishGoal();
}

void LocalGoalCreator::publishGoal()
{
<<<<<<< HEAD
<<<<<<< HEAD
=======
    int path_size = path_.poses.size();
    if (goal_index_ >= path_size) return;  // 最後のゴールに到達した場合

    //ゴールまでの距離の計算を行う
    target_distance_ = getDistance();
    //設定値に応じて，ゴール位置の変更を行う
    double goal_distance = 1.5; 
    if( target_distance_ < goal_distance ){
        goal_index_ += index_step_;
        goal_.header = path_.header;
        goal_.point = path_.poses[goal_index_].pose.position;
    }
    
    local_goal_pub_->publish(goal_);
}

double LocalGoalCreator::getDistance()//距離計算関数（使わなくても平気）
{
    double dx = goal_.point.x - pose_.pose.position.x;
    double dy = goal_.point.y - pose_.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}#include "local_goal_creator/local_goal_creator.hpp"
LocalGoalCreator::LocalGoalCreator() : Node("LocalGoalCreator")
{
    //pubやsubの定義，tfの統合
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/global_path", rclcpp::QoS(10),std::bind(&LocalGoalCreator::pathCallback, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/estimated_pose", rclcpp::QoS(10),std::bind(&LocalGoalCreator::poseCallback, this, std::placeholders::_1));

    local_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/local_goal", rclcpp::QoS(10));
    //初期値の設定
    hz_ = this->declare_parameter<int>("hz", 10);// ループ周期 [Hz]
    index_step_ = this->declare_parameter<int>("index_step", 5);// １回で更新するインデックス数
    goal_index_ = this->declare_parameter<int>("goal_index", 25);// グローバルパス内におけるローカルゴールのインデックス
    target_distance_ = this->declare_parameter<double>("target_distance", 0.0);// 現在位置-ゴール間の距離 [m]
}

void LocalGoalCreator::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)//subのコールバック関数
{
    pose_=*msg;
}

void LocalGoalCreator::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)//subのコールバック関数
{
    path_=*msg;
    is_path_ = true;
}

int LocalGoalCreator::getOdomFreq()//hzを返す関数（無くてもいい）
{
    return hz_;
}

void LocalGoalCreator::process()//main文ので実行する関数
{
    //pathが読み込めた場合にpublishGoal関数を実行
    if (!is_path_) return;
    publishGoal();
}

void LocalGoalCreator::publishGoal()
{
>>>>>>> 34d755d91456c3df8feffe402ee154d9cd5154c6
    int path_size = path_.poses.size();
    if (goal_index_ >= path_size) return;  // 最後のゴールに到達した場合

    //ゴールまでの距離の計算を行う
    target_distance_ = getDistance();
    //設定値に応じて，ゴール位置の変更を行う
    double goal_distance = 1.5; 
    if( target_distance_ < goal_distance ){
        goal_index_ += index_step_;
        goal_.header = path_.header;
        goal_.point = path_.poses[goal_index_].pose.position;
    }
    
    local_goal_pub_->publish(goal_);
}

double LocalGoalCreator::getDistance()//距離計算関数（使わなくても平気）
{
    double dx = goal_.point.x - pose_.pose.position.x;
    double dy = goal_.point.y - pose_.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}