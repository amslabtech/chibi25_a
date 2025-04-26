#include "local_goal_creator/local_goal_creator.hpp"
LocalGoalCreator::LocalGoalCreator() : Node("LocalGoalCreator")
{
    //pubやsubの定義，tfの統合
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/global_path", rclcpp::QoS(10),std::bind(&LocalGoalCreator::pathCallback, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/estimated_pose", rclcpp::QoS(10),std::bind(&LocalGoalCreator::poseCallback, this, std::placeholders::_1));

    local_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/local_goal", rclcpp::QoS(10));
    //初期値の設定
    hz_ = this->declare_parameter<int>("hz", 10);// ループ周期 [Hz]
    index_step_ = this->declare_parameter<int>("index_step", 15);// １回で更新するインデックス数
    goal_index_ = this->declare_parameter<int>("goal_index", 3);// グローバルパス内におけるローカルゴールのインデックス
    target_distance_ = this->declare_parameter<double>("target_distance", 0.0);// 現在位置-ゴール間の距離 [m]

    goal_.header.frame_id = "map";
}

void LocalGoalCreator::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)//subのコールバック関数
{
    pose_=*msg;
    //RCLCPP_INFO(this->get_logger(),"OK1");
}

void LocalGoalCreator::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)//subのコールバック関数
{
    path_=*msg;
    //RCLCPP_INFO(this->get_logger(),"OK2");
    is_path_ = true;
}

int LocalGoalCreator::getOdomFreq()//hzを返す関数（無くてもいい）
{
    return hz_;
}

void LocalGoalCreator::process()//main文ので実行する関数
{
    //pathが読み込めた場合にpublishGoal関数を実行
    // if (!is_path_) {
    //     return;
    // }
    //RCLCPP_INFO(this->get_logger(),"OK3");
    publishGoal();
}

void LocalGoalCreator::publishGoal()
{
    int path_size = path_.poses.size();
    if (goal_index_ >= path_size) {return;}  // 最後のゴールに到達した場合
    //RCLCPP_INFO(this->get_logger(),"OK");
    //ゴールまでの距離の計算を行う
    target_distance_ = getDistance();
    //設定値に応じて，ゴール位置の変更を行う
    double goal_distance = 1.5; 
    //RCLCPP_INFO(this->get_logger(),"this:%f goal.distance:%f",target_distance_,goal_distance);
    if( target_distance_ < goal_distance ){
        //RCLCPP_INFO(this->get_logger(),"OK4");
        goal_index_ += index_step_;
        goal_.header.stamp = this->now();
        goal_.point = path_.poses[goal_index_].pose.position;
        local_goal_pub_->publish(goal_);
    }
    
}

double LocalGoalCreator::getDistance()//距離計算関数（使わなくても平気）
{
    RCLCPP_INFO(this->get_logger(),"%f %f",path_.poses[goal_index_].pose.position.x,pose_.pose.position.x);
    double dx = path_.poses[goal_index_].pose.position.x - pose_.pose.position.x;
    double dy = path_.poses[goal_index_].pose.position.y - pose_.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}