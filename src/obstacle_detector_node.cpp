#include "obstacle_detector/obstacle_detector.hpp"
int main(int argc, char *argv[])

{
    rclcpp::init(argc, argv); // ノードの初期化
    // std::shared_ptr<ObstacleDetector> dwa = std::make_shared<ObstacleDetector>();
    auto obstacle_detector = std::make_shared<ObstacleDetector>();  // ノードの作成
    rclcpp::Rate loop_rate(10.0); // 制御周波数の設定（10Hz）（必要？）   

    while(rclcpp::ok())
    {
        obstacle_detector->process();
        rclcpp::spin_some(obstacle_detector);   // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
    rclcpp::shutdown();
    return 0;
}
