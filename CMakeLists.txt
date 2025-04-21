# local_path_plannerから引用
cmake_minimum_required(VERSION 3.5)
# team_obstacle_detectorである必要はあるのか？
project(team_obstacle_detector)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# find_package(rclcpp REQUIRED)  # 追加
# find_package(sensor_msgs REQUIRED)  # 追加
# find_package(geometry_msgs REQUIRED)  # 追加

# package.xmlのdependをチェックして自動的にfind_packageしてくれる
ament_auto_find_build_dependencies()

# 実行ファイルを作成する
ament_auto_add_executable(${PROJECT_NAME}_node
  src/obstacle_detector.cpp
  src/obstacle_detector_node.cpp)
# launchファイルをインストール（ament_package_indexが探せる場所に置く）
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
# ヘッダーのインストールや共有ライブラリのエクスポート等を自動で行ってくれる
ament_auto_package()