cmake_minimum_required(VERSION 3.5)
<<<<<<< HEAD
project(team_a_local_goal_creator)
=======
project(team_local_goal_creator)
>>>>>>> 34d755d91456c3df8feffe402ee154d9cd5154c6

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# package.xmlのdependをチェックして自動的にfind_packageしてくれる
ament_auto_find_build_dependencies()
# 実行ファイルを作成する
ament_auto_add_executable(${PROJECT_NAME}_node
  src/local_goal_creator.cpp
  src/local_goal_creator_node.cpp)
# launchファイルをインストール（ament_package_indexが探せる場所に置く）
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
# ヘッダーのインストールや共有ライブラリのエクスポート等を自動で行ってくれる
ament_auto_package()