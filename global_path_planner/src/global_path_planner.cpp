#include "global_path_planner/global_path_planner.hpp"

using namespace std::chrono_literals;

// デフォルトコンストラクタ
// パラメータの宣言と取得
Astar::Astar() : Node("team_a_path_planner"), clock_(RCL_ROS_TIME)
{
    // ###### パラメータの宣言 ######
    //declare_parameter<double>("resolution", 0.0);       // マップの解像度（m/グリッド）
    declare_parameter<double>("margin_", 0.3);          // 障害物拡張マージン（グリッド数）
    declare_parameter<std::vector<double>>("way_points_x", {-0.16,16.4,17.3,0.917,-16.1,-17.1,-0.16}); // ウェイポイントX座標リスト
    declare_parameter<std::vector<double>>("way_points_y", {0.11,-0.772,13.4,14.2,15.1,1.01,0.11}); // ウェイポイントY座標リスト
    declare_parameter<bool>("test_show", false);

    // ###### パラメータの取得 ######
    //resolution_ = get_parameter("resolution").as_double();
    margin_ = get_parameter("margin_").as_double();
    way_points_x_ = get_parameter("way_points_x").as_double_array();
    way_points_y_ = get_parameter("way_points_y").as_double_array();
    test_show_ = get_parameter("test_show").as_bool();

    // ###### global_path_とcurrent_node_のframe_id設定 ######
    global_path_.header.frame_id = "map";      // グローバルパスのフレーム
    current_node_.header.frame_id = "map";     // 現在ノード表示用フレーム

    // dataサイズの確保
    global_path_.poses.reserve(2000);


    // ####### Subscriber #######
    sub_map_ =  this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).reliable(),std::bind(&Astar::map_callback, this, std::placeholders::_1));

    // ###### Publisher ######

    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/global_path",rclcpp::QoS(1).reliable());// 最終的な経路
    pub_node_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/current_node", rclcpp::QoS(1).reliable());// 現在のノード（デバッグ用）
    pub_current_path_ = this->create_publisher<nav_msgs::msg::Path>("/current_path", rclcpp::QoS(1).reliable());// 現在の部分経路（デバッグ用）
    pub_new_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/new_map", rclcpp::QoS(1).reliable());// 拡張後のマップ
}



// mapのコールバック関数
// msgを受け取り，map_に代入，その情報をそれぞれ取得
// process()を実行
void Astar::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)  //マップの読み込み
{
    if (!msg) {
        RCLCPP_ERROR(this->get_logger(), "Received null map message");
        return;
    }
    /* Rvizから提供されるマップデータの処理:
    1. マップメタデータの取得
    2. グリッドサイズ計算
    3. 障害物拡張用マップコピー
    4. 処理開始フラグ設定 */
    map_ = *msg;                    // マップデータ保存
    new_map_ = map_;                // 障害物拡張用コピー
    width_ = map_.info.width;       // マップ幅（グリッド数）
    height_ = map_.info.height;     // マップ高さ（グリッド数）
    origin_x_ = map_.info.origin.position.x;  // マップ原点X（m）
    origin_y_ = map_.info.origin.position.y;  // マップ原点Y（m）
    resolution_=map_.info.resolution;// マップの解像度
    map_checker_ = true;            // マップ取得完了フラグ

    RCLCPP_INFO(this->get_logger(), "Map callback executed successfully");

    printf("%d %d %f %f %f \n",width_,height_,origin_x_,origin_y_,resolution_);
    process();                      // メインプロセス起動
}


// マップ全体の障害物を拡張処理（new_map_をpublishする）
void Astar::obs_expander()
{   
    if (new_map_.data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "New map data is empty, cannot expand obstacles");
        return;
    }
    /* 安全マージンを確保するため障害物周囲を拡張:
    1. 元マップの全セルを走査
    2. 障害物（値100）周囲を指定マージン分拡張
    3. 新しいマップデータをパブリッシュ */
    for(int i=0; i<map_.data.size(); ++i){
        if(map_.data[i] == 100){
            obs_expand(i);  // 個別障害物拡張処理
        }
    }
    pub_new_map_->publish(new_map_);  // 拡張マップを公開
}


// 指定されたインデックスの障害物を拡張（margin_cells分）
void Astar::obs_expand(const int index)
{
    /* 指定インデックスの障害物を周囲に拡張:
    1. グリッド座標変換（index → x,y）
    2. 周囲マージン分のセルを障害物として設定 */
    const int x = index % width_;     // Xグリッド座標
    const int y = index / width_;     // Yグリッド座標
    int margin_cells = round((margin_ / resolution_));  // マージンのセル数
    
    // 周囲マージン分ループ
    for(int dx=-margin_cells; dx<=margin_cells; ++dx){
        for(int dy=-margin_cells; dy<=margin_cells; ++dy){
            int nx = x + dx;    // 新しいX座標
            int ny = y + dy;    // 新しいY座標
            // マップ範囲内チェック
            if(0<=nx && nx<width_ && 0<=ny && ny<height_){
                int n_index = ny * width_ + nx;  // 新しいインデックス計算
                new_map_.data[n_index] = 100;          // 障害物としてマーク
            }
        }
    }
}


// ヒューリスティック関数の計算
double Astar::make_heuristic(const Node_ node)
{
    /* ユークリッド距離による推定コスト計算:
    √[(x_goal - x)^2 + (y_goal - y)^2] */
    return std::hypot(node.x - goal_node_.x, node.y - goal_node_.y);
}


// スタートとゴールの取得（mからグリッド単位への変換も行う）
Node_ Astar::set_way_point(int phase)
{
    /* Rviz座標（m）からグリッド座標へ変換:
    1. マップ原点を考慮した座標変換
    2. グリッドインデックス計算 */
    Node_ wp;
    wp.x = std::round((way_points_x_[phase] - origin_x_) / resolution_);
    wp.y = std::round((way_points_y_[phase] - origin_y_) / resolution_);
    return wp;
}


// ノードをたどり，waypoint間のパスを作成．その後グローバルパスに追加
// 参考：push_back(...) https://cpprefjp.github.io/reference/vector/vector/push_back.html
void Astar::create_path(Node_ node)
{
    /* パス生成フロー：
    1. ゴールノードから逆方向に親ノードを追跡
    2. 部分パスを構築
    3. 順序反転してグローバルパスに追加 */
    //部分パスを初期化

    nav_msgs::msg::Path partial_path;
   // partial_path.poses.clear();
    //partial_path.poses.push_back(node_to_pose(node));
    //// 現在のノード（最初はゴールノード）をパスに追加

    // ###### パスの作成 ######
    // スタート地点まで遡ってパス構築
    /*while(!check_start(node)){

        
        for(const Node_& n : close_list_){
            if(n.x == node.parent_x && n.y == node.parent_y){
                partial_path.poses.push_back(node_to_pose(n)); // パス点追加
                node = n;  // 親ノードに移動
                break;
            }
        }

    }*/

/*
    for(int i=close_list_.size()-1; i>=0; i--){
        if(!check_same_node(close_list_[i],start_node_)){
            if(check_parent(i-1,close_list_[i])){
                partial_path.poses.push_back(node_to_pose(close_list_[i-1]));
            }
        }
    }
*/

/*
    geometry_msgs::msg::PoseStamped pose = node_to_pose(node);
    partial_path.poses.push_back(pose);

    Node_ current = node;
    int count = 0;
    while (!check_start(current) && count < 10000) {
        bool found = false;
        for (int i = 0; i < close_list_.size(); ++i) {
            if (close_list_[i].x == current.parent_x && close_list_[i].y == current.parent_y) {
                geometry_msgs::msg::PoseStamped pose = node_to_pose(close_list_[i]);
                partial_path.poses.push_back(pose);
                current = close_list_[i];
                found = true;
                break;
            }
        }
        if (!found) {
            RCLCPP_WARN(get_logger(), "Parent not found!");
            break;
        }
        count++;
    }
*/
    partial_path.header.frame_id = "map";
    std::vector<Node_> path;
    Node_ current = node;
std::cout << "Path " << current.x << " " << current.y << std::endl;
    // 経路復元
    while (!check_start(current)) {
        bool found = false;
        // close_list_を逆順に探索 (重要)
        for (int i = close_list_.size() - 1; i >= 0; --i) {
            if (close_list_[i].x == current.parent_x && close_list_[i].y == current.parent_y) {
                path.push_back(close_list_[i]);
                current = close_list_[i];
                found = true;
                break;
            }
        }
        if (!found) {
            RCLCPP_WARN(get_logger(), "Parent not found!");
            return;
        }
    }

    // パス順序反転（スタート→ゴール）
    //std::reverse(partial_path.poses.begin(), partial_path.poses.end());
    // グローバルパスにマージ


    std::reverse(path.begin(), path.end()); // 順序を反転

    // PoseStamped 変換
    for (const auto& n : path) {
        geometry_msgs::msg::PoseStamped pose = node_to_pose(n);
        partial_path.poses.push_back(pose);
    }

    /*
    global_path_.poses.insert(global_path_.poses.end(),
                             partial_path.poses.begin(),
                             partial_path.poses.end());
    */

    global_path_.poses.push_back(node_to_pose(start_node_)); 

    /*for (const auto& pose : partial_path.poses) {
        global_path_.poses.push_back(pose);  // グローバルパスに追加
    }*/

    std::cout << "start point" << start_node_.x << " " << start_node_.y << std::endl;
    geometry_msgs::msg::PoseStamped start_pose = node_to_pose(start_node_);
    global_path_.poses.push_back(start_pose);



    // ###### パスの追加 ######

}


// ノード座標（グリッド）をgeometry_msgs::msg::PoseStamped（m単位のワールド座標系）に変換
geometry_msgs::msg::PoseStamped Astar::node_to_pose(const Node_ node)
{
    /* グリッド座標をRviz表示用の座標系に変換:
    1. グリッド中心座標計算
    2. マップ原点を加算 */

    geometry_msgs::msg::PoseStamped pose;
    // X座標変換：グリッド位置 → 実世界座標
    // (node.x + 0.5) → グリッドセルの中央位置を算出
    // resolution_ → 1セルあたりのメートル単位の大きさ
    // origin_x_ → マップの原点X座標（通常はマップ左下）
    pose.pose.position.x = origin_x_ + (node.x ) * resolution_;
    // Y座標変換：グリッド位置 → 実世界座標
    // (node.y + 0.5) → グリッドセルの中央位置を算出
    // origin_y_ → マップの原点Y座標
    pose.pose.position.y = origin_y_ + (node.y ) * resolution_;
    // 座標系の指定（通常はマップ座標系）
    pose.header.frame_id = "map";
    pose.header.stamp = clock_.now(); // タイムスタンプ追加
    return pose;
}


// openリスト内で最もf値が小さいノードを取得する関数
Node_ Astar::select_min_f()
{   
    if (open_list_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Open list is empty, cannot select minimum f-value");
        throw std::runtime_error("Open list is empty");
    }
    /* オープンリスト内で最小f値のノードを選択:
    1. std::min_elementで最小要素検索
    2. リストから削除して返却 */
    std::vector<Node_>::iterator min_it = std::min_element(open_list_.begin(), open_list_.end(),
        [](const Node_& a, const Node_& b){ return a.f < b.f; });
    Node_ min_node = *min_it;
    open_list_.erase(min_it); // 選択したノードをオープンリストから削除
    return min_node;
}


// スタートノードの場合，trueを返す
bool Astar::check_start(const Node_ node)
{
    /* スタートノード判定 */
    return node.x == start_node_.x && node.y == start_node_.y;
}


// ゴールノードの場合，trueを返す
bool Astar::check_goal(const Node_ node)
{
    /* ゴールノード判定 */
    return node.x == goal_node_.x && node.y == goal_node_.y;
}


// 2つが同じノードである場合，trueを返す
bool Astar::check_same_node(const Node_ n1, const Node_ n2)
{
    /* 同一ノード判定 */
    return n1.x == n2.x && n1.y == n2.y;
}


// 指定したリストに指定のノードが含まれるか検索
//（含まれる場合はインデックス番号を返し，含まれない場合は-1を返す）
int Astar::check_list(const Node_ target_node, std::vector<Node_>& set)
{
    /* リスト内ノード検索：
    1. ラムダ式で比較関数を定義
    2. std::find_ifで要素検索 */

    /*auto it = std::find_if(set.begin(), set.end(),
        [&](const Node_& n){ return check_same_node(n, target_node); });
    return (it != set.end()) ? std::distance(set.begin(), it) : -1;*/

    for (size_t i = 0; i < set.size(); ++i) {
        if (check_same_node(target_node, set[i])) {
            return i;
        }
    }
    return -1;
}


// list1から指定されたノードを探し，リスト1から削除してリスト2に移動する関数
void Astar::swap_node(const Node_ node, std::vector<Node_>& list1, std::vector<Node_>& list2)
{
    /* ノード移動処理：
    1. リスト1から要素検索
    2. リスト2に移動後、リスト1から削除 */
    auto it = std::find_if(list1.begin(), list1.end(),
        [&](const Node_& n){ return check_same_node(n, node); });
    if(it != list1.end()){
        list2.push_back(*it);
        list1.erase(it);
    }else {
        RCLCPP_WARN(get_logger(), "Node (%d, %d) not found in list", node.x, node.y);
    }
}


// 指定のノードが障害物である場合，trueを返す
// 障害物チェック
bool Astar::check_obs(const Node_ node)
{
    /* 障害物判定処理：
    1. グリッドインデックス計算
    2. マップ範囲チェック
    3. 障害物値(100)判定 */
    const int index = node.y * width_ + node.x;
    return (index >=0 && index < new_map_.data.size()) ? 
           (new_map_.data[index] == 100) : true; // 範囲外は障害物扱い
}


// 隣接ノードを基にOpenリスト・Closeリストを更新
// 隣接ノードを計算し，障害物を避けつつ，リスト内のノードを適切に追加・更新
// 複数の変数への代入はstd::tie(...)を使用すると便利 https://minus9d.hatenablog.com/entry/2015/05/24/133253
// リスト更新処理
void Astar::update_list(const Node_ node)
{
    /* 隣接ノード評価:
    1. 8方向の隣接ノード生成
    2. 各ノードの有効性チェック
    3. リスト状態に応じて更新 */

    // 隣接ノードを宣言
    std::vector<Node_> neighbor_nodes;
    create_neighbor_nodes(node, neighbor_nodes);

    // ###### 隣接ノード ######
    for(const Node_& neighbor : neighbor_nodes){
        // 障害物チェック
        if(check_obs(neighbor)) continue;

        /*int open_index = check_list(neighbor, open_list_);
        int close_index = check_list(neighbor, close_list_);

        if(open_index == -1 && close_index == -1){
            open_list_.push_back(neighbor); // 新規ノード追加
        }
        else if(open_index != -1 && neighbor.f < open_list_[open_index].f){
            open_list_[open_index] = neighbor; // コスト低い経路で更新
        }*/

        //auto [open_index, close_index] = search_node(neighbor);
        int open_index, close_index;
        std::tie(open_index, close_index) = search_node(neighbor);
        
        if (open_index == -1 && close_index == -1) {
            open_list_.push_back(neighbor);  // 新しいノードの場合、オープンリストに追加
        } else if (open_index != -1 && neighbor.f < open_list_[open_index].f) {
            open_list_[open_index] = neighbor;  // より良いパスが見つかった場合、更新
        } else if (close_index != -1 && neighbor.f < close_list_[close_index].f) {
        close_list_.erase(close_list_.begin() + close_index);
        open_list_.push_back(neighbor);
    }
    
    }
}


// 現在のノードを基に隣接ノードを作成
void Astar::create_neighbor_nodes(const Node_ node, std::vector<Node_>&  neighbor_nodes)
{
    /* 8方向隣接ノード生成：
    1. 動作モデル取得
    2. 各方向のノードを生成
    3. 有効範囲内のみ追加 */

    // 動作モデルの作成
    std::vector<Motion_> motion_list;
    get_motion(motion_list);

    // ###### 動作モデルの作成 ######

    /*for(const auto& motion : motion_list){
        Node_ neighbor;
        neighbor.x = node.x + motion.dx;
        neighbor.y = node.y + motion.dy;
        neighbor.cost = node.cost + motion.cost; // 累積コスト更新
        neighbor.f = neighbor.cost + make_heuristic(neighbor); // f値計算
        neighbor.parent_x = node.x; // 親ノード情報設定
        neighbor.parent_y = node.y;

        if(neighbor.x >=0 && neighbor.x < width_ && 
           neighbor.y >=0 && neighbor.y < height_){
            neighbor_nodes.push_back(neighbor); // 有効ノードのみ追加
        }
    }*/
   for (const Motion_& m : motion_list) {
        Node_ neighbor = get_neighbor_node(node, m);
        // マップ範囲内かチェック
        if (neighbor.x >= 0 && neighbor.x < width_ && 
            neighbor.y >= 0 && neighbor.y < height_) {
            neighbor_nodes.push_back(neighbor);
        }
        /*neighbor_nodes.push_back(get_neighbor_node(node, m));*/
         //neighbor_nodes.push_back(get_neighbor_node(node, m));
    }

    // ###### 隣接ノードの作成 ######

}


// 動作モデルを作成（前後左右，斜めの8方向）
void Astar::get_motion(std::vector<Motion_>& list)
{
    /* 8方向移動モデル：
    直進4方向＋斜め4方向
    コスト：直進=1, 斜め=√2 */
    list.push_back(motion( 1, 0, 1));   // 前
    list.push_back(motion(0, 1, 1));    // 右
    list.push_back(motion(-1, 0, 1));   // 後
    list.push_back(motion(0, -1, 1));   // 左
    list.push_back(motion(1, 1, std::sqrt(2)));  // 右前
    list.push_back(motion(1, -1, std::sqrt(2))); // 右後
    list.push_back(motion(-1, 1, std::sqrt(2))); // 左前
    list.push_back(motion(-1, -1, std::sqrt(2)));// 左後
    // ###### 上を参考に動作モデルの追加 ######
}


// 与えられたdx, dy, costを基にモーション（移動）を作成
// 隣接したグリッドに移動しない場合はエラーメッセージを出力して終了
Motion_ Astar::motion(const int dx,const int dy,const int cost)
{
    if(std::abs(dx) >1 || std::abs(dy) >1){
        RCLCPP_ERROR(get_logger(), "Invalid motion model");
        rclcpp::shutdown();
    }
    /*return Motion_{dx, dy, cost};*/
    return Motion_{dx, dy, static_cast<double>(cost)};
}

// 現在のノードと与えられたモーションを基に隣接ノードを計算し，その隣接ノードのf値と親ノードを更新して返す
Node_ Astar::get_neighbor_node(const Node_ node, const Motion_ motion)
{
    Node_ neighbor;
    neighbor.x = node.x + motion.dx;
    neighbor.y = node.y + motion.dy;
    //neighbor.cost = node.cost + motion.cost;
    //neighbor.f = neighbor.cost + make_heuristic(neighbor);
    neighbor.parent_x = node.x;
    neighbor.parent_y = node.y;
    // 累積コスト (g) を計算: 現在ノードのf値からヒューリスティック値を引く
    double g = node.f - make_heuristic(node);
    // 隣接ノードの総コスト (f) を計算: g + 移動コスト + ヒューリスティック値
    neighbor.f = g + motion.cost + make_heuristic(neighbor);
    return neighbor;
}

// 指定されたノードがOpenリストまたはCloseリストに含まれているかを調べ，結果をインデックスとともに返す
// 1はOpenリスト，2はCloseリストにノードが含まれていることを示す
// -1はどちらのリストにもノードが含まれていないことを示す
std::tuple<int, int> Astar::search_node(const Node_ node)
{
    /*int open_index = check_list(node, open_list_);
    if(open_index != -1) return {1, open_index};

    int close_index = check_list(node, close_list_);
    if(close_index != -1) return {2, close_index};

    return {-1, -1};*/

    int open_index = check_list(node, open_list_);
    int close_index = check_list(node, close_list_);
    //return std::make_tuple(open_index != -1 ? 1 : -1, close_index != -1 ? 2 : -1);
    return std::make_tuple(open_index, close_index);

}


// 親ノードかの確認
bool Astar::check_parent(const int index, const Node_ node)
{
    return close_list_[index].parent_x == node.x && close_list_[index].parent_y == node.y;
}



// 指定リスト内のノード検索
// 同一のノードが見つかればそのインデックスを返す
// 見つからなければ-1を返す
int Astar::search_node_from_list(const Node_ node, std::vector<Node_>& list)
{
    return check_list(node, list);
}


// ［デバック用］指定されたノードの位置をRvizに表示
// test_show_がtrueの場合，ノードの座標をワールド座標系に変換し
// そのノードの情報をRvizにパブリッシュ
void Astar::show_node_point(const Node_ node)
{
    /*if (test_show_) {
        geometry_msgs::msg::PoseStamped pose = node_to_pose(node);
        current_node_ = pose;
        current_node_pub_->publish(current_node_);
    }*/
   if (test_show_) {
        geometry_msgs::msg::PointStamped point;
        point.header.frame_id = "map";
        point.point.x = origin_x_ + (node.x ) * resolution_;
        point.point.y = origin_y_ + (node.y ) * resolution_;
        pub_node_point_->publish(point);
    }
}

// ［デバック用］指定されたパスをRvizに表示
// test_show_がtrueの場合，パスのフレームIDを"map"に設定し
// パス情報をRvizにパブリッシュ
void Astar::show_path(nav_msgs::msg::Path& current_path)
{
    if (test_show_) {

        current_path.header.frame_id = "map";
        pub_current_path_->publish(current_path);
        //global_path_.header.stamp = clock_.now();
        //pub_current_path_->publish(global_path_); // 全体パスを
    }
}

// 実行時間を表示（スタート時間beginを予め設定する）
void Astar::show_exe_time()
{
    RCLCPP_INFO_STREAM(get_logger(), "Duration = " << std::fixed << std::setprecision(2) << clock_.now().seconds() - begin_.seconds() << "s");
}



// 経路計画を行う関数
// 目的地までの経路をA*アルゴリズムを用いて計算し，グローバルパスを作成
// 各フェーズ（ウェイポイント間）について，OpenリストとCloseリストを操作しながら経路を探索
void Astar::planning()
{

    RCLCPP_INFO(get_logger(), "Checking waypoints...");
    for (int i = 0; i < way_points_x_.size(); ++i) {
        Node_ wp = set_way_point(i);
        RCLCPP_INFO(get_logger(), "Waypoint %d: x = %d, y = %d", i, wp.x, wp.y);
    }



    global_path_.poses.clear(); // 全体パスを初期化
    RCLCPP_INFO(get_logger(), "Starting global path planning...");
    begin_ = clock_.now();
    const int total_phase = way_points_x_.size();
    

    // ###### ウェイポイント間の経路探索 ######
    for (int phase = 0; phase < total_phase; phase++) {
        //global_path_.poses.clear();
        start_node_ = set_way_point(phase);
        goal_node_ = set_way_point(phase + 1);
        
        //start_node_.f = make_heuristic(start_node_);

        open_list_.clear();
        close_list_.clear();
        start_node_.f = make_heuristic(start_node_);
        open_list_.push_back(start_node_);
        
        while (rclcpp::ok()) {
            Node_ current_node = select_min_f();
            close_list_.push_back(current_node);
            
            if (check_goal(current_node)) {
                create_path(current_node);
                if (test_show_) {
                    //show_path(global_path_);
                }
                break;
            }
    
            update_list(current_node);

            /*if (test_show_) {
                nav_msgs::msg::Path current_path;
                Node_ temp_node = current_node;
                current_path.poses.push_back(node_to_pose(temp_node));
                int max_iter = width_ * height_;
                while (!check_start(temp_node) && max_iter-- > 0) {
                    bool found = false;
                    for (int i = close_list_.size() - 1; i >= 0; i--) {
                        if (close_list_[i].x == temp_node.parent_x && close_list_[i].y == temp_node.parent_y) {
                            current_path.poses.push_back(node_to_pose(close_list_[i]));
                            temp_node = close_list_[i];
                            found = true;
                            break;
                            }
                    }
                    if (!found) break;
                }
                std::reverse(current_path.poses.begin(), current_path.poses.end());
                show_path(current_path);
                rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time_ * 1000)));
            }*/

            /*if (test_show_) 
            {  // デバッグ表示モードの場合
                show_node_point(current_node);  // 現在のノードを表示
                nav_msgs::msg::Path current_path;
                //create_path(current_node);  // 現在のパスを生成
                show_path(current_path);  // 現在のパスを表示
                rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time_ * 1000)));  // 表示のための一時停止
            }*/
        }
        //pub_path_ ->publish(global_path_);
    }



/*
    // 最後のパスを表示するために、ここに追加
        start_node_ = set_way_point(total_phase-2);
        goal_node_ = set_way_point(total_phase-1);

        open_list_.clear();
        close_list_.clear();
        start_node_.f = make_heuristic(start_node_);
        open_list_.push_back(start_node_);

        while (rclcpp::ok()) {
            Node_ current_node = select_min_f();
            close_list_.push_back(current_node);

            if (check_goal(current_node)) {
                create_path(current_node);
                break;
            }

            update_list(current_node);
        }
        for(int i = 0; i < global_path_.poses.size(); i++){
         global_path_.poses[i].header.stamp = clock_.now();
         }
*/


         // パスのタイムスタンプを更新
        for(auto & pose : global_path_.poses) {
            pose.header.stamp = clock_.now();
        }


        if (test_show_) {
            show_path(global_path_);
        }





    pub_path_ ->publish(global_path_);

    show_exe_time();
    RCLCPP_INFO_STREAM(get_logger(), "COMPLITE ASTAR PROGLAM");
}


// map_callback()関数で実行する関数
// A*アルゴリズムを実行する前にマップのロードをチェック
// マップが読み込まれた後に壁判定と経路計画を実行
void Astar::process()
{
    RCLCPP_INFO_STREAM(get_logger(), "process is starting...");

    if(!map_checker_){
    RCLCPP_INFO_STREAM(get_logger(), "NOW LOADING...");
    }else
    {
        RCLCPP_INFO_STREAM(get_logger(), "NOW LOADED MAP");
        obs_expander(); // 壁の拡張
        planning(); // グローバルパスの作成
        exit(0);
    }
}
