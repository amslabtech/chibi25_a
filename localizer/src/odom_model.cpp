#include "localizer/odom_model.hpp"

// デフォルトコンストラクタ
OdomModel::OdomModel() {}

// コンストラクタ
OdomModel::OdomModel(const double ff, const double fr, const double rf, const double rr)
    : engine_(seed_gen_()), std_norm_dist_(0.0, 1.0), fw_dev_(0.0), rot_dev_(0.0)
{
    fw_var_per_fw_ = pow(ff, 2.0);
    fw_var_per_rot_ = pow(fr, 2.0);
    rot_var_per_fw_ = pow(rf, 2.0);
    rot_var_per_rot_ = pow(rr, 2.0);
}

// 代入演算子
OdomModel& OdomModel::operator =(const OdomModel& model)
{
    fw_var_per_fw_ = model.fw_var_per_fw_;
    fw_var_per_rot_ = model.fw_var_per_rot_;
    rot_var_per_fw_ = model.rot_var_per_fw_;
    rot_var_per_rot_ = model.rot_var_per_rot_;
    fw_dev_ = model.fw_dev_;
    rot_dev_ = model.rot_dev_;
    return *this;
}

// 並進，回転に関する標準偏差の設定
void OdomModel::set_dev(const double length, const double angle)
{
    fw_dev_ = sqrt(length*length*fw_var_per_fw_ + angle*angle*fw_var_per_rot_);
    rot_dev_ = sqrt(length*length*rot_var_per_fw_ + angle*angle*rot_var_per_rot_);
}

// 直進に関するノイズ（fw_dev_）の取得
double OdomModel::get_fw_noise()
{
    return fw_dev_ * std_norm_dist_(engine_);
}

// 回転に関するノイズ（rot_dev_）の取得
double OdomModel::get_rot_noise()
{
    return rot_dev_ * std_norm_dist_(engine_);
}
