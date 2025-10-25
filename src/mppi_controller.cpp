#include "mppi_planner/mppi_controller.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <chrono>
#include <omp.h>

namespace mppi_core
{
MPPICore::MPPICore(const MPPIParams& params) : generator_(random_seed_)
{
    setParams(params);
}

MPPICore::~MPPICore(){}

void MPPICore::setParams(const MPPIParams& params)
{
    K_ = params.K;                               // サンプル数
    T_ = params.T;                               // 時間ステップ数
    int UDIM = params.UDIM;                      // 制御入力の次元
    int XDIM = params.XDIM;                      // 状態の次元
    dt_ = params.dt;                             // 時間ステップ
    lambda_ = params.lambda;                     // 温度パラメータ
    alpha_ = params.alpha;                       // ノイズ減衰率
    exploration_rate_ = params.exploration_rate; // 探索率
    sigma_ = params.sigma;                       // ノイズ標準偏差
    max_linear_velocity_ = params.max_linear_velocity;   // 最大並進速度
    max_angular_velocity_ = params.max_angular_velocity; // 最大角速度

    // 状態、ノイズ、重み、コストの初期化
    State init_state(XDIM, 0.0);
    Control init_control(UDIM, 0.0);
    
    state_sample_ = StateSeqSamples(K_, StateSeq(T_, init_state));        
    u_samples_ = ControlSeqSamples(K_, ControlSeq(T_, init_control));
    u_opt_seq_ = ControlSeq(T_, init_control);
    u_opt_seq_latest_ = ControlSeq(T_, init_control);
    x_opt_seq_ = StateSeq(T_, init_state);
    costs_ = std::vector<double>(K_, 0.0);
    weights_ = std::vector<double>(K_, 0.0);
    noises_ = NoiseSamples(K_, NoiseSeq(T_, init_control));
    u_prev_ = init_control; // 前回制御入力を初期化

    // モデルの初期化
    model_ = std::make_shared<DiffDrive>(dt_);
}

void MPPICore::setCostMap(const nav_msgs::msg::OccupancyGrid& costmap)
{
    local_costmap_ = costmap;
    local_costmap_origin_x_ = costmap.info.origin.position.x;
    local_costmap_origin_y_ = costmap.info.origin.position.y;
    local_costmap_resolution_ = costmap.info.resolution;
    map_size_ = costmap.info.width; // assuming square map
    obstacle_grid_ = costmap.data;
}

VelocityCommand MPPICore::solveMPPI(
    const Pose2D& current_state,
    const Pose2D& target_point
){
    //　Cost Initialization
    costs_ = std::vector<double>(K_, 0.0);
    // generate noise
    noises_ = generateNoise();

    // 計測開始
    auto start = std::chrono::high_resolution_clock::now();
    
    //TODO 並列処理
    #pragma omp parallel for num_threads(omp_get_max_threads()) collapse(1) // 1階層並列化
    for (int k = 0; k < K_; k++)
    {
        state_sample_[k][0] = current_state;
        State state = current_state;

        for (int t = 1; t < T_ + 1; t++)
        {
            // 制御入力サンプル
            if (k < (1.0-exploration_rate_)*K_) {
                u_samples_[k][t-1] = u_opt_seq_latest_[t-1];
                for (int i = 0; i < 2; i++) { // UDIM = 2 for [linear_vel, angular_vel]
                    u_samples_[k][t-1][i] += noises_[k][t-1][i];
                }
            } else {
                u_samples_[k][t-1] = noises_[k][t-1];
            }

            // 状態更新
            state = model_->update_state(state,u_samples_[k][t-1]);
            state_sample_[k][t-1] = state;

            // コスト計算
            // 前回の制御入力を取得（t=1の場合は保存された前回値、それ以降はu_samples_[k][t-2]）
            Control u_previous = (t == 1) ? u_prev_ : u_samples_[k][t-2];
            costs_[k] += stage_cost(state, u_samples_[k][t-1], target_point, u_previous);
        }
        // 終端コスト計算
        costs_[k] += terminal_cost(state, target_point);
    }
    computeWeights();

    // 最適制御入力の更新（重み付け平均）
    for (int t = 0; t < T_; t++) {
        u_opt_seq_[t][0] = 0.0;
        u_opt_seq_[t][1] = 0.0;
        for (int k = 0; k < K_; k++) {
            u_opt_seq_[t][0] += weights_[k] * u_samples_[k][t][0];
            u_opt_seq_[t][1] += weights_[k] * u_samples_[k][t][1];
        }
        // clamp
        u_opt_seq_[t][0] = std::clamp(u_opt_seq_[t][0], -max_linear_velocity_, max_linear_velocity_);
        u_opt_seq_[t][1] = std::clamp(u_opt_seq_[t][1], -max_angular_velocity_, max_angular_velocity_);
    }

    u_opt_seq_latest_ = u_opt_seq_;
    
    // 前回制御入力を更新
    u_prev_[0] = u_opt_seq_[0][0];
    u_prev_[1] = u_opt_seq_[0][1];
    
    VelocityCommand cmd(2);
    cmd[0] = u_opt_seq_[0][0];  // linear_velocity
    cmd[1] = u_opt_seq_[0][1];  // angular_velocity
    
    // stop timer
    auto end_time = std::chrono::high_resolution_clock::now();  
    calc_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start).count();

    return cmd;
}

NoiseSamples MPPICore::generateNoise()
{
    for (int k = 0; k < K_; k++)
    {
        for (int t = 0; t < T_; t++)
        {
            std::normal_distribution<double> dist_x(0.0, sigma_[0]); 
            std::normal_distribution<double> dist_y(0.0, sigma_[1]); 
            noises_[k][t][0] = dist_x(generator_);
            noises_[k][t][1] = dist_y(generator_);
        }
    }
    return noises_;
}

void MPPICore::computeWeights(){
    // min_cost
    double min_cost = *std::min_element(costs_.begin(), costs_.end());

    double eta = 0.0;
    for (int k = 0; k < K_; k++)
    {
        eta += std::exp( (-1.0/ lambda_) * (costs_[k] - min_cost) );
    }
    
    // Calculate weights
    for (int k = 0; k < K_; k++)
    {
        weights_[k] = (1.0 / eta) * std::exp( (-1.0/ lambda_) * (costs_[k] - min_cost) );
    }
}

double MPPICore::stage_cost(const std::vector<double>& X, const std::vector<double>& U, const Pose2D& target_point, const std::vector<double>& U_prev)
{
    // 1.目的地までの距離コスト
    double dist_cost = 0.0;
    double dx = X[0] - target_point[0];
    double dy = X[1] - target_point[1];
    dist_cost = std::sqrt(dx * dx + dy * dy) ;

    // 2.障害物コスト
    int x_idx = static_cast<int>((X[0] - local_costmap_origin_x_) / local_costmap_resolution_);
    int y_idx = static_cast<int>((X[1] - local_costmap_origin_y_) / local_costmap_resolution_);

    double obstacle_cost = 0.0;
    // if (x_idx >= 0 && y_idx >= 0 && x_idx < map_size_ && y_idx < map_size_ && obstacle_grid_[y_idx * map_size_ + x_idx] > 0) {
    //     obstacle_cost = std::numeric_limits<double>::infinity();
    //     //TODO ポテンシャル場での計算に変更
    // }
    

    // 3.制御入力コスト（制御入力の大きさに対するペナルティ）
    // 速度を出すために重みを大幅に削減 (0.1 -> 0.01)
    double control_cost = 0.01 * (U[0] * U[0] + U[1] * U[1]);

    // 4.急な加速に対するコスト（前回制御入力との差分）
    double accel_linear = (U[0] - U_prev[0]) / dt_;   // 線形加速度
    double accel_angular = (U[1] - U_prev[1]) / dt_;  // 角加速度
    // 加速度ペナルティを削減して速度向上を許可 (1.0 -> 0.05)
    double acceleration_cost = 0.05 * (accel_linear * accel_linear + accel_angular * accel_angular);

    // 距離コストの重みを増やして目標への到達を優先 (係数10.0を追加)
    return 10.0 * dist_cost + obstacle_cost + control_cost + acceleration_cost;
}

double MPPICore::terminal_cost(const std::vector<double>& X, const Pose2D& target_point)
{
    double dist_cost = 0.0;
    double dx = X[0] - target_point[0];
    double dy = X[1] - target_point[1];
    dist_cost = std::sqrt(dx * dx + dy * dy) ;
    // 終端での距離コストも同様に重みを増やす
    return 10.0 * dist_cost;
}

double MPPICore::getStateCost() {
    return state_cost_;
}
double MPPICore::getControlCost() {
    return control_cost_;
}


} // namespace mppi_core
