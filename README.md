| パラメータ名                         | 型            | 単位              | デフォルト値                    | 説明                     |
| ------------------------------ | ------------ | --------------- | ------------------------- | ---------------------- |
| `delta_t`                      | float        | s               | 0.05                      | サンプリング周期               | 
| `wheel_base`                   | float        | m               | 2.5                       | ホイールベース                |
| `vehicle_width`                | float        | m               | 3.0                       | 車両幅                    |
| `control_frequency_` | double | hz | | 実行周期 |
| `max_angle_` | double | | | |
| `max_velocity_` | double | | | |
| `max_accel_abs`                | float        | m/s²            | 2.0                       | 最大加速度                  |
| `horizon_step_`               | int          | step            | 30                        | 予測ホライゾン長               |
| `number_of_samples_`          | int          | sample          | 1000                      | サンプル数                  |
| `lambda`                 | float        | -               | 50.0                      | 情報理論的重みのスケーリング         |
| `alpha`                  | float        | -               | 1.0                       | 更新率                    |
| `sigma`                        | 2×2 ndarray  | -               | \[\[0.5,0],\[0,0.1]]      | ノイズ分散共分散行列             |
| `stage_cost_weight`            | ndarray(4)   | -               | \[50,50,1,20]             | ステージコスト重み \[x,y,yaw,v] |
| `terminal_cost_weight`         | ndarray(4)   | -               | \[50,50,1,20]             | 終端コスト重み \[x,y,yaw,v]   |
| `collision_safety_margin_rate` | float        | -               | 1.2                       | 車両形状拡張係数               |
