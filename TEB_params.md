ご要望に応じて、`teb_local_planner`の**全設定パラメータ一覧**を、ソースに含まれるカテゴリ分けに従い、マークダウン形式で出力します。これらのパラメータは、`TebConfig`クラスに格納されており、ROSパラメータサーバーからロードされるか、動的リコンフィギュレーション（`rqt_reconfigure`）によって実行時に変更されます。

リストは、動的リコンフィギュレーションファイル（`TebLocalPlannerReconfigure.cfg`）および設定構造体（`teb_config.h`）の内容に基づいています。

## TebLocalPlannerの全設定パラメータ一覧

### 1. Trajectory (軌道関連) パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| :--- | :--- | :--- | :--- |
| **`teb_autosize`** | `bool` | `True` | 最適化中に軌道のサイズを自動的にリサイズするかどうかを有効にする。 |
| **`dt_ref`** | `double` | `0.3` | 計画された軌道の望ましい時間分解能（通常は制御レートの大きさにあわせて設定）。 |
| **`dt_hysteresis`** | `double` | `0.1` | 現在の時間分解能に基づいて自動リサイズに使用されるヒステリシス（通常は `dt_ref` の10%）。 |
| **`min_samples`** | `int` | `3` | 軌道に存在するべき最小のサンプル数（3以上である必要がある）。 |
| **`max_samples`** | `int` | `500` | 最大サンプル数。 |
| **`global_plan_overwrite_orientation`** | `bool` | `True` | グローバルプランによって提供されるローカルなサブゴールの向きを、自動的に決定された向きで上書きするかどうか。 |
| **`allow_init_with_backwards_motion`** | `bool` | `False` | ローカルコストマップ内で目標が開始点の背後にある場合に、後退動作で軌道を初期化することを許可するか（後方センサーを備えている場合のみ推奨）。 |
| **`global_plan_viapoint_sep`** | `double` | `-0.1` | グローバルプランから抽出される連続する2つの経由点間の最小分離距離（負の場合は無効）。 (注: 以前は`global_plan_via_point_sep`という名前であったが、一貫性のために置き換えられた)。 |
| **`via_points_ordered`** | `bool` | `False` | 経由点の順序を遵守するか。 |
| **`max_global_plan_lookahead_dist`** | `double` | `3.0` | 最適化で考慮されるグローバルプランのサブセットの最大長さ（累積ユークリッド距離）。 |
| **`global_plan_prune_distance`** | `double` | `1.0` | 刈り込み（pruning）に使用されるロボットとグローバルプランの経由点との間の距離。 |
| **`exact_arc_length`** | `bool` | `False` | 速度、加速度、回転率の計算で正確な円弧長を使用するか（CPU時間が増加する）。 |
| **`force_reinit_new_goal_dist`** | `double` | `1.0` | 以前のゴールが指定値（メートル単位）以上離れて更新された場合に、軌道の再初期化を強制するか（ホットスタートをスキップする）。 |
| **`force_reinit_new_goal_angular`** | `double` | `0.78` | 以前のゴールが指定値（ラジアン単位）以上回転差をもって更新された場合に、軌道の再初期化を強制するか（ホットスタートをスキップする）。 |
| **`feasibility_check_no_poses`** | `int` | `5` | 各サンプリング間隔で実行可能性がチェックされる、予測されたプラン上のポーズのインデックス上限。 |
| **`feasibility_check_lookahead_distance`** | `double` | `-1.0` | 各サンプリング間隔で実行可能性がチェックされる、ロボットからの距離上限。 |
| **`publish_feedback`** | `bool` | `False` | 評価やデバッグ目的で、完全な軌道とアクティブな障害物リストを含むフィードバックをパブリッシュするか。 |
| **`min_resolution_collision_check_angular`** | `double` | `M_PI` | コストマップの衝突チェック中に使用される最小角度分解能。 |
| **`control_look_ahead_poses`** | `int` | `1` | 速度コマンドを抽出するために使用されるポーズのインデックス。 |
| **`prevent_look_ahead_poses_near_goal`** | `int` | `0` | 目標付近でのオーバーシュートや振動を防ぐために、`control_look_ahead_poses`が目標からこの数だけポーズを遡って参照しないようにする。 |
| **`visualize_with_time_as_z_axis_scale`** | `double` | `0.0` | 0より大きい場合、軌道と障害物を時間軸をZ軸スケールとして使用して3Dで可視化する。動的障害物に最も有用です。 |

### 2. Robot (ロボット制約) パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| :--- | :--- | :--- | :--- |
| **`max_vel_x`** | `double` | `0.4` | ロボットのX方向の最大速度。 |
| **`max_vel_x_backwards`** | `double` | `0.2` | 後退時のロボットの最大並進速度。 |
| **`max_vel_y`** | `double` | `0.0` | ロボットの最大横移動速度（非ホロノミックロボットの場合はゼロであるべき）。 |
| **`max_vel_trans`** | `double` | `0.0` | ホロノミックロボットの最大線形速度。デフォルトの0.0の場合、`max_vel_x`と同じに設定される。 |
| **`max_vel_theta`** | `double` | `0.3` | ロボットの最大角速度。 |
| **`acc_lim_x`** | `double` | `0.5` | ロボットの最大並進加速度。 |
| **`acc_lim_y`** | `double` | `0.5` | ロボットの最大横移動加速度。 |
| **`acc_lim_theta`** | `double` | `0.5` | ロボットの最大角加速度。 |
| **`is_footprint_dynamic`** | `bool` | `False` | 軌道の実行可能性をチェックする前にフットプリントを更新するかどうか。 |
| **`use_proportional_saturation`** | `bool` | `False` | いずれかの成分が対応する境界を超えた場合、個別に飽和させる代わりに、すべてのツイスト成分（線形x/y、角z）を比例的に減少させるかどうか。 |
| **`transform_tolerance`** | `double` | `0.5` | TFツリーに変換を問い合わせる際の許容時間（秒）。 |
| **`min_turning_radius`** | `double` | `0.0` | カーライクロボットの最小旋回半径（差動駆動ロボットはゼロ）。 |
| **`wheelbase`** | `double` | `1.0` | 駆動軸と操舵軸の間の距離（カーライクロボットでのみ必要）。 |
| **`cmd_angle_instead_rotvel`** | `bool` | `False` | コマンド速度メッセージの回転速度を対応する操舵角に置き換えるかどうか。 |

### 3. GoalTolerance (目標許容誤差) パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| :--- | :--- | :--- | :--- |
| **`xy_goal_tolerance`** | `double` | `0.2` | 目標位置に対する最終的なユークリッド距離の許容誤差。 |
| **`yaw_goal_tolerance`** | `double` | `0.1` | 目標の向きに対する最終的な向きの誤差の許容誤差。 |
| **`free_goal_vel`** | `bool` | `False` | 計画目的でロボットの速度が非ゼロであることを許可するか（ロボットが最大速度で目標に到達可能）。 |
| **`trans_stopped_vel`** | `double` | `0.1` | ロボットが並進で停止したと見なされる最大速度。 |
| **`theta_stopped_vel`** | `double` | `0.1` | ロボットが回転で停止したと見なされる最大回転速度。 |
| **`complete_global_plan`** | `bool` | `True` | ロボットが最終目標を横切ったときにパスを早く終了することを防ぐかどうか。 |

### 4. Obstacles (障害物関連) パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| :--- | :--- | :--- | :--- |
| **`min_obstacle_dist`** | `double` | `0.5` | 障害物との最小希望分離距離。 |
| **`inflation_dist`** | `double` | `0.6` | 非ゼロのペナルティコストを持つ障害物周辺のバッファゾーン（`min_obstacle_dist`より大きいべき）。 |
| **`dynamic_obstacle_inflation_dist`** | `double` | `0.6` | 動的障害物の予測位置周辺の非ゼロのペナルティコストを持つバッファゾーン。 |
| **`include_dynamic_obstacles`** | `bool` | `True` | 動的障害物の動きを一定速度モデルで予測に含めるか（ホモトピー計画にも影響する）。Falseの場合、すべての障害物は静的と見なされる。 |
| **`include_costmap_obstacles`** | `bool` | `True` | コストマップ内の障害物を直接考慮するかどうか。 |
| **`costmap_obstacles_behind_robot_dist`** | `double` | `1.5` | ロボットの後ろで計画に考慮されるローカルコストマップの障害物を制限する距離（メートル単位）。 |
| **`obstacle_poses_affected`** | `int` | `30` | 軌道上の最も近いポーズに障害物位置を関連付け、近隣のポーズも考慮に入れる数。 |
| **`legacy_obstacle_association`** | `bool` | `False` | 旧式の障害物関連付け戦略を使用するか。 |
| **`obstacle_association_force_inclusion_factor`** | `double` | `1.5` | 非レガシーな関連付け技術で、指定された距離内（`min_obstacle_dist`の倍数）のすべての障害物を強制的に含める係数。 |
| **`obstacle_association_cutoff_factor`** | `double` | `5.0` | `min_obstacle_dist`の[値]倍を超えたすべての障害物は最適化中に無視される（`force_inclusion_factor`が先に処理される）。 |
| **`costmap_converter_plugin`** | `string` | `""` | コストマップセルをポリゴンなどに変換する`costmap_converter`パッケージのプラグイン名。 |
| **`costmap_converter_spin_thread`** | `bool` | `True` | コストマップコンバーターがコールバックキューを別のスレッドで呼び出すか。 |
| **`costmap_converter_rate`** | `int` | `5` | コストマップコンバータープラグインが現在のコストマップを処理する頻度を定義するレート。 |
| **`obstacle_proximity_ratio_max_vel`** | `double` | `1.0` | 静的障害物への近接度によって速度を制限する際に使用される最大速度の比率。 |
| **`obstacle_proximity_lower_bound`** | `double` | `0.0` | 速度を下げるべき静的障害物までの距離。 |
| **`obstacle_proximity_upper_bound`** | `double` | `0.5` | 速度を上げるべき静的障害物までの距離。 |

### 5. Optimization (最適化重みと設定) パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| :--- | :--- | :--- | :--- |
| **`no_inner_iterations`** | `int` | `5` | 各アウターループ反復で呼び出されるソルバーの反復回数。 |
| **`no_outer_iterations`** | `int` | `4` | 軌道の自動リサイズと内部ソルバーループの呼び出しが行われるアウターループの反復回数。 |
| **`optimization_activate`** | `bool` | `True` | 最適化を有効にするか。 |
| **`optimization_verbose`** | `bool` | `False` | 詳細情報を出力するか。 |
| **`penalty_epsilon`** | `double` | `0.05` | ハード制約の近似のためのペナルティ関数に追加する小さな安全マージン。 |
| **`weight_max_vel_x`** | `double` | `2.0` | 許容される最大並進速度（X方向）を満たすための最適化重み。 |
| **`weight_max_vel_y`** | `double` | `2.0` | 許容される最大横移動速度（Y方向）を満たすための最適化重み（ホロノミックロボットでのみ使用）。 |
| **`weight_max_vel_theta`** | `double` | `1.0` | 許容される最大角速度を満たすための最適化重み。 |
| **`weight_acc_lim_x`** | `double` | `1.0` | 許容される最大並進加速度（X方向）を満たすための最適化重み。 |
| **`weight_acc_lim_y`** | `double` | `1.0` | 許容される最大横移動加速度（Y方向）を満たすための最適化重み（ホロノミックロボットでのみ使用）。 |
| **`weight_acc_lim_theta`** | `double` | `1.0` | 許容される最大角加速度を満たすための最適化重み。 |
| **`weight_kinematics_nh`** | `double` | `1000.0` | 非ホロノミックな運動学的制約を満たすための最適化重み。 |
| **`weight_kinematics_forward_drive`** | `double` | `1.0` | ロボットに前進方向のみを選択させるための最適化重み（差動駆動ロボットのみ）。 |
| **`weight_kinematics_turning_radius`** | `double` | `1.0` | 最小旋回半径を強制するための最適化重み（カーライクロボット）。 |
| **`weight_optimaltime`** | `double` | `1.0` | 遷移時間に関して軌道を収縮させるための最適化重み。 |
| **`weight_shortest_path`** | `double` | `0.0` | 経路長に関して軌道を収縮させるための最適化重み（最短経路エッジ）。 |
| **`weight_obstacle`** | `double` | `50.0` | 障害物からの最小分離距離を満たすための最適化重み。 |
| **`weight_inflation`** | `double` | `0.1` | インフレーションペナルティ（バッファゾーン）に対する最適化重み（小さい値であるべき）。 |
| **`weight_dynamic_obstacle`** | `double` | `50.0` | 動的障害物からの最小分離距離を満たすための最適化重み。 |
| **`weight_dynamic_obstacle_inflation`** | `double` | `0.1` | 動的障害物のインフレーションペナルティに対する最適化重み（小さい値であるべき）。 |
| **`weight_velocity_obstacle_ratio`** | `double` | `0.0` | 静的障害物までの距離に対する最大許容速度を満たすための最適化重み。 |
| **`weight_viapoint`** | `double` | `1.0` | 経由点への距離を最小化するための最適化重み。 |
| **`weight_prefer_rotdir`** | `double` | `50.0` | 特定の回転方向を優先するための最適化重み（振動が検出された場合にのみ現在アクティブになる）。 |
| **`weight_adapt_factor`** | `double` | `2.0` | 特定の重み（現在は`weight_obstacle`）を各外部TEB反復で繰り返しスケーリングする係数。 |
| **`obstacle_cost_exponent`** | `double` | `1.0` | 非線形障害物コストの指数（1に設定すると非線形コストは無効になる）。 |

### 6. Homotopy Class Planner (HCP) パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| :--- | :--- | :--- | :--- |
| **`enable_homotopy_class_planning`** | `bool` | `True` | ホモトピー経路計画を有効にするか。 |
| **`enable_multithreading`** | `bool` | `True` | 複数の軌道を並行して計画するためにマルチスレッドを有効にするか。 |
| **`simple_exploration`** | `bool` | `False` | 経路生成のために単純な左-右アプローチを使用するか。 |
| **`max_number_classes`** | `int` | `5` | 許可される代替ホモトピー等価クラスの最大数。 |
| **`max_number_plans_in_current_class`** | `int` | `1` | 現在の最良の軌道と同じホモトピー等価クラス内で試行する軌道の最大数。 |
| **`selection_cost_hysteresis`** | `double` | `1.0` | 新しい候補が以前に選択された軌道に対して選択されるために必要なコスト（`new_cost < old_cost * factor`の場合に選択）。 |
| **`selection_prefer_initial_plan`** | `double` | `0.95` | 初期プランの等価クラスにある軌道に対するコスト削減係数（0から1の間）。 |
| **`selection_obst_cost_scale`** | `double` | `2.0` | 「最良」候補を選択するためだけの障害物コスト項目の追加スケーリング。 |
| **`selection_viapoint_cost_scale`** | `double` | `1.0` | 「最良」候補を選択するためだけの経由点コスト項目の追加スケーリング。 |
| **`selection_alternative_time_cost`** | `bool` | `False` | Trueの場合、時間コストが総遷移時間に置き換えられる。 |
| **`selection_dropping_probability`** | `double` | `0.0` | 各計画サイクルで、現在の「最良」以外のTEBがランダムに破棄される確率。 |
| **`switching_blocking_period`** | `double` | `0.0` | 新しい等価クラスへの切り替えが許可されるまでに経過する必要がある時間（秒単位）。 |
| **`roadmap_graph_no_samples`** | `int` | `15` | `simple_exploration`がオフの場合に、ロードマップグラフを作成するために生成されるサンプル数。 |
| **`roadmap_graph_area_width`** | `double` | `5.0` | 開始点と目標点の間でサンプルが生成される領域の幅（メートル単位）。 |
| **`roadmap_graph_area_length_scale`** | `double` | `1.0` | 矩形領域の長さをスケーリングするパラメータ。 |
| **`h_signature_prescaler`** | `double` | `1.0` | 膨大な数の障害物を考慮に入れるために、障害物値をスケーリングする。 |
| **`h_signature_threshold`** | `double` | `0.1` | 2つのHシグネチャが等しいと見なされるしきい値。 |
| **`obstacle_keypoint_offset`** | `double` | `0.1` | `simple_exploration`がオンの場合、障害物の左右に新しいキーポイントが作成される距離。 |
| **`obstacle_heading_threshold`** | `double` | `0.45` | 障害物を探索に考慮に入れるための、障害物ヘディングと目標ヘディング間の正規化されたスカラー積の値。 |
| **`viapoints_all_candidates`** | `bool` | `True` | 異なるトポロジーのすべての軌道を経由点に付加するかどうか。 |
| **`visualize_hc_graph`** | `bool` | `False` | 新しいホモトピー等価クラスを探索するために作成されるグラフを可視化するか。 |
| **`delete_detours_backwards`** | `bool` | `True` | ベストプランに対して後方に遠回りするプランを破棄するかどうか。 |
| **`detours_orientation_tolerance`** | `double` | `0.785` (M_PI/2) | プランの開始向きがベストプランとこれ以上異なると遠回りと見なされる許容誤差。 |
| **`length_start_orientation_vector`** | `double` | `0.4` | プランの開始向きを計算するために使用されるベクトルの長さ。 |
| **`max_ratio_detours_duration_best_duration`** | `double` | `3.0` | 遠回りするプランの実行時間とベストTEBの実行時間の比率がこれより大きい場合、プランが破棄される。 |

### 7. Recovery (復旧とバックアップ) パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| :--- | :--- | :--- | :--- |
| **`shrink_horizon_backup`** | `bool` | `True` | 自動検出された問題が発生した場合に、一時的にホライズンを縮小することを許可するか。 |
| **`shrink_horizon_min_duration`** | `double` | `10.0` | 実行不可能な軌道が検出された場合の、縮小されたホライズンの最小期間。 |
| **`oscillation_recovery`** | `bool` | `True` | 複数の解の間で振動を検出し、解決を試みるか。 |
| **`oscillation_v_eps`** | `double` | `0.1` | 平均正規化線形速度のしきい値。これと`oscillation_omega_eps`の両方が超えられない場合、振動が検出される。 |
| **`oscillation_omega_eps`** | `double` | `0.1` | 平均正規化角速度のしきい値。 |
| **`oscillation_recovery_min_duration`** | `double` | `10.0` | 振動が検出された後に復旧モードがアクティブになる最小期間（秒）。 |
| **`oscillation_filter_duration`** | `double` | `10.0` | 振動検出のためのフィルターの長さ/期間（秒）。 |
| **`divergence_detection_enable`** | `bool` | `False` | 発散検出を有効にするか。 |
| **`divergence_detection_max_chi_squared`** | `int` | `10` | 最適化が発散したと見なされる許容可能な最大マハラノビス距離（カイ二乗）。 |
