# LiDAR Perception System

ROS 2 HumbleとCUDAを使う、相手の移動バケツ専用のLiDAR認識パッケージです。入力はLivox Mid-360等の`PointCloud2`と時刻対応の`map → base_link → センサ` TFです。点群の変換と高さ別グリッド集計をGPUで行い、支持部の候補を追跡します。

固定バケツ・旗の認識、PCL、背景PCD、KD-tree背景差分、中間点群の配信は使用しません。既知の固定設備のTFがある場合は、その周辺を候補から除外します。

## ビルド

必要な直接依存: ROS 2 Humble、CUDAToolkit、Eigen3、nlohmann_json。Ceresは現実装では使用しません。CUDA GPUと互換ドライバが必要です。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lidar_perception_system --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=<対象GPUのSM番号>
source install/setup.bash
```

`CMAKE_CUDA_ARCHITECTURES`は対象機のGPUに合わせて指定してください。開発機のRTX 4060 Laptop GPUでは`89`を使用しました。Jetsonの型番を確認せずに`89`を流用しないでください。

## 実行

```bash
ros2 launch lidar_perception_system moving_bucket.launch.py
```

設定は[config/moving_bucket.json](config/moving_bucket.json)で指定します。別設定を使う場合:

```bash
ros2 launch lidar_perception_system moving_bucket.launch.py config_file:=/absolute/path/to/moving_bucket.json
```

bag再生時はbagの時刻とTFが一致するようにしてください。途中からの再生では、収録冒頭の`/tf_static`が飛ばされる場合があります。付属ベンチマークはbagの静的TFを再配信します。

## RViz2での確認

```bash
ros2 launch lidar_perception_system moving_bucket_rviz.launch.py
```

大会bagと一緒に使用する場合は`use_sim_time:=true`を付け、`ros2 bag play /path/to/rosbag_directory --clock`で再生します。固定フレームは設定の`target_frame`と同じ`map`にしてください。bagの途中から再生する場合は`/tf_static`も利用可能にしてください。

RViz2には元点群、目標位置の円柱、速度矢印、直近100観測の軌跡、状態ラベルが表示されます。緑はバケツ本体の直接観測、橙は支持部からの推定（Zは暫定値）、青は短時間の予測です。目標が無効になると位置マーカーを消し、赤い`NO TARGET`と理由を表示します。状態ラベルの`tf_drops`が増え続けるときは点群時刻に対応するTFを確認してください。TF表示は必要に応じてRViz側で有効にできます。

マーカーはRViz等の購読者がいるときだけ生成するため、通常運転時の追加負荷を抑えています。表示が途切れた場合も、点群とマーカーを一緒に見て「未検出」「TF不足」「ノード停止」を区別してください。マーカーの有無だけで認識精度を保証するものではありません。

## 出力

- `/moving_bucket_detector/target`: `lidar_perception_system/msg/MovingBucketTrack`。位置、速度、共分散、観測状態、validを含みます。
- `/moving_bucket_detector/diagnostics`: 入力件数、処理件数、TF待ち超過、キュー破棄、処理異常、処理時間のp95・最大値、TF待ち時間のp95。
- `map → moving_bucket` TF: 設定で有効化した場合、直接観測時のみ配信します。

`observation_mode=SUPPORT_INFERRED`はバケツ本体が直接見えていないことを表します。現在の`support_target_z=1.2 m`は大会bagによる暫定値であり、実物の支持部と開口中心の関係を実測するまで、Zを高精度な観測値として扱わないでください。`confidence`は校正された確率ではありません。`valid=false`の場合は古い位置を照準に使わないでください。

## 検証

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 scripts/benchmark_cuda.py /path/to/rosbag_directory --offset 0 --seconds 275 --output /tmp/lps_benchmark.json
```

比較対象の`/opponent_robot/bucket_target`は既存アルゴリズムの結果で、正解ラベルではありません。精度の合否には、映像や実測位置に基づく別の正解データが必要です。精度を評価するときは観測不能な区間とTF欠落区間を分けて集計してください。
