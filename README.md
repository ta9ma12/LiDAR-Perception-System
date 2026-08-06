# LiDAR Perception System (LPS)

高専ロボコン2026向けの 3D LiDAR 空間認識 ROS 2 パッケージです。  
Livox Mid-360 から取得した 3D 点群データ（Point Cloud）を基に、自律移動および自動照準（エイム）に必要なターゲット空間座標をリアルタイム（10Hz以上）で算出・ブロードキャストします。

---

## 概要

* **対象ハードウェア:** Livox Mid-360
* **開発環境:** Ubuntu 22.04 LTS / ROS 2 Humble
* **使用ライブラリ:** PCL (Point Cloud Library), `tf2_ros`, `tf2_sensor_msgs`

事前マップとROI（関心領域）クロップを用いて固定オブジェクト（固定バケツ・旗等）を検知するとともに、非反復スキャン対策のフレームバッファリングおよびKD-Tree背景差分法を用いて動的オブジェクト（相手ロボット・移動バケツ）を高速かつ高精度に抽出します。

---

## ディレクトリ構成

```text
LiDAR-Perception-System/
├── CMakeLists.txt                  # ビルド設定ファイル
├── package.xml                     # ROS 2 パッケージマニフェスト
├── config/
│   └── perception_params.yaml      # パラメータ設定ファイル
├── include/lidar_perception_system/
│   ├── static_obj_detector.hpp     # 静的オブジェクト認識ノード ヘッダー
│   └── dynamic_obj_detector.hpp    # 動的オブジェクト認識ノード ヘッダー
├── src/
│   ├── static_obj_detector.cpp     # ROIクロップ・固定物認識ノード
│   ├── dynamic_obj_detector.cpp    # 背景差分・動的物認識ノード（フォールバック機能付き）
│   └── dummy_cloud_publisher.cpp   # オフラインテスト用ダミー点群発行ノード
├── launch/
│   └── perception.launch.py        # 全ノード一括起動Launchスクリプト
├── maps/
│   └── README.md                   # PCDマップ格納ディレクトリ
└── rviz/
    └── perception_debug.rviz       # デバッグ用 RViz2 設定ファイル
```

---

## 主な機能

### 1. 静的オブジェクト認識 (`static_obj_detector`)
* `map` 座標系において、事前に設定された理論座標周辺の空間領域（ROI）のみを `pcl::CropBox` で抽出。
* `pcl::EuclideanClusterExtraction` によりクラスタリングを行い、重心を算出。
* 認識結果を `tf` (`fixed_target_fixed_bucket_1`, `fixed_target_flag` など) および RViz2 用 MarkerArray として配信。

### 2. 動的オブジェクト認識 (`dynamic_obj_detector`)
* **非反復スキャン対策:** Mid-360 の疎な点群密度を補うため、過去数フレーム（デフォルト: 3フレーム）の点群を各タイムスタンプの TF を用いて統合（リングバッファリング）。
* **背景差分法:** 静的マップ PCD が指定されている場合、KD-Tree による近傍検索で固定背景を高速除去。
* **移動バケツ追従:** 高さフィルタリング（Z=1.2m〜2.1m）で移動バケツ領域を抽出。
* **透明バケツ（ポリカバケツ）透過対策フォールバック:**
  レーザーが透過・乱反射してバケツ本体の点群が取得できない場合、自動的に低い高さ範囲（Z=0.1m〜1.0m）のロボット本体・台座クラスタを検知し、その重心から指定オフセット（例: +0.80m）を加算して `moving_bucket` 仮想座標を推定・ブロードキャスト。

### 3. オフライン開発用ダミーノード (`dummy_cloud_publisher`)
* 実機や rosbag がない環境でも、ダミーの LiDAR 点群と TF (`map` -> `base_link` -> `livox_frame`) を 10Hz で生成・配信してアルゴリズムを単体検証可能。

---

## パラメータ設定 (`config/perception_params.yaml`)

| パラメータ | デフォルト値 | 説明 |
| --- | --- | --- |
| `voxel_leaf_size` | `0.05` | VoxelGrid ダウンサンプリングのリーフサイズ (m) |
| `cluster_tolerance` | `0.15`〜`0.20` | クラスタリングの点間距離閾値 (m) |
| `min_cluster_size` | `15` | クラスタと認識する最小点数 |
| `frame_accumulation_count` | `3` | 動的オブジェクト検出時の統合フレーム数 |
| `enable_clear_bucket_fallback` | `true` | 透明バケツ用フォールバック処理の有効化フラグ |
| `fallback_virtual_z_offset` | `0.80` | 台座重心から仮想バケツ中心までの高さオフセット (m) |

---

## ビルド・実行手順

### 1. ビルド
ROS 2 ワークスペースの `src` ディレクトリ配下に配置し、`colcon build` を実行します。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lidar_perception_system
source install/setup.bash
```

### 2. 実行

#### 1. 本番環境（実機 / rosbag 連携時）
認識ノード群（`static_obj_detector` / `dynamic_obj_detector`）のみを起動します：

```bash
ros2 launch lidar_perception_system perception.launch.py
```
*(自己位置推定に ndt_localizer を用いる場合は `localization_type:=ndt` を指定)*

#### 2. オフライン動作テスト時（ダミー点群発行）
実機や rosbag がない環境で、ダミー点群・TFパブリッシャーを含めて起動します：

```bash
ros2 launch lidar_perception_system test_dummy.launch.py
```

#### 3. デバッグ用 RViz2 の起動
完全事前セッティング済みの RViz2 を別ターミナルで起動します：

```bash
ros2 launch lidar_perception_system rviz.launch.py
```

---

## ライセンス

[Apache-2.0](LICENSE)