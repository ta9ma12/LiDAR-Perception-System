/**
 * generate_cad_map.cpp
 *
 * Generates a high-density PCD point cloud of the Robocon 2026 competition field,
 * with all object positions and dimensions strictly derived from the official CAD drawings.
 *
 * Architecture:
 *   - cad2026::official: Direct raw values from official CAD drawings.
 *   - cad2026::calc: Mathematically derived object center coordinates.
 *   - cad2026::lidar_model: Approximate parameters for LiDAR simulation (hollow structures, legs, etc.).
 *   - cad2026::validation: Compile-time assertions checking formula correctness against exact expectations.
 *
 * Coordinate system:
 *   X: ±5700mm (Red zone = negative, Blue zone = positive)
 *   Y: -4700mm (手前/near) to +5800mm (奥/far) — NON-SYMMETRIC
 *   Z: 0mm = floor level
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>

using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;

namespace cad2026
{
// =========================================================================
// 1. OFFICIAL CAD RAW DATA (公式図面に明記された確定寸法のみ)
// =========================================================================
namespace official
{
    constexpr double FIELD_X_HALF  = 5.700;
    constexpr double FIELD_Y_FRONT = -4.700;
    constexpr double FIELD_Y_BACK  =  5.800;

    constexpr double PLATFORM_HALF_X  = 0.300;
    constexpr double PLATFORM_HEIGHT  = 0.200;
    constexpr double FENCE_THICK      = 0.150;
    constexpr double START_ZONE_DEPTH = 1.000;

    // カッティングシート寸法 (注記: *カッティングシートまでの距離)
    constexpr double CHAIR_SHEET_W = 0.420;
    constexpr double CHAIR_SHEET_D = 0.460;
    constexpr double FLAG_SHEET_W  = 0.450;
    constexpr double FLAG_SHEET_D  = 0.450;
    constexpr double BUCKET1_SHEET_DIAMETER = 0.340; // φ340明記
    constexpr double BUCKET23_SHEET_W = 0.360;
    constexpr double BUCKET23_SHEET_D = 0.360;
    constexpr double DESK_SHEET_W  = 0.510;
    constexpr double DESK_SHEET_D  = 0.710;

    constexpr double CONTROL_STATION_W = 3.400; // 平面図上部の寸法
    constexpr double CONTROL_STATION_D = 0.550;

    // 寸法チェーン (*シート端までの距離)
    constexpr double DIM_FENCE_TO_CHAIR  = 0.490;
    constexpr double DIM_CHAIR_TO_FLAG   = 1.500;
    constexpr double DIM_FLAG_TO_BUCKET1 = 1.760;

    constexpr double DIM_PLATFORM_TO_BUCKET = 1.000;
    constexpr double DIM_PLATFORM_TO_DESK   = 3.300;

    constexpr double DIM_FRONT_TO_CHAIR   = 1.430;
    constexpr double DIM_FRONT_TO_FLAG    = 1.415;
    constexpr double DIM_FRONT_TO_BUCKET1 = 1.470;
    constexpr double DIM_FRONT_TO_DESK2   = 1.500;
    constexpr double DIM_BACK_TO_DESK1    = 2.600;

    constexpr double DIM_START_TO_BUCKET2 = 3.800; // シート上側端まで

    // 競技用品寸法
    constexpr double CHAIR_PRODUCT_W  = 0.360;
    constexpr double CHAIR_PRODUCT_D  = 0.400;
    constexpr double CHAIR_PRODUCT_H  = 0.807;
    constexpr double CHAIR_PRODUCT_SH = 0.460;

    constexpr double DESK_PRODUCT_W = 0.650;
    constexpr double DESK_PRODUCT_D = 0.450;
    constexpr double DESK_PRODUCT_H = 0.760;

    constexpr double BUCKET_R = 0.273 / 2.0;
    constexpr double BUCKET_H = 0.255;
    constexpr double BUCKET2_BASE_W = 0.300;
    constexpr double BUCKET2_BASE_H = 0.600;
    constexpr double BUCKET3_BASE_H = 0.300;

    constexpr double FLAG_TOTAL_H = 3.000;
}

// =========================================================================
// 2. CALCULATED COORDINATES (公式寸法チェーンからの算術結果)
// =========================================================================
namespace calc
{
    constexpr double CHAIR_X_ABS = official::FIELD_X_HALF - (official::DIM_FENCE_TO_CHAIR + official::CHAIR_SHEET_W / 2.0);
    constexpr double CHAIR_Y     = official::FIELD_Y_FRONT + official::DIM_FRONT_TO_CHAIR + official::CHAIR_SHEET_D / 2.0;

    constexpr double FLAG_X_ABS  = official::FIELD_X_HALF - (official::DIM_FENCE_TO_CHAIR + official::CHAIR_SHEET_W + official::DIM_CHAIR_TO_FLAG + official::FLAG_SHEET_W / 2.0);
    constexpr double FLAG_Y      = official::FIELD_Y_FRONT + official::DIM_FRONT_TO_FLAG + official::FLAG_SHEET_D / 2.0;

    constexpr double BUCKET1_X_ABS = official::FIELD_X_HALF - (official::DIM_FENCE_TO_CHAIR + official::CHAIR_SHEET_W + official::DIM_CHAIR_TO_FLAG + official::FLAG_SHEET_W + official::DIM_FLAG_TO_BUCKET1 + official::BUCKET1_SHEET_DIAMETER / 2.0);
    constexpr double BUCKET1_Y     = official::FIELD_Y_FRONT + official::DIM_FRONT_TO_BUCKET1 + official::BUCKET1_SHEET_DIAMETER / 2.0;

    constexpr double DESK_X_ABS  = official::PLATFORM_HALF_X + official::DIM_PLATFORM_TO_DESK + official::DESK_SHEET_W / 2.0;
    constexpr double DESK1_Y     = official::FIELD_Y_BACK - official::DIM_BACK_TO_DESK1 - official::DESK_SHEET_D / 2.0;
    constexpr double DESK2_Y     = official::FIELD_Y_FRONT + official::DIM_FRONT_TO_DESK2 + official::DESK_SHEET_D / 2.0;

    constexpr double BUCKET2_X_ABS = official::PLATFORM_HALF_X + official::DIM_PLATFORM_TO_BUCKET + official::BUCKET23_SHEET_W / 2.0;
    constexpr double BUCKET2_Y     = (official::FIELD_Y_BACK - official::START_ZONE_DEPTH) - official::DIM_START_TO_BUCKET2 + official::BUCKET23_SHEET_D / 2.0;

    constexpr double BUCKET3_X_ABS = official::PLATFORM_HALF_X + official::DIM_PLATFORM_TO_BUCKET + official::BUCKET23_SHEET_W / 2.0;
    // BUCKET3_Y は公式図面から直接導出できないため calc では定義しない
}

// =========================================================================
// 3. LIDAR_MODEL (CADに直接存在しない近似値・LiDAR用推定モデル)
// =========================================================================
namespace lidar_model
{
    constexpr double DESK_LEG_WIDTH = 0.030;
    constexpr double DESK_TOP_THICKNESS = 0.020;
    constexpr double DESK_LEG_INSET = 0.010;

    constexpr double CHAIR_LEG_WIDTH = 0.025;
    constexpr double CHAIR_TOP_THICKNESS = 0.020;

    constexpr double FLAG_POLE_RADIUS = 0.020;
    constexpr double FLAG_BASE_THICKNESS = 0.050;

    // コントロールステーションは側面図省略につき、高さを推定値として扱う
    constexpr double CONTROL_STATION_H = 0.760;

    // バケツ③のY座標（PCD描写用の暫定近似値）
    constexpr double BUCKET3_Y_APPROX = -1.820;
}

// =========================================================================
// 4. VALIDATION (コンパイル時自動検証)
// =========================================================================
namespace validation
{
    constexpr double EPS = 1e-6;
    constexpr bool is_equal(double a, double b) { return (a > b ? a - b : b - a) < EPS; }

    static_assert(is_equal(calc::CHAIR_X_ABS, 5.000), "CHAIR_X_ABS mismatch");
    static_assert(is_equal(calc::CHAIR_Y, -3.040), "CHAIR_Y mismatch");

    static_assert(is_equal(calc::FLAG_X_ABS, 3.065), "FLAG_X_ABS mismatch");
    static_assert(is_equal(calc::FLAG_Y, -3.060), "FLAG_Y mismatch");

    static_assert(is_equal(calc::BUCKET1_X_ABS, 0.910), "BUCKET1_X_ABS mismatch");
    static_assert(is_equal(calc::BUCKET1_Y, -3.060), "BUCKET1_Y mismatch");

    static_assert(is_equal(calc::DESK_X_ABS, 3.855), "DESK_X_ABS mismatch");
    static_assert(is_equal(calc::DESK1_Y, 2.845), "DESK1_Y mismatch");
    static_assert(is_equal(calc::DESK2_Y, -2.845), "DESK2_Y mismatch");

    static_assert(is_equal(calc::BUCKET2_X_ABS, 1.480), "BUCKET2_X_ABS mismatch");
    static_assert(is_equal(calc::BUCKET2_Y, 1.180), "BUCKET2_Y mismatch");
    static_assert(is_equal(calc::BUCKET3_X_ABS, 1.480), "BUCKET3_X_ABS mismatch");
}

} // namespace cad2026

// Generate surface points for a box specified by min/max bounds
void addBoxBounds(Cloud& cloud, double x0, double x1,
                  double y0, double y1, double z0, double z1, double res = 0.02)
{
    for (double x = x0; x <= x1 + res * 0.4; x += res)
        for (double y = y0; y <= y1 + res * 0.4; y += res) {
            cloud.push_back(PointT(x, y, z0));
            cloud.push_back(PointT(x, y, z1));
        }
    for (double x = x0; x <= x1 + res * 0.4; x += res)
        for (double z = z0; z <= z1 + res * 0.4; z += res) {
            cloud.push_back(PointT(x, y0, z));
            cloud.push_back(PointT(x, y1, z));
        }
    for (double y = y0; y <= y1 + res * 0.4; y += res)
        for (double z = z0; z <= z1 + res * 0.4; z += res) {
            cloud.push_back(PointT(x0, y, z));
            cloud.push_back(PointT(x1, y, z));
        }
}

// Generate surface points for a 3D box (6 faces, hollow inside)
void addBox(Cloud& cloud, double cx, double cy, double cz,
            double sx, double sy, double sz, double res = 0.02)
{
    addBoxBounds(cloud, cx - sx/2.0, cx + sx/2.0, cy - sy/2.0, cy + sy/2.0, cz - sz/2.0, cz + sz/2.0, res);
}

// Generate a table/desk model with a hollow center (tabletop + 4 legs)
void addHollowTable(Cloud& cloud, double cx, double cy, double z_bottom,
                    double w, double d, double h, double top_t, double leg_w, double leg_inset, double res = 0.02)
{
    // Tabletop
    addBoxBounds(cloud, cx - w/2.0, cx + w/2.0, cy - d/2.0, cy + d/2.0, z_bottom + h - top_t, z_bottom + h, res);
    
    // Legs
    double lx_min = cx - w/2.0 + leg_inset;
    double lx_max = cx + w/2.0 - leg_inset - leg_w;
    double ly_min = cy - d/2.0 + leg_inset;
    double ly_max = cy + d/2.0 - leg_inset - leg_w;
    
    addBoxBounds(cloud, lx_min, lx_min + leg_w, ly_min, ly_min + leg_w, z_bottom, z_bottom + h - top_t, res);
    addBoxBounds(cloud, lx_min, lx_min + leg_w, ly_max, ly_max + leg_w, z_bottom, z_bottom + h - top_t, res);
    addBoxBounds(cloud, lx_max, lx_max + leg_w, ly_min, ly_min + leg_w, z_bottom, z_bottom + h - top_t, res);
    addBoxBounds(cloud, lx_max, lx_max + leg_w, ly_max, ly_max + leg_w, z_bottom, z_bottom + h - top_t, res);
}

// Generate a chair model with hollow legs, seat, and backrest
void addHollowChair(Cloud& cloud, double cx, double cy, double z_bottom,
                    double w, double d, double h_total, double h_seat, double top_t, double leg_w, double res = 0.02)
{
    // Seat
    addBoxBounds(cloud, cx - w/2.0, cx + w/2.0, cy - d/2.0, cy + d/2.0, z_bottom + h_seat - top_t, z_bottom + h_seat, res);
    
    // Backrest (approx)
    addBoxBounds(cloud, cx - w/2.0, cx + w/2.0, cy + d/2.0 - top_t, cy + d/2.0, z_bottom + h_seat, z_bottom + h_total, res);
    
    // 4 legs
    double lx_min = cx - w/2.0;
    double lx_max = cx + w/2.0 - leg_w;
    double ly_min = cy - d/2.0;
    double ly_max = cy + d/2.0 - leg_w;
    
    addBoxBounds(cloud, lx_min, lx_min + leg_w, ly_min, ly_min + leg_w, z_bottom, z_bottom + h_seat - top_t, res);
    addBoxBounds(cloud, lx_min, lx_min + leg_w, ly_max, ly_max + leg_w, z_bottom, z_bottom + h_seat - top_t, res);
    addBoxBounds(cloud, lx_max, lx_max + leg_w, ly_min, ly_min + leg_w, z_bottom, z_bottom + h_seat - top_t, res);
    addBoxBounds(cloud, lx_max, lx_max + leg_w, ly_max, ly_max + leg_w, z_bottom, z_bottom + h_seat - top_t, res);
}

// Generate surface points for a vertical cylinder
void addCylinder(Cloud& cloud, double cx, double cy, double cz,
                 double radius, double height, double res = 0.02)
{
    double z0 = cz - height / 2.0;
    double z1 = cz + height / 2.0;

    int n_ang = std::max(8, static_cast<int>(2.0 * M_PI * radius / res));
    for (double z = z0; z <= z1 + res * 0.4; z += res)
        for (int i = 0; i < n_ang; ++i) {
            double a = 2.0 * M_PI * i / n_ang;
            cloud.push_back(PointT(cx + radius * cos(a), cy + radius * sin(a), z));
        }

    int n_r = std::max(1, static_cast<int>(radius / res));
    for (int ri = 0; ri <= n_r; ++ri) {
        double r = radius * ri / n_r;
        int na = (r < 1e-6) ? 1 : std::max(1, static_cast<int>(2.0 * M_PI * r / res));
        for (int i = 0; i < na; ++i) {
            double a = 2.0 * M_PI * i / na;
            double x = cx + r * cos(a), y = cy + r * sin(a);
            cloud.push_back(PointT(x, y, z0));
            cloud.push_back(PointT(x, y, z1));
        }
    }
}

// Build control station (hollow model using official 3.4m x 0.55m dimensions)
void buildControlStation(Cloud& cloud, double sign, double res)
{
    using namespace cad2026::official;
    using namespace cad2026::lidar_model;

    double cx = sign * (FIELD_X_HALF + (FIELD_X_HALF - CONTROL_STATION_W)) / 2.0;
    double cy = FIELD_Y_BACK - CONTROL_STATION_D / 2.0;

    addHollowTable(cloud, cx, cy, 0.0,
                   CONTROL_STATION_W, CONTROL_STATION_D, CONTROL_STATION_H,
                   DESK_TOP_THICKNESS, DESK_LEG_WIDTH, DESK_LEG_INSET, res);
}

// Build all objects for one side of the field (Red or Blue)
void buildSide(Cloud& cloud, double sign, double res)
{
    using namespace cad2026::official;
    using namespace cad2026::calc;
    using namespace cad2026::lidar_model;

    // 手前行
    addHollowChair(cloud, sign * CHAIR_X_ABS, CHAIR_Y, 0.0,
                   CHAIR_PRODUCT_W, CHAIR_PRODUCT_D, CHAIR_PRODUCT_H, CHAIR_PRODUCT_SH, 
                   CHAIR_TOP_THICKNESS, CHAIR_LEG_WIDTH, res);

    addBox(cloud, sign * FLAG_X_ABS, FLAG_Y, FLAG_BASE_THICKNESS / 2.0,
           FLAG_SHEET_W, FLAG_SHEET_D, FLAG_BASE_THICKNESS, res);
    addCylinder(cloud, sign * FLAG_X_ABS, FLAG_Y, FLAG_BASE_THICKNESS + (FLAG_TOTAL_H - FLAG_BASE_THICKNESS) / 2.0,
                FLAG_POLE_RADIUS, FLAG_TOTAL_H - FLAG_BASE_THICKNESS, res);

    addCylinder(cloud, sign * BUCKET1_X_ABS, BUCKET1_Y, BUCKET_H / 2.0,
                BUCKET_R, BUCKET_H, res);

    // 上段行
    addHollowTable(cloud, sign * DESK_X_ABS, DESK1_Y, 0.0,
                   DESK_PRODUCT_W, DESK_PRODUCT_D, DESK_PRODUCT_H, 
                   DESK_TOP_THICKNESS, DESK_LEG_WIDTH, DESK_LEG_INSET, res);

    addHollowTable(cloud, sign * DESK_X_ABS, DESK2_Y, 0.0,
                   DESK_PRODUCT_W, DESK_PRODUCT_D, DESK_PRODUCT_H, 
                   DESK_TOP_THICKNESS, DESK_LEG_WIDTH, DESK_LEG_INSET, res);

    addBox(cloud, sign * BUCKET2_X_ABS, BUCKET2_Y, BUCKET2_BASE_H / 2.0,
           BUCKET2_BASE_W, BUCKET2_BASE_W, BUCKET2_BASE_H, res);
    addCylinder(cloud, sign * BUCKET2_X_ABS, BUCKET2_Y, BUCKET2_BASE_H + BUCKET_H / 2.0,
                BUCKET_R, BUCKET_H, res);

    // 固定バケツ③ (YはPCD視覚化用の暫定近似値 BUCKET3_Y_APPROX を使用)
    addBox(cloud, sign * BUCKET3_X_ABS, BUCKET3_Y_APPROX, BUCKET3_BASE_H / 2.0,
           BUCKET2_BASE_W, BUCKET2_BASE_W, BUCKET3_BASE_H, res);
    addCylinder(cloud, sign * BUCKET3_X_ABS, BUCKET3_Y_APPROX, BUCKET3_BASE_H + BUCKET_H / 2.0,
                BUCKET_R, BUCKET_H, res);

    // コントロールステーション
    buildControlStation(cloud, sign, res);
}

int main()
{
    using namespace cad2026::official;
    Cloud cloud;
    double res = 0.02;

    // Floor
    addBoxBounds(cloud, -FIELD_X_HALF, FIELD_X_HALF, FIELD_Y_FRONT, FIELD_Y_BACK, -0.01, 0.0, res);

    // Fences
    addBoxBounds(cloud, -FIELD_X_HALF - FENCE_THICK, FIELD_X_HALF + FENCE_THICK, FIELD_Y_BACK, FIELD_Y_BACK + FENCE_THICK, 0.0, FENCE_THICK, res);
    addBoxBounds(cloud, -FIELD_X_HALF - FENCE_THICK, FIELD_X_HALF + FENCE_THICK, FIELD_Y_FRONT - FENCE_THICK, FIELD_Y_FRONT, 0.0, FENCE_THICK, res);
    addBoxBounds(cloud, -FIELD_X_HALF - FENCE_THICK, -FIELD_X_HALF, FIELD_Y_FRONT, FIELD_Y_BACK, 0.0, FENCE_THICK, res);
    addBoxBounds(cloud, FIELD_X_HALF, FIELD_X_HALF + FENCE_THICK, FIELD_Y_FRONT, FIELD_Y_BACK, 0.0, FENCE_THICK, res);

    // Center Platform
    addBoxBounds(cloud, -PLATFORM_HALF_X, PLATFORM_HALF_X, FIELD_Y_FRONT, FIELD_Y_BACK, 0.0, PLATFORM_HEIGHT, res);

    // Build sides
    buildSide(cloud, -1.0, res);   // Red zone
    buildSide(cloud, +1.0, res);   // Blue zone

    // Save
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;

    std::string out = "/home/lambda/ros2_ws/src/LiDAR-Perception-System/maps/robocon2026_field.pcd";
    pcl::io::savePCDFileBinary(out, cloud);
    std::cout << "Saved " << cloud.points.size() << " points to " << out << std::endl;

    return 0;
}
