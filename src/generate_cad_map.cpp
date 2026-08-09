/**
 * generate_cad_map.cpp
 *
 * Generates a high-density PCD point cloud of the Robocon 2026 competition field,
 * with all object positions and dimensions strictly matching the official CAD floorplan.
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
// 1. OFFICIAL CAD RAW DATA (公式図面に明記された確定寸法)
// =========================================================================
namespace official
{
    constexpr double FIELD_X_HALF  = 5.700;
    constexpr double FIELD_Y_FRONT = -4.700;
    constexpr double FIELD_Y_BACK  =  5.800;

    constexpr double PLATFORM_HALF_X = 0.300;
    constexpr double PLATFORM_HEIGHT = 0.200;
    constexpr double FENCE_THICK     = 0.150;

    // 平面図上の回転考慮後の競技用品サイズ [m]
    constexpr double CHAIR_PRODUCT_X  = 0.400;
    constexpr double CHAIR_PRODUCT_Y  = 0.360;
    constexpr double CHAIR_PRODUCT_H  = 0.807;
    constexpr double CHAIR_PRODUCT_SH = 0.460;

    constexpr double DESK_PRODUCT_X = 0.450;
    constexpr double DESK_PRODUCT_Y = 0.650;
    constexpr double DESK_PRODUCT_H = 0.760;

    constexpr double BUCKET_R = 0.273 / 2.0;
    constexpr double BUCKET_H = 0.255;
    constexpr double BUCKET2_BASE_W = 0.300;
    constexpr double BUCKET2_BASE_H = 0.600;
    constexpr double BUCKET3_BASE_H = 0.300;

    // 旗寸法
    constexpr double FLAG_BASE_X  = 0.450;
    constexpr double FLAG_BASE_Y  = 0.450;
    constexpr double FLAG_WIDTH   = 0.600;
    constexpr double FLAG_HEIGHT  = 1.800;
    constexpr double FLAG_TOTAL_H = 3.000;
    constexpr double FLAG_BOTTOM_Z = FLAG_TOTAL_H - FLAG_HEIGHT; // 1.200m
}

// =========================================================================
// 2. CALCULATED COORDINATES (公式平面図からの確定中心座標)
// =========================================================================
namespace calc
{
    // 手前中央水平列 (Y = 0.000)
    constexpr double CHAIR_X_ABS   = 4.980;
    constexpr double CHAIR_Y       = 0.000;

    constexpr double FLAG_X_ABS    = 3.025;
    constexpr double FLAG_Y        = 0.000;

    constexpr double BUCKET1_X_ABS = 0.870;
    constexpr double BUCKET1_Y     = 0.000;

    // 机① (奥), 机② (手前)
    constexpr double DESK_X_ABS = 3.855;
    constexpr double DESK1_Y    = 2.845;
    constexpr double DESK2_Y    = -2.845;

    // 固定バケツ② (高台座), 固定バケツ③ (低台座)
    constexpr double BUCKET2_X_ABS = 1.480;
    constexpr double BUCKET2_Y     = 1.820;

    constexpr double BUCKET3_X_ABS = 1.480;
    constexpr double BUCKET3_Y     = -1.820;

    // 奥壁沿い (コントロールステーション & 補充スポット)
    constexpr double CONTROL_X_ABS = 5.475;
    constexpr double CONTROL_Y     = 5.475;

    constexpr double REFILL_X_ABS  = 1.075;
    constexpr double REFILL_Y      = 5.475;
}

// =========================================================================
// 3. LIDAR_MODEL (LiDAR静的マップ用の近似・構造モデル)
// =========================================================================
namespace lidar_model
{
    constexpr double DESK_LEG_WIDTH = 0.030;
    constexpr double DESK_TOP_THICKNESS = 0.020;
    constexpr double DESK_LEG_INSET = 0.010;

    constexpr double CHAIR_LEG_WIDTH = 0.025;
    constexpr double CHAIR_TOP_THICKNESS = 0.020;

    constexpr double FLAG_POLE_RADIUS = 0.020;
    constexpr double FLAG_CLOTH_THICKNESS = 0.020;
    constexpr double FLAG_BASE_THICKNESS = 0.050;
}

// =========================================================================
// 4. VALIDATION (コンパイル時自動検証)
// =========================================================================
namespace validation
{
    constexpr double EPS = 1e-6;
    constexpr bool is_equal(double a, double b) { return (a > b ? a - b : b - a) < EPS; }

    static_assert(is_equal(calc::CHAIR_X_ABS, 4.980), "CHAIR_X_ABS mismatch");
    static_assert(is_equal(calc::CHAIR_Y, 0.000), "CHAIR_Y mismatch");

    static_assert(is_equal(calc::FLAG_X_ABS, 3.025), "FLAG_X_ABS mismatch");
    static_assert(is_equal(calc::FLAG_Y, 0.000), "FLAG_Y mismatch");

    static_assert(is_equal(calc::BUCKET1_X_ABS, 0.870), "BUCKET1_X_ABS mismatch");
    static_assert(is_equal(calc::BUCKET1_Y, 0.000), "BUCKET1_Y mismatch");

    static_assert(is_equal(calc::DESK_X_ABS, 3.855), "DESK_X_ABS mismatch");
    static_assert(is_equal(calc::DESK1_Y, 2.845), "DESK1_Y mismatch");
    static_assert(is_equal(calc::DESK2_Y, -2.845), "DESK2_Y mismatch");

    static_assert(is_equal(calc::BUCKET2_X_ABS, 1.480), "BUCKET2_X_ABS mismatch");
    static_assert(is_equal(calc::BUCKET2_Y, 1.820), "BUCKET2_Y mismatch");

    static_assert(is_equal(calc::BUCKET3_X_ABS, 1.480), "BUCKET3_X_ABS mismatch");
    static_assert(is_equal(calc::BUCKET3_Y, -1.820), "BUCKET3_Y mismatch");

    static_assert(is_equal(calc::CONTROL_X_ABS, 5.475), "CONTROL_X_ABS mismatch");
    static_assert(is_equal(calc::CONTROL_Y, 5.475), "CONTROL_Y mismatch");

    static_assert(is_equal(calc::REFILL_X_ABS, 1.075), "REFILL_X_ABS mismatch");
    static_assert(is_equal(calc::REFILL_Y, 5.475), "REFILL_Y mismatch");
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

// Generate a chair model with hollow legs, seat, and backrest facing outer wall (Red = -X, Blue = +X)
void addHollowChair(Cloud& cloud, double cx, double cy, double z_bottom,
                    double w, double d, double h_total, double h_seat,
                    double top_t, double leg_w, double sign, double res = 0.02)
{
    // Seat
    addBoxBounds(cloud, cx - w/2.0, cx + w/2.0, cy - d/2.0, cy + d/2.0, z_bottom + h_seat - top_t, z_bottom + h_seat, res);
    
    // Backrest (outer wall facing: Red = -X, Blue = +X)
    double bx0 = (sign < 0.0) ? (cx - w/2.0) : (cx + w/2.0 - top_t);
    double bx1 = (sign < 0.0) ? (cx - w/2.0 + top_t) : (cx + w/2.0);
    addBoxBounds(cloud, bx0, bx1, cy - d/2.0, cy + d/2.0, z_bottom + h_seat, z_bottom + h_total, res);
    
    // 4 legs
    double lx_min = cx - w/2.0, lx_max = cx + w/2.0 - leg_w;
    double ly_min = cy - d/2.0, ly_max = cy + d/2.0 - leg_w;
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

// Generate complete flag assembly (Base plate + Pole + 600x1800 Flag cloth)
void addFlag(Cloud& cloud, double cx, double cy, double res = 0.02)
{
    using namespace cad2026::official;
    using namespace cad2026::lidar_model;

    // 1. 土台 (450 x 450 x 50mm)
    addBox(cloud, cx, cy, FLAG_BASE_THICKNESS / 2.0, FLAG_BASE_X, FLAG_BASE_Y, FLAG_BASE_THICKNESS, res);

    // 2. 支柱 (φ40mm, Z = 0.05m ~ 3.00m)
    addCylinder(cloud, cx, cy, FLAG_BASE_THICKNESS + (FLAG_TOTAL_H - FLAG_BASE_THICKNESS) / 2.0,
                FLAG_POLE_RADIUS, FLAG_TOTAL_H - FLAG_BASE_THICKNESS, res);

    // 3. 旗布 (600x1800mm, Z = 1.20m ~ 3.00m)
    addBox(cloud, cx, cy, FLAG_BOTTOM_Z + FLAG_HEIGHT / 2.0,
           FLAG_CLOTH_THICKNESS, FLAG_WIDTH, FLAG_HEIGHT, res);
}

// Build all objects for one side of the field (Red or Blue)
void buildSide(Cloud& cloud, double sign, double res)
{
    using namespace cad2026::official;
    using namespace cad2026::calc;
    using namespace cad2026::lidar_model;

    // 手前中央水平列 (Y = 0.000)
    addHollowChair(cloud, sign * CHAIR_X_ABS, CHAIR_Y, 0.0,
                   CHAIR_PRODUCT_X, CHAIR_PRODUCT_Y, CHAIR_PRODUCT_H, CHAIR_PRODUCT_SH, 
                   CHAIR_TOP_THICKNESS, CHAIR_LEG_WIDTH, sign, res);

    addFlag(cloud, sign * FLAG_X_ABS, FLAG_Y, res);

    addCylinder(cloud, sign * BUCKET1_X_ABS, BUCKET1_Y, BUCKET_H / 2.0,
                BUCKET_R, BUCKET_H, res);

    // 机① (奥), 机② (手前)
    addHollowTable(cloud, sign * DESK_X_ABS, DESK1_Y, 0.0,
                   DESK_PRODUCT_X, DESK_PRODUCT_Y, DESK_PRODUCT_H, 
                   DESK_TOP_THICKNESS, DESK_LEG_WIDTH, DESK_LEG_INSET, res);

    addHollowTable(cloud, sign * DESK_X_ABS, DESK2_Y, 0.0,
                   DESK_PRODUCT_X, DESK_PRODUCT_Y, DESK_PRODUCT_H, 
                   DESK_TOP_THICKNESS, DESK_LEG_WIDTH, DESK_LEG_INSET, res);

    // 固定バケツ② (高台座), 固定バケツ③ (低台座)
    addBox(cloud, sign * BUCKET2_X_ABS, BUCKET2_Y, BUCKET2_BASE_H / 2.0,
           BUCKET2_BASE_W, BUCKET2_BASE_W, BUCKET2_BASE_H, res);
    addCylinder(cloud, sign * BUCKET2_X_ABS, BUCKET2_Y, BUCKET2_BASE_H + BUCKET_H / 2.0,
                BUCKET_R, BUCKET_H, res);

    addBox(cloud, sign * BUCKET3_X_ABS, BUCKET3_Y, BUCKET3_BASE_H / 2.0,
           BUCKET2_BASE_W, BUCKET2_BASE_W, BUCKET3_BASE_H, res);
    addCylinder(cloud, sign * BUCKET3_X_ABS, BUCKET3_Y, BUCKET3_BASE_H + BUCKET_H / 2.0,
                BUCKET_R, BUCKET_H, res);

    // 奥壁沿い：コントロールステーション
    addHollowTable(cloud, sign * CONTROL_X_ABS, CONTROL_Y, 0.0,
                   DESK_PRODUCT_X, DESK_PRODUCT_Y, DESK_PRODUCT_H,
                   DESK_TOP_THICKNESS, DESK_LEG_WIDTH, DESK_LEG_INSET, res);

    // 奥壁沿い：補充スポット
    addHollowTable(cloud, sign * REFILL_X_ABS, REFILL_Y, 0.0,
                   DESK_PRODUCT_X, DESK_PRODUCT_Y, DESK_PRODUCT_H,
                   DESK_TOP_THICKNESS, DESK_LEG_WIDTH, DESK_LEG_INSET, res);
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
