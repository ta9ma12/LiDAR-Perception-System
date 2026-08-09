/**
 * generate_cad_map.cpp
 *
 * Generates a high-density PCD point cloud of the Robocon 2026 competition field,
 * with all object positions and dimensions strictly derived from the official CAD drawings.
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

// =========================================================================
// 1. OFFICIAL CAD RAW DATA (公式図面に明記された確定寸法)
// =========================================================================
namespace official
{
    constexpr double FIELD_X_INNER = 5.700;
    constexpr double FIELD_Y_FRONT = -4.700;
    constexpr double FIELD_Y_BACK  =  5.800;

    constexpr double PLATFORM_HALF_X = 0.300;
    constexpr double PLATFORM_HEIGHT = 0.200;
    constexpr double FENCE_THICK     = 0.150;

    constexpr double CHAIR_SHEET_W = 0.420;
    constexpr double CHAIR_SHEET_D = 0.460;

    constexpr double FLAG_SHEET_W = 0.450;
    constexpr double FLAG_SHEET_D = 0.450;

    constexpr double BUCKET1_SHEET_D = 0.340; // φ340
    constexpr double BUCKET23_SHEET_W = 0.360;
    constexpr double BUCKET23_SHEET_D = 0.360;

    constexpr double DESK_SHEET_W = 0.510;
    constexpr double DESK_SHEET_D = 0.710;

    constexpr double DIM_FENCE_TO_CHAIR = 0.490;
    constexpr double DIM_CHAIR_TO_FLAG = 1.500;
    constexpr double DIM_FLAG_TO_BUCKET1 = 1.760;

    constexpr double DIM_FRONT_TO_CHAIR = 1.430;
    constexpr double DIM_FRONT_TO_FLAG = 1.415;
    constexpr double DIM_FRONT_TO_BUCKET1 = 1.470;

    constexpr double DIM_PLATFORM_TO_BUCKET23 = 1.000;
    constexpr double DIM_PLATFORM_TO_DESK = 3.300;

    constexpr double DIM_BACK_TO_DESK1 = 2.600;
    constexpr double DIM_FRONT_TO_DESK2 = 1.500;

    constexpr double DIM_START_TO_BUCKET2 = 3.800;
    constexpr double DIM_FRONT_TO_BUCKET3 = 2.700;

    constexpr double START_ZONE_DEPTH = 1.000;
    
    // 競技用品寸法
    constexpr double CHAIR_PRODUCT_W = 0.360;
    constexpr double CHAIR_PRODUCT_D = 0.400;
    constexpr double CHAIR_PRODUCT_H = 0.807;
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
    constexpr double CHAIR_X_ABS =
        official::FIELD_X_INNER
        - official::DIM_FENCE_TO_CHAIR
        - official::CHAIR_SHEET_W / 2.0; // 5.000

    constexpr double FLAG_X_ABS =
        official::FIELD_X_INNER
        - official::DIM_FENCE_TO_CHAIR
        - official::CHAIR_SHEET_W
        - official::DIM_CHAIR_TO_FLAG
        - official::FLAG_SHEET_W / 2.0; // 3.065

    constexpr double BUCKET1_X_ABS =
        official::FIELD_X_INNER
        - official::DIM_FENCE_TO_CHAIR
        - official::CHAIR_SHEET_W
        - official::DIM_CHAIR_TO_FLAG
        - official::FLAG_SHEET_W
        - official::DIM_FLAG_TO_BUCKET1
        - official::BUCKET1_SHEET_D / 2.0; // 0.910

    constexpr double BUCKET23_X_ABS =
        official::PLATFORM_HALF_X
        + official::DIM_PLATFORM_TO_BUCKET23
        + official::BUCKET23_SHEET_W / 2.0; // 1.480

    constexpr double DESK_X_ABS =
        official::PLATFORM_HALF_X
        + official::DIM_PLATFORM_TO_DESK
        + official::DESK_SHEET_W / 2.0; // 3.855

    constexpr double CHAIR_Y =
        official::FIELD_Y_FRONT
        + official::DIM_FRONT_TO_CHAIR
        + official::CHAIR_SHEET_D / 2.0; // -3.040

    constexpr double FLAG_Y =
        official::FIELD_Y_FRONT
        + official::DIM_FRONT_TO_FLAG
        + official::FLAG_SHEET_D / 2.0; // -3.060

    constexpr double BUCKET1_Y =
        official::FIELD_Y_FRONT
        + official::DIM_FRONT_TO_BUCKET1
        + official::BUCKET1_SHEET_D / 2.0; // -3.060

    constexpr double DESK1_Y =
        official::FIELD_Y_BACK
        - official::DIM_BACK_TO_DESK1
        - official::DESK_SHEET_D / 2.0; // +2.845

    constexpr double DESK2_Y =
        official::FIELD_Y_FRONT
        + official::DIM_FRONT_TO_DESK2
        + official::DESK_SHEET_D / 2.0; // -2.845

    constexpr double BUCKET2_Y =
        official::FIELD_Y_BACK
        - official::START_ZONE_DEPTH
        - official::DIM_START_TO_BUCKET2
        - official::BUCKET23_SHEET_D / 2.0; // +0.820

    constexpr double BUCKET3_Y =
        official::FIELD_Y_FRONT
        + official::DIM_FRONT_TO_BUCKET3
        + official::BUCKET23_SHEET_D / 2.0; // -1.820
}

// =========================================================================
// 3. LIDAR APPROXIMATION MODELS (LiDAR静的マップ用の近似・推測モデル)
// =========================================================================
namespace lidar_model
{
    // 公式図面にない脚の太さなどの「推定値」はここに隔離
    constexpr double DESK_LEG_WIDTH = 0.030;
    constexpr double DESK_TOP_THICKNESS = 0.020;
    constexpr double DESK_LEG_INSET = 0.010;

    constexpr double CHAIR_LEG_WIDTH = 0.025;
    constexpr double CHAIR_TOP_THICKNESS = 0.020;
    
    constexpr double FLAG_POLE_RADIUS = 0.020;
    constexpr double FLAG_BASE_THICKNESS = 0.050;
}

// Generate surface points for a box specified by min/max bounds
void addBoxBounds(Cloud& cloud, double x0, double x1,
                  double y0, double y1, double z0, double z1, double res = 0.02)
{
    // Top & bottom faces (Z = z0, z1)
    for (double x = x0; x <= x1 + res * 0.4; x += res)
        for (double y = y0; y <= y1 + res * 0.4; y += res) {
            cloud.push_back(PointT(x, y, z0));
            cloud.push_back(PointT(x, y, z1));
        }
    // Front & back faces (Y = y0, y1)
    for (double x = x0; x <= x1 + res * 0.4; x += res)
        for (double z = z0; z <= z1 + res * 0.4; z += res) {
            cloud.push_back(PointT(x, y0, z));
            cloud.push_back(PointT(x, y1, z));
        }
    // Left & right faces (X = x0, x1)
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
    
    // 4 legs
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

    // Side surface
    int n_ang = std::max(8, static_cast<int>(2.0 * M_PI * radius / res));
    for (double z = z0; z <= z1 + res * 0.4; z += res)
        for (int i = 0; i < n_ang; ++i) {
            double a = 2.0 * M_PI * i / n_ang;
            cloud.push_back(PointT(cx + radius * cos(a), cy + radius * sin(a), z));
        }

    // Top & bottom caps
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

// Build all objects for one side of the field (Red or Blue)
// sign: -1.0 = Red (left/X<0), +1.0 = Blue (right/X>0)
void buildSide(Cloud& cloud, double sign, double res)
{
    using namespace official;
    using namespace calc;
    using namespace lidar_model;

    // ========================================
    // 手前行 (Bottom row)
    // ========================================

    // 椅子 (Chair)
    addHollowChair(cloud, sign * CHAIR_X_ABS, CHAIR_Y, 0.0,
                   CHAIR_PRODUCT_W, CHAIR_PRODUCT_D, CHAIR_PRODUCT_H, CHAIR_PRODUCT_SH, 
                   CHAIR_TOP_THICKNESS, CHAIR_LEG_WIDTH, res);

    // 旗 (Field flag)
    addBox(cloud, sign * FLAG_X_ABS, FLAG_Y, FLAG_BASE_THICKNESS / 2.0,
           FLAG_SHEET_W, FLAG_SHEET_D, FLAG_BASE_THICKNESS, res); // 台座近似
    addCylinder(cloud, sign * FLAG_X_ABS, FLAG_Y, FLAG_BASE_THICKNESS + (FLAG_TOTAL_H - FLAG_BASE_THICKNESS) / 2.0,
                FLAG_POLE_RADIUS, FLAG_TOTAL_H - FLAG_BASE_THICKNESS, res); // ポール

    // 固定バケツ① (Fixed bucket 1)
    addCylinder(cloud, sign * BUCKET1_X_ABS, BUCKET1_Y, BUCKET_H / 2.0,
                BUCKET_R, BUCKET_H, res);

    // ========================================
    // 上段行 (Upper row)
    // ========================================

    // 机① (Desk 1, 奥側/upper)
    addHollowTable(cloud, sign * DESK_X_ABS, DESK1_Y, 0.0,
                   DESK_PRODUCT_W, DESK_PRODUCT_D, DESK_PRODUCT_H, 
                   DESK_TOP_THICKNESS, DESK_LEG_WIDTH, DESK_LEG_INSET, res);

    // 机② (Desk 2, 手前側/lower)
    addHollowTable(cloud, sign * DESK_X_ABS, DESK2_Y, 0.0,
                   DESK_PRODUCT_W, DESK_PRODUCT_D, DESK_PRODUCT_H, 
                   DESK_TOP_THICKNESS, DESK_LEG_WIDTH, DESK_LEG_INSET, res);

    // 固定バケツ② (Fixed bucket 2, 高台座H600)
    addBox(cloud, sign * BUCKET23_X_ABS, BUCKET2_Y, BUCKET2_BASE_H / 2.0,
           BUCKET2_BASE_W, BUCKET2_BASE_W, BUCKET2_BASE_H, res); // 台座
    addCylinder(cloud, sign * BUCKET23_X_ABS, BUCKET2_Y, BUCKET2_BASE_H + BUCKET_H / 2.0,
                BUCKET_R, BUCKET_H, res);                        // バケツ

    // 固定バケツ③ (Fixed bucket 3, 低台座H300)
    addBox(cloud, sign * BUCKET23_X_ABS, BUCKET3_Y, BUCKET3_BASE_H / 2.0,
           BUCKET2_BASE_W, BUCKET2_BASE_W, BUCKET3_BASE_H, res); // 台座
    addCylinder(cloud, sign * BUCKET23_X_ABS, BUCKET3_Y, BUCKET3_BASE_H + BUCKET_H / 2.0,
                BUCKET_R, BUCKET_H, res);                        // バケツ

    // ========================================
    // コントロールステーション (Control station)
    // ========================================
    double cs_x0 = std::min(sign * 2.300, sign * 5.700);
    double cs_x1 = std::max(sign * 2.300, sign * 5.700);
    addBoxBounds(cloud, cs_x0, cs_x1, 3.000, 5.800, 0.0, 0.760, res);
}

int main()
{
    using namespace official;
    Cloud cloud;
    double res = 0.02;  // 20mm resolution

    // ================================================================
    // Field floor
    // ================================================================
    addBoxBounds(cloud, -FIELD_X_INNER, FIELD_X_INNER, FIELD_Y_FRONT, FIELD_Y_BACK, -0.01, 0.0, res);

    // ================================================================
    // Outer fences
    // ================================================================
    addBoxBounds(cloud, -FIELD_X_INNER - FENCE_THICK, FIELD_X_INNER + FENCE_THICK, FIELD_Y_BACK, FIELD_Y_BACK + FENCE_THICK, 0.0, FENCE_THICK, res); // Top
    addBoxBounds(cloud, -FIELD_X_INNER - FENCE_THICK, FIELD_X_INNER + FENCE_THICK, FIELD_Y_FRONT - FENCE_THICK, FIELD_Y_FRONT, 0.0, FENCE_THICK, res); // Bottom
    addBoxBounds(cloud, -FIELD_X_INNER - FENCE_THICK, -FIELD_X_INNER, FIELD_Y_FRONT, FIELD_Y_BACK, 0.0, FENCE_THICK, res); // Left
    addBoxBounds(cloud, FIELD_X_INNER, FIELD_X_INNER + FENCE_THICK, FIELD_Y_FRONT, FIELD_Y_BACK, 0.0, FENCE_THICK, res); // Right

    // ================================================================
    // Center platform
    // ================================================================
    addBoxBounds(cloud, -PLATFORM_HALF_X, PLATFORM_HALF_X, FIELD_Y_FRONT, FIELD_Y_BACK, 0.0, PLATFORM_HEIGHT, res);

    // ================================================================
    // Build both sides (symmetric in X)
    // ================================================================
    buildSide(cloud, -1.0, res);   // Red zone (X < 0)
    buildSide(cloud, +1.0, res);   // Blue zone (X > 0)

    // ================================================================
    // Save
    // ================================================================
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;

    std::string out = "/home/lambda/ros2_ws/src/LiDAR-Perception-System/maps/robocon2026_field.pcd";
    pcl::io::savePCDFileBinary(out, cloud);
    std::cout << "Saved " << cloud.points.size() << " points to " << out << std::endl;

    return 0;
}
