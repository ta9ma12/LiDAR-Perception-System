/**
 * generate_cad_map.cpp
 *
 * Generates a high-density PCD point cloud of the Robocon 2026 competition field,
 * with all object positions and dimensions precisely matching the official CAD drawings.
 *
 * Coordinate system:
 *   X: ±5700mm (Red zone = negative, Blue zone = positive)
 *   Y: -4700mm (手前/near) to +5800mm (奥/far) — NON-SYMMETRIC
 *   Z: 0mm = floor level
 *
 * Object positions are specified using cutting-sheet center (カッティングシート芯) distances.
 *
 * Usage:
 *   ros2 run lidar_perception_system generate_cad_map
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>

using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;

// Generate surface points for a 3D box (6 faces, hollow inside)
void addBox(Cloud& cloud, double cx, double cy, double cz,
            double sx, double sy, double sz, double res = 0.02)
{
    double x0 = cx - sx / 2.0, x1 = cx + sx / 2.0;
    double y0 = cy - sy / 2.0, y1 = cy + sy / 2.0;
    double z0 = cz - sz / 2.0, z1 = cz + sz / 2.0;

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

// Generate surface points for a box specified by min/max bounds
void addBoxBounds(Cloud& cloud, double x0, double x1,
                  double y0, double y1, double z0, double z1, double res = 0.02)
{
    addBox(cloud,
           (x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0,
           x1 - x0, y1 - y0, z1 - z0, res);
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
    // ========================================
    // 手前行 (Bottom row) — from CAD dimension chain 490+1500+1760
    // X measured from left fence inner edge (-5700) using cutting sheet center distances
    // Y measured from bottom fence inner edge (-4700) using side-view annotations
    // ========================================

    // 椅子 (Chair): X = -5700+490 = -5210, Y = -4700+1430 = -3270
    // Product: W360×D400×H807
    addBox(cloud, sign * 5.210, -3.270, 0.807 / 2.0,
           0.360, 0.400, 0.807, res);

    // 旗 (Field flag): X = -5700+1990 = -3710, Y = -4700+1415 = -3285
    // Base cutting sheet: W450×D450, pole: φ40×H3000
    addBox(cloud, sign * 3.710, -3.285, 0.025,
           0.450, 0.450, 0.05, res);                       // 台座
    addCylinder(cloud, sign * 3.710, -3.285, 1.525,
                0.020, 2.95, res);                          // ポール

    // 固定バケツ① (Fixed bucket 1, floor-level): X=-5700+3750=-1950, Y=-4700+1470=-3230
    // Bucket: φ273×H255
    addCylinder(cloud, sign * 1.950, -3.230, 0.255 / 2.0,
                0.273 / 2.0, 0.255, res);

    // ========================================
    // 上段行 (Upper row) — interpretation A
    // X: 教壇端基準 +1000mm / +3300mm
    // Y: 側面図 1800mm→Y=-2.900, +3000mm→Y=+0.100
    // ========================================

    // 机① (Desk 1, 奥側/upper): X=-(300+3300)=-3600, Y=+0.100
    // Product: W650×D450×H760
    addBox(cloud, sign * 3.600, 0.100, 0.760 / 2.0,
           0.650, 0.450, 0.760, res);

    // 机② (Desk 2, 手前側/lower): X=-3600, Y=-2.900
    addBox(cloud, sign * 3.600, -2.900, 0.760 / 2.0,
           0.650, 0.450, 0.760, res);

    // 固定バケツ② (Fixed bucket 2, 高台座H600): X=-(300+1000)=-1300, Y=+0.100
    // Stand: W300×D300×H600, Bucket: φ273×H255
    addBox(cloud, sign * 1.300, 0.100, 0.300,
           0.300, 0.300, 0.600, res);                       // 台座
    addCylinder(cloud, sign * 1.300, 0.100, 0.600 + 0.255 / 2.0,
                0.273 / 2.0, 0.255, res);                   // バケツ

    // 固定バケツ③ (Fixed bucket 3, 低台座H300): X=-1300, Y=-2.900
    // Stand: W300×D300×H300, Bucket: φ273×H255
    addBox(cloud, sign * 1.300, -2.900, 0.150,
           0.300, 0.300, 0.300, res);                       // 台座
    addCylinder(cloud, sign * 1.300, -2.900, 0.300 + 0.255 / 2.0,
                0.273 / 2.0, 0.255, res);                   // バケツ

    // ========================================
    // コントロールステーション (Control station)
    // X: fence(-5.700) to 3400mm right = -2.300
    // Y: +3.000 to +5.800, H=760mm (desk height)
    // ========================================
    double cs_x0 = std::min(sign * 2.300, sign * 5.700);
    double cs_x1 = std::max(sign * 2.300, sign * 5.700);
    addBoxBounds(cloud, cs_x0, cs_x1, 3.000, 5.800, 0.0, 0.760, res);
}

int main()
{
    Cloud cloud;
    double res = 0.02;  // 20mm resolution

    // ================================================================
    // Field floor
    // Inner: X ±5.700, Y -4.700 to +5.800
    // ================================================================
    addBoxBounds(cloud, -5.700, 5.700, -4.700, 5.800, -0.01, 0.0, res);

    // ================================================================
    // Outer fences (150mm thick, ~150mm tall)
    // ================================================================
    // Top fence (Y+ side)
    addBoxBounds(cloud, -5.850, 5.850, 5.800, 5.950, 0.0, 0.150, res);
    // Bottom fence (Y- side)
    addBoxBounds(cloud, -5.850, 5.850, -4.850, -4.700, 0.0, 0.150, res);
    // Left fence (X- side)
    addBoxBounds(cloud, -5.850, -5.700, -4.700, 5.800, 0.0, 0.150, res);
    // Right fence (X+ side)
    addBoxBounds(cloud, 5.700, 5.850, -4.700, 5.800, 0.0, 0.150, res);

    // ================================================================
    // Center platform (教壇): W600 × full-length × H200
    // ================================================================
    addBoxBounds(cloud, -0.300, 0.300, -4.700, 5.800, 0.0, 0.200, res);

    // ================================================================
    // Center flag: base 300×300 on platform, pole φ40×H3000
    // ================================================================
    addBox(cloud, 0.0, 0.0, 0.200 + 0.025, 0.300, 0.300, 0.05, res);
    addCylinder(cloud, 0.0, 0.0, 0.200 + 1.425, 0.020, 2.75, res);

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
