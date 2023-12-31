#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include "gmapping/grid/map.h"
#include "gmapping/grid/gridlinetraversal.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace gmapping{

#define GMAPPING_UNKNOWN (-1)
#define GMAPPING_FREE (0)
#define GMAPPING_OCC (100)
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class GMapping : public rclcpp::Node{
public:
    GMapping(): Node("gmapping"){
        mapPub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
        mapMetadataPub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_metadata", 10);
        // Create a subscriber on the topic "scan"
        scanSub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&GMapping::scanCallback, this, std::placeholders::_1));
        odomSub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&GMapping::odomCallback, this, std::placeholders::_1)); 

        tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
        tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }
private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapMetadataPub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

    nav_msgs::msg::OccupancyGrid map;
    double maxRange, maxUseRange;
    double xmin, ymin, xmax, ymax;
    double resolution;
    double occThresh;

    bool getFirstScan = false;
    std::vector<double> angleCos;
    std::vector<double> angleSin;

    std::vector<GridLineTraversalLine> lineLists;
    std::vector<Point> hitLists;

    void initParams(){

    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        if(!getFirstScan){
            createCache(msg);
            getFirstScan = true;
        }
        publishMap(msg);
    }

    void createCache(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        angleCos.clear(); angleSin.clear();
        double angle;
        for(unsigned int i = 0; i < msg->ranges.size(); i++){
            angle = msg->angle_min + msg->angle_increment * i;
            angleCos.push_back(cos(angle));
            angleSin.push_back(sin(angle));
        }
    }

    void publishMap(const sensor_msgs::msg::LaserScan::SharedPtr &msg){
        Point center;
        center.x = (xmin + xmax) / 2.0;
        center.y = (ymin + ymax) / 2.0;

        ScanMatcherMap gmappingMap(center, xmin, ymin, xmax, ymax, resolution);

        computeMap(gmappingMap, msg);

        for (int x = 0; x < gmappingMap.getMapSizeX(); x++){
            for (int y = 0; y < gmappingMap.getMapSizeY(); y++)
            {
                IntPoint p(x, y);
                // 获取这点栅格的值，只有大于occ_thresh_时才认为是占用
                double occ = gmappingMap.cell(p); 

                // 未知
                if (occ < 0){
                    this->map.data[MAP_IDX(this->map.info.width, x, y)] = GMAPPING_UNKNOWN;
                }
                // 占用
                else if (occ > occThresh){ // 默认0.25
                    this->map.data[MAP_IDX(this->map.info.width, x, y)] = GMAPPING_OCC;
                }
                // 空闲
                else{
                    this->map.data[MAP_IDX(this->map.info.width, x, y)] = GMAPPING_FREE;
                }
            }
        }

        // 添加当前的时间戳
        map.header.stamp = this->now();
        map.header.frame_id = msg->header.frame_id;

        // 发布map和map_metadata
        this->mapPub->publish(this->map);
        // this->mapMetadataPub->publish(this->map.info);
    }

    void computeMap(ScanMatcherMap &gmappingMap, const sensor_msgs::msg::LaserScan::SharedPtr &msg){
        lineLists.clear(); hitLists.clear();

        OrientedPoint lp(0, 0, 0.0);
        IntPoint p0 = gmappingMap.world2map(lp);

        HierarchicalArray2D<PointAccumulator>::PointSet activeArea;

        for (unsigned int i = 0; i < msg->ranges.size(); i++){
            // filtering invalid scan points
            double d = msg->ranges[i];
            if (d > maxRange || d == 0.0 || !std::isfinite(d)){
                continue;
            }
            if (d > maxUseRange){
                d = maxUseRange;
            }

            // p1为激光雷达的数据点在地图坐标系下的坐标
            Point phit = lp;
            phit.x += d * angleCos[i];
            phit.y += d * angleSin[i];
            IntPoint p1 = gmappingMap.world2map(phit);

            // 使用bresenham算法来计算 从激光位置到激光点 要经过的栅格的坐标
            GridLineTraversalLine line;
            GridLineTraversal::gridLine(p0, p1, &line);
            // 将line保存起来以备后用
            lineLists.push_back(line);

            // 计算活动区域的大小
            for (int i = 0; i < line.num_points - 1; i++){
                activeArea.insert(gmappingMap.storage().patchIndexes(line.points[i]));
            }
            // 如果d<m_usableRange则需要把击中点也算进去 说明这个值是好的。
            // 同时如果d==max_use_range_ 那么说明这个值只用来进行标记空闲区域　不用来进行标记障碍物
            if (d < maxUseRange){
                IntPoint cp = gmappingMap.storage().patchIndexes(p1);
                activeArea.insert(cp);
                hitLists.push_back(phit);
            }
        }

        // 为activeArea分配内存
        gmappingMap.storage().setActiveArea(activeArea, true);
        gmappingMap.storage().allocActiveArea();

        // 在map上更新空闲点
        for (auto line : lineLists){
            // 更新空闲位置
            for (int k = 0; k < line.num_points - 1; k++){
                // 未击中，就不记录击中的位置了，所以传入参数Point(0,0)
                gmappingMap.cell(line.points[k]).update(false, Point(0, 0));
            }
        }
        // 在map上添加hit点
        for (auto hit : hitLists)
        {
            IntPoint p1 = gmappingMap.world2map(hit);
            gmappingMap.cell(p1).update(true, hit);
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        // 获取激光雷达在odom坐标系下的坐标
        tf2::Transform laserPose;
        tf2::fromMsg(msg->pose.pose, laserPose);

        // 获取激光雷达在map坐标系下的坐标
        tf2::Transform mapPose;
        try{
            mapPose = tfBuffer->lookupTransform("map", "odom", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex){
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }
        tf2::Transform laserPoseInMap = mapPose * laserPose;

        // 发布map->odom的tf
        geometry_msgs::msg::TransformStamped mapToOdom;
        mapToOdom.header.stamp = this->now();
        mapToOdom.header.frame_id = "map";
        mapToOdom.child_frame_id = "odom";
        mapToOdom.transform = tf2::toMsg(mapPose);
        tfBroadcaster->sendTransform(mapToOdom);

        // 发布odom->base_link的tf
        geometry_msgs::msg::TransformStamped odomToBaseLink;
        odomToBaseLink.header.stamp = this->now();
        odomToBaseLink.header.frame_id = "odom";
        odomToBaseLink.child_frame_id = "base_link";
        odomToBaseLink.transform = tf2::toMsg(laserPose);
        tfBroadcaster->sendTransform(odomToBaseLink);
    }
};

} // namespace gmapping

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gmapping::GMapping>());
    rclcpp::shutdown();
    return 0;

}