#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <csm/csm.h> 
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <vector>

class PlicpOdometry : public rclcpp::Node
{
    public:
        PlicpOdometry(std::string name) : Node(name){
            RCLCPP_INFO(this->get_logger(), "Hello, world!");
            laserSub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10, std::bind(&PlicpOdometry::laserCallback, this, std::placeholders::_1));
            odomPub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

            tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
            tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            initParam();
        }
    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
        rclcpp::Time currentTime;
        rclcpp::Time lastIcpTime;
        bool getFirstScan = false;
        std::vector<double> angleCos;
        std::vector<double> angleSin;

        std::unique_ptr<tf2_ros::Buffer> tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
        tf2::Transform laserToBaseTf;
        tf2::Transform baseTolaserTf;
        tf2::Transform fixedToBaseTf;
        tf2::Transform prevFixedToBaseTf;
        tf2::Transform fixedToBaseTfKeyframe;

        std::string odomFrame;
        std::string baseFrame;

        double kfDistLinear;
        double kfDistLinearSq;
        double kfDistAngular;

        sm_params input;
        sm_result output;
        LDP prevLdpScan;

        void initParam(){
            baseFrame = "base_footprint";
            odomFrame = "odom";

            kfDistLinear = 0.1;
            kfDistAngular = 10.0 * M_PI / 180.0;
            kfDistLinearSq = kfDistLinear * kfDistLinear;

            fixedToBaseTf.setIdentity();
            prevFixedToBaseTf.setIdentity();
            fixedToBaseTfKeyframe.setIdentity();

            input.max_angular_correction_deg = 45.0;
            input.max_linear_correction = 0.5;
            input.max_iterations = 10;
            input.epsilon_xy = 0.000001;
            input.epsilon_theta = 0.000001;
            input.max_correspondence_dist = 0.3;
            input.sigma = 0.01;
            input.use_corr_tricks = 1;
            input.restart = 0;
            input.restart_threshold_mean_error = 0.01;
            input.restart_dt = 1.0;
            input.restart_dtheta = 0.1;
            input.clustering_threshold = 0.25;
            input.orientation_neighbourhood = 20;
            input.use_point_to_line_distance = 1;
            input.do_alpha_test = 0;
            input.do_alpha_test_thresholdDeg = 20.0;
            input.outliers_maxPerc = 0.9;
            input.outliers_adaptive_order = 0.7;
            input.outliers_adaptive_mult = 2.0;
            input.do_visibility_test = 0;
            input.outliers_remove_doubles = 1;
            input.do_compute_covariance = 0;
            input.debug_verify_tricks = 0;
            input.use_ml_weights = 0;
            input.use_sigma_weights = 0;

            input.laser[0] = 0.0;
            input.laser[1] = 0.0;
            input.laser[2] = 0.0;

            output.cov_x_m = 0;
            output.dx_dy1_m = 0;
            output.dx_dy2_m = 0;
        }

        void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            currentTime = msg->header.stamp;
            if(!getFirstScan){
                createCache(msg);
                if(!getBaseToLaserTf(msg->header.frame_id)){
                    RCLCPP_WARN(get_logger(), "Skipping scan");
                    return;
                }

                laserScanToLDP(msg, prevLdpScan);
                lastIcpTime = currentTime;
                getFirstScan = true;
            }
            LDP currLDPScan;
            laserScanToLDP(msg, currLDPScan);
            processScan(currLDPScan, currentTime);
        }

        void createCache(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            angleCos.clear(); angleSin.clear();
            double angle;
            for(unsigned int i = 0; i < msg->ranges.size(); i++){
                angle = msg->angle_min + msg->angle_increment * i;
                angleCos.push_back(cos(angle));
                angleSin.push_back(sin(angle));
            }
            input.min_reading = msg->range_min;
            input.max_reading = msg->range_max;
        }

        bool getBaseToLaserTf(const std::string &frameId){
            rclcpp::Time t = this->now();

            tf2::Stamped<tf2::Transform> tf;
            geometry_msgs::msg::TransformStamped laserPoseMsg;
            try{
                laserPoseMsg = tfBuffer->lookupTransform(baseFrame, frameId, t, rclcpp::Duration(10,0));
                tf.setOrigin(tf2::Vector3(
                    laserPoseMsg.transform.translation.x,
                    laserPoseMsg.transform.translation.y,
                    laserPoseMsg.transform.translation.z));
                tf2::Quaternion q(
                    laserPoseMsg.transform.rotation.x,
                    laserPoseMsg.transform.rotation.y,
                    laserPoseMsg.transform.rotation.z,
                    laserPoseMsg.transform.rotation.w);
                tf.setRotation(q);
            }
            catch( tf2::TransformException &ex){
                RCLCPP_INFO(this->get_logger(), "Could not get initial transform");
                return false;
            }
            baseTolaserTf = tf;
            laserToBaseTf = baseTolaserTf.inverse();
            return true;
        }

        void laserScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr& scan, LDP& ldp){
            unsigned int n = scan->ranges.size();
            ldp = ld_alloc_new(n);
            for(unsigned int i = 0; i < n; i++){
                double r = scan->ranges[i];
                if(r > scan->range_min && r < scan->range_max){
                    ldp->valid[i] = 1;
                    ldp->readings[i] = r;
                }
                else{
                    ldp->valid[i] = 0;
                    ldp->readings[i] = -1;
                }
                ldp->theta[i] = scan->angle_min + i * scan->angle_increment;
                ldp->cluster[i] = -1;
                
            }
            ldp->min_theta = ldp->theta[0];
            ldp->max_theta = ldp->theta[n-1];

            ldp->odometry[0] = 0.0;
            ldp->odometry[1] = 0.0;
            ldp->odometry[2] = 0.0;

            ldp->true_pose[0] = 0.0;
            ldp->true_pose[1] = 0.0;
            ldp->true_pose[2] = 0.0;
        }

        bool processScan(LDP& curLdpScan, const rclcpp::Time& time){
            prevLdpScan->odometry[0] = 0.0;
            prevLdpScan->odometry[1] = 0.0;
            prevLdpScan->odometry[2] = 0.0;

            prevLdpScan->estimate[0] = 0.0;
            prevLdpScan->estimate[1] = 0.0;
            prevLdpScan->estimate[2] = 0.0;

            prevLdpScan->true_pose[0] = 0.0;
            prevLdpScan->true_pose[1] = 0.0;
            prevLdpScan->true_pose[2] = 0.0;

            input.laser_ref  = prevLdpScan; // first scan for reference
            input.laser_sens = curLdpScan;  // second scan

            tf2::Transform predictChangeInLaser;

            double dt = (now()-lastIcpTime).nanoseconds() / 1e9;
            // double predictChangeX, predictChangeY, predictChangeA;

            tf2::Transform predictChange;
            createTfFromXYTheta(0.0, 0.0, 0.0, predictChange);

            predictChange = predictChange * (fixedToBaseTf * fixedToBaseTfKeyframe.inverse());
        
            predictChangeInLaser = laserToBaseTf * fixedToBaseTf.inverse()*predictChange*fixedToBaseTf*baseTolaserTf;
            input.first_guess[0] = predictChangeInLaser.getOrigin().getX();
            input.first_guess[1] = predictChangeInLaser.getOrigin().getY();
            input.first_guess[2] = tf2::getYaw(predictChangeInLaser.getRotation());

            if(output.cov_x_m){
                gsl_matrix_free(output.cov_x_m);
                output.cov_x_m = 0;
            }
            if(output.dx_dy1_m){
                gsl_matrix_free(output.dx_dy1_m);
                output.dx_dy1_m = 0;
            }
            if(output.dx_dy2_m){
                gsl_matrix_free(output.dx_dy2_m);
                output.dx_dy2_m = 0;
            }

            // Scan matching
            sm_icp(&input, &output);
            tf2::Transform corrChange;

            if(output.valid){
                tf2::Transform corrChangeInLaser;
                createTfFromXYTheta(output.x[0], output.x[1], output.x[2], corrChangeInLaser);
                corrChange = baseTolaserTf*corrChangeInLaser*laserToBaseTf;
                fixedToBaseTf = fixedToBaseTfKeyframe * corrChange;
            }else{
                corrChange.setIdentity();
                RCLCPP_WARN(get_logger(), "ICP failed");
                return false;
            }
            // publish odom
            nav_msgs::msg::Odometry odomMsg;
            odomMsg.header.stamp = time;
            odomMsg.header.frame_id = odomFrame;
            odomMsg.child_frame_id = baseFrame;
            odomMsg.pose.pose.position.x = fixedToBaseTf.getOrigin().x();
            odomMsg.pose.pose.position.y = fixedToBaseTf.getOrigin().y();
            odomMsg.pose.pose.position.z = fixedToBaseTf.getOrigin().z();

            odomMsg.pose.pose.orientation.x = fixedToBaseTf.getRotation().x();
            odomMsg.pose.pose.orientation.y = fixedToBaseTf.getRotation().y();
            odomMsg.pose.pose.orientation.z = fixedToBaseTf.getRotation().z();
            odomMsg.pose.pose.orientation.w = fixedToBaseTf.getRotation().w();

            auto poseDifference = prevFixedToBaseTf.inverse() * fixedToBaseTf;
            odomMsg.twist.twist.linear.x = poseDifference.getOrigin().x() / dt;
            odomMsg.twist.twist.linear.y = poseDifference.getOrigin().y() / dt;
            odomMsg.twist.twist.angular.z = tf2::getYaw(poseDifference.getRotation()) / dt;

            prevFixedToBaseTf = fixedToBaseTf;
            odomPub->publish(odomMsg);

            // publish tf
            geometry_msgs::msg::TransformStamped tfMsg;
            tfMsg.header.stamp = time;
            tfMsg.header.frame_id = odomFrame;
            tfMsg.child_frame_id = baseFrame;
            tfMsg.transform.translation.x = fixedToBaseTf.getOrigin().x();
            tfMsg.transform.translation.y = fixedToBaseTf.getOrigin().y();
            tfMsg.transform.translation.z = fixedToBaseTf.getOrigin().z();
            tfMsg.transform.rotation.x = fixedToBaseTf.getRotation().x();
            tfMsg.transform.rotation.y = fixedToBaseTf.getRotation().y();
            tfMsg.transform.rotation.z = fixedToBaseTf.getRotation().z();
            tfMsg.transform.rotation.w = fixedToBaseTf.getRotation().w();

            tfBroadcaster->sendTransform(tfMsg);

            if(newKeyFrameNeeded(corrChange)){
                ld_free(prevLdpScan);
                prevLdpScan = curLdpScan;
                fixedToBaseTfKeyframe = fixedToBaseTf;
            }else{
                ld_free(curLdpScan);
            
            }
            lastIcpTime = now();
            return true;
        }
        void createTfFromXYTheta(double x, double y, double theta, tf2::Transform& t){
            t.setOrigin(tf2::Vector3(x, y, 0.0));
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, theta);
            t.setRotation(q);
        }
        bool newKeyFrameNeeded(const tf2::Transform& d){
            if(fabs(tf2::getYaw(d.getRotation())) > kfDistAngular){
                return true;
            }
            double x = d.getOrigin().x();
            double y = d.getOrigin().y();
            if(x*x+y*y > kfDistLinearSq){
                return true;
            }
            return false;
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlicpOdometry>("plicp_odometry");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}