#ifndef _WALLFOLLOWER_H_
#define _WALLFOLLOWER_H_

#include <iostream>
#include <cmath>
#include <stdlib.h>     // for using rand()

#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class WallFollower {
public:
    typedef std::shared_ptr<WallFollower> Ptr;

    WallFollower(ros::NodeHandle& nh, GridMap::Ptr& grid_map_ptr);
	~WallFollower(){}


private:
    struct PtsEndFov {
        /* param */
        double f_;
        double deltaY_, deltaZ_;            // width and height in metric unit
        double X_;                          // depth
        Eigen::Matrix3d R_turn_round_;
        ros::Publisher pts_end_fov_pub_;    // only for test

        /* data */
        int width_idx_, height_idx_;
        double cx_, cy_;
        std::vector<Eigen::Vector3d> pts_end_body_;
        std::vector<Eigen::Vector3d> pts_end_world_;

        typedef std::shared_ptr<PtsEndFov> Ptr;

        PtsEndFov(ros::NodeHandle& nh);
        void publicPtsEndFov();
    };

    struct PlaneFitter {
        struct Plane {
            Eigen::Vector3d v_, p_;
        };

        /* param */
        int iters_;
        double sigma_;                       // margin of a point in the given plane
        double p_;                           // posibilaty of picking 3 inliers
        bool occupied_pts_updated_;
        ros::Publisher occupied_pts_pub_;    // only for test
        ros::Publisher plane_fitting_arrow_pub_;    // only for test
        bool have_plane_;

        /* data */
        std::vector<Eigen::Vector3d> occupied_pts_;
        Plane current_plane_, last_best_plane_;

        typedef std::shared_ptr<PlaneFitter> Ptr;

        PlaneFitter(ros::NodeHandle& nh);
        void publicOccupiedPts();
        void publicPlaneFittingArrow();
        void solvePlane(std::vector<Eigen::Vector3d> &random_3pts);
        inline double solveDistance(Eigen::Vector3d &pt, Plane &plane);
    };

    /* param */
    PtsEndFov::Ptr pts_end_fov_ptr_;
    PlaneFitter::Ptr plane_fitter_ptr_;
    
    GridMap::Ptr grid_map_ptr_;

    ros::Timer vis_timer_, find_waypoint_timer_;
    ros::Subscriber odom_sub_, waypoint_sub_;
    ros::Publisher waypoint_pub_;

    bool have_odom_, is_next_waypoint_initialized_;
    double dist_from_wall_;
    int have_plane_threshold_;
    double reach_waypoint_threshold_;
    int max_planned_waypoints_num_;
    bool wall_follower_enable_;

    /* data */
    Eigen::Vector3d body_pos_, next_way_point_;
    Eigen::Matrix3d body_r_m_;
    int planned_waypoints_count_;

    void ptsEndFovGeneration();
    PlaneFitter::Plane planeFitting();
    bool findNextWayPoint();
    void findWayPointCallback(const ros::TimerEvent& /*event*/);
    void visCallback(const ros::TimerEvent& /*event*/);
    void odomCallback(const nav_msgs::OdometryConstPtr& odom);
    void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    void publicWayPoint(Eigen::Vector3d waypoint);

};


#endif
