#include "wall_follower/wall_follower.h"

#define M_PI    3.14159265358979323846  /* pi */

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

WallFollower::WallFollower(ros::NodeHandle& nh, GridMap::Ptr& grid_map_ptr)
{
    nh.param("wall_follower/wall_follower_enable", wall_follower_enable_, false);

    if (wall_follower_enable_ == true) {
        ROS_INFO("Wall follower init begin.");
        
        nh.param("wall_follower/dist_from_wall", dist_from_wall_, 0.5);
        nh.param("wall_follower/have_plane_threshold", have_plane_threshold_, 20);
        nh.param("wall_follower/reach_waypoint_threshold", reach_waypoint_threshold_, 0.5);
        nh.param("wall_follower/max_planned_waypoints_num", max_planned_waypoints_num_, 10);

        grid_map_ptr_ = grid_map_ptr;

        pts_end_fov_ptr_.reset(new PtsEndFov(nh));
        ptsEndFovGeneration();
        plane_fitter_ptr_.reset(new PlaneFitter(nh));

        odom_sub_ = nh.subscribe<nav_msgs::Odometry>("grid_map/odom", 10, &WallFollower::odomCallback, this);
        have_odom_ = false;
        is_next_waypoint_initialized_ = false;

        waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &WallFollower::waypointCallback, this);
        waypoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

        find_waypoint_timer_ = nh.createTimer(ros::Duration(0.5), &WallFollower::findWayPointCallback, this);
        vis_timer_ = nh.createTimer(ros::Duration(0.2), &WallFollower::visCallback, this); 
    }   
}


void WallFollower::visCallback(const ros::TimerEvent& /*event*/)
{
    pts_end_fov_ptr_->publicPtsEndFov();
    plane_fitter_ptr_->publicOccupiedPts();
    plane_fitter_ptr_->publicPlaneFittingArrow();
}


void WallFollower::findWayPointCallback(const ros::TimerEvent& /*event*/)
{
    if (have_odom_) {
        if (!is_next_waypoint_initialized_) {
            next_way_point_ = body_pos_;
            is_next_waypoint_initialized_ = true;
            planned_waypoints_count_ = 1;
            ros::Duration(2.0).sleep();
        }
        // std::cout << "MY_DEBUG: dist = " << (body_pos_-next_way_point_).norm() << std::endl;
        if (planned_waypoints_count_ < max_planned_waypoints_num_) {
            if ((body_pos_-next_way_point_).norm() < reach_waypoint_threshold_) {
                findNextWayPoint();
                planned_waypoints_count_++;
            }
        }
    }
}


void WallFollower::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  body_pos_(0) = odom->pose.pose.position.x;
  body_pos_(1) = odom->pose.pose.position.y;
  body_pos_(2) = odom->pose.pose.position.z;

  body_r_m_ = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                       odom->pose.pose.orientation.x,
                                       odom->pose.pose.orientation.y,
                                       odom->pose.pose.orientation.z).toRotationMatrix();

  have_odom_ = true;
}

void WallFollower::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    if (msg->pose.position.z < -0.1)
        return;

    next_way_point_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, 1.0);
}

bool WallFollower::findNextWayPoint()
{
    std::vector<Eigen::Vector3d> pts_end, occupied_pts;
    Eigen::Matrix3d body_r_m;
    Eigen::Vector3d body_pos, ray_pt;
    Eigen::Vector3i id;
    double map_resolution;
    RayCaster raycaster;

    Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);

    pts_end = pts_end_fov_ptr_->pts_end_body_;

    body_r_m = body_r_m_;
    body_pos = body_pos_;

    map_resolution = grid_map_ptr_->mp_.resolution_;

    /* raycasting */
    for (auto& pt_end: pts_end) {
        pt_end = body_r_m * pt_end + body_pos;

        raycaster.setInput(body_pos/map_resolution, pt_end/map_resolution);
        while (raycaster.step(ray_pt)) {
            Eigen::Vector3d tmp = (ray_pt + half) * map_resolution;
            grid_map_ptr_->posToIndex(tmp, id);
            if (grid_map_ptr_->isKnownOccupied(id)) {
                occupied_pts.push_back(tmp);
                break;
            }
        }
    }
    plane_fitter_ptr_->occupied_pts_ = occupied_pts;
    
    /* only for visual test, remember del or comment */
    pts_end_fov_ptr_->pts_end_world_ = pts_end;
    
    /* wall existing */
    if (plane_fitter_ptr_->occupied_pts_.size() > (std::size_t)have_plane_threshold_) {
        plane_fitter_ptr_->last_best_plane_ = planeFitting();
        /* make arrow always point to agent's side */
        double tmp = plane_fitter_ptr_->last_best_plane_.v_.transpose() * 
                        (body_pos - plane_fitter_ptr_->last_best_plane_.p_);
        int sign = sgn(tmp);
        plane_fitter_ptr_->last_best_plane_.v_ = (double)sign * plane_fitter_ptr_->last_best_plane_.v_;
        plane_fitter_ptr_->have_plane_ = true;
    }
    /* wall absent */
    else {
        plane_fitter_ptr_->have_plane_ = false;
    }

    /* generate next waypoint and pub */
    if (plane_fitter_ptr_->have_plane_) {
        next_way_point_ = plane_fitter_ptr_->last_best_plane_.p_ + 
                            dist_from_wall_ * plane_fitter_ptr_->last_best_plane_.v_;
        next_way_point_(2) = 1.0;

        /* check if next waypoint is occupied */
        grid_map_ptr_->posToIndex(next_way_point_, id);
        /* occupied */
        if (grid_map_ptr_->isKnownOccupied(id)) {
            raycaster.setInput(body_pos/map_resolution, next_way_point_/map_resolution);
            while (raycaster.step(ray_pt)) {
                Eigen::Vector3d tmp = (ray_pt + half) * map_resolution;
                grid_map_ptr_->posToIndex(tmp, id);

                if (grid_map_ptr_->isKnownOccupied(id)) {
                    next_way_point_ = body_pos + 0.8 * (tmp - body_pos);
                    break;
                }
            }
        }
        std::cout << "MY_DEBUG: next_way_point_ = " << next_way_point_.transpose() << std::endl;
        std::cout << "MY_DEBUG: publicWayPoint(next_way_point_) start" << std::endl;
        publicWayPoint(next_way_point_);
        std::cout << "MY_DEBUG: publicWayPoint(next_way_point_) end" << std::endl;
    }
}


WallFollower::PtsEndFov::PtsEndFov(ros::NodeHandle& nh)
{
    nh.param("wall_follower/f", f_, 30.0);
    nh.param("wall_follower/deltaY", deltaY_, 4.0);
    nh.param("wall_follower/deltaZ", deltaZ_, 2.0);
    nh.param("wall_follower/X", X_, 3.0);

    Eigen::Matrix3d R_turn_round =  Eigen::Quaterniond(cos(-M_PI/4.0/2.0), 0, 0, sin(-M_PI/4.0/2.0)).toRotationMatrix();
    // Eigen::Matrix3d R_turn_round =  Eigen::Quaterniond(1, 0, 0, 0).toRotationMatrix();

    nh.param("wall_follower/R_turn_round_00", R_turn_round_(0,0), R_turn_round(0,0));
    nh.param("wall_follower/R_turn_round_01", R_turn_round_(0,1), R_turn_round(0,1));
    nh.param("wall_follower/R_turn_round_02", R_turn_round_(0,2), R_turn_round(0,2));
    nh.param("wall_follower/R_turn_round_10", R_turn_round_(1,0), R_turn_round(1,0));
    nh.param("wall_follower/R_turn_round_11", R_turn_round_(1,1), R_turn_round(1,1));
    nh.param("wall_follower/R_turn_round_12", R_turn_round_(1,2), R_turn_round(1,2));
    nh.param("wall_follower/R_turn_round_20", R_turn_round_(2,0), R_turn_round(2,0));
    nh.param("wall_follower/R_turn_round_21", R_turn_round_(2,1), R_turn_round(2,1));
    nh.param("wall_follower/R_turn_round_22", R_turn_round_(2,2), R_turn_round(2,2));

    width_idx_ = (int)std::ceil(f_/X_ * deltaY_);
    height_idx_ = (int)std::ceil(f_/X_ * deltaZ_);

    cx_ = width_idx_ / 2.0;
    cy_ = height_idx_ / 2.0;

    std::cout << std::endl;
    std::cout << "wall follower param and data info: " << std::endl;
    std::cout << "f_ = " << f_ << std::endl;
    std::cout << "deltaY_ = " << deltaY_ << std::endl;
    std::cout << "deltaZ_ = " << deltaZ_ << std::endl;
    std::cout << "R_turn_round_ = " << R_turn_round_ << std::endl;
    std::cout << "width_idx_ = " << width_idx_ << std::endl;
    std::cout << "height_idx_ = " << height_idx_ << std::endl;
    std::cout << std::endl;
    

    pts_end_fov_pub_ = nh.advertise<sensor_msgs::PointCloud2>("wall_follower/pts_end_fov", 10);

}


void WallFollower::ptsEndFovGeneration()
{
    Eigen::Vector3d pt_end;
    Eigen::Matrix3d R_turn_round;
    double X, f;
    int width_idx, height_idx;

    PtsEndFov::Ptr pts_end_fov_ptr;

    pts_end_fov_ptr = pts_end_fov_ptr_;

    X = pts_end_fov_ptr->X_;
    width_idx = pts_end_fov_ptr->width_idx_;
    height_idx = pts_end_fov_ptr->height_idx_;
    f = pts_end_fov_ptr->f_;
    R_turn_round = pts_end_fov_ptr->R_turn_round_;

    pt_end(0) = X;
    for (int u = 0; u < width_idx; u++) {
        for (int v = 0; v < height_idx; v++) {
            pt_end(1) = -X/f * (u - width_idx/2);
            pt_end(2) = -X/f * (v - height_idx/2);
            pts_end_fov_ptr->pts_end_body_.push_back(R_turn_round * pt_end);
        }
    }

    pts_end_fov_ptr->pts_end_world_ = pts_end_fov_ptr->pts_end_body_;

    // std::cout << std::endl;
    // std::cout << "pts_end_body_.size() = " << pts_end_body_.size() << std::endl;
    // std::cout << "pts_end_world_.size() = " << pts_end_world_.size() << std::endl;
    // std::cout << std::endl;

}


WallFollower::PlaneFitter::PlaneFitter(ros::NodeHandle& nh)
{
    nh.param("wall_follower/plan_fitter/iters", iters_, 1000);
    nh.param("wall_follower/plan_fitter/sigma", sigma_, 0.05);
    nh.param("wall_follower/plan_fitter/p", p_, 0.99);
    have_plane_ = false;
    occupied_pts_updated_ = false;
    occupied_pts_pub_ = nh.advertise<sensor_msgs::PointCloud2>("wall_follower/plane_fitter/occupied_pts", 10);
    plane_fitting_arrow_pub_ = nh.advertise<visualization_msgs::Marker>("wall_follower/plane_fitter/plane_fitting_arrow", 2);
}

void WallFollower::PlaneFitter::publicOccupiedPts()
{
    if (occupied_pts_pub_.getNumSubscribers() <= 0)
            return;

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::vector<Eigen::Vector3d> occupied_pts;

    occupied_pts = occupied_pts_;

    for (auto& occupied_pt: occupied_pts) {
        pt.x = occupied_pt(0);
        pt.y = occupied_pt(1);
        pt.z = occupied_pt(2);
        cloud.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = string("world");
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    occupied_pts_pub_.publish(cloud_msg);
}


void WallFollower::PlaneFitter::solvePlane(std::vector<Eigen::Vector3d> &random_3pts)
{
    Eigen::Matrix3d matrix_3pts;

    matrix_3pts.row(0) = random_3pts[0];
    matrix_3pts.row(1) = random_3pts[1];
    matrix_3pts.row(2) = random_3pts[2];

    current_plane_.v_ = matrix_3pts.colPivHouseholderQr().solve(Eigen::Vector3d::Ones());
    current_plane_.v_ = current_plane_.v_.normalized();
    current_plane_.p_ = random_3pts[0];
}

inline double WallFollower::PlaneFitter::solveDistance(Eigen::Vector3d &pt, Plane &plane)
{
    return std::abs(plane.v_.transpose() * (pt - plane.p_));
}


WallFollower::PlaneFitter::Plane WallFollower::planeFitting()
{
    int iters = plane_fitter_ptr_->iters_;
    std::vector<Eigen::Vector3d> &occupied_pts = plane_fitter_ptr_->occupied_pts_;
    std::vector<Eigen::Vector3d> random_3pts;
    int inlier_cnt, max_inlier_cnt;
    PlaneFitter::Plane last_best_plane;

    /* RANSAC iters */
    max_inlier_cnt = 0;
    for (int i = 0; i < iters; i++) {
        /* random picks 3 points from occupied_pts_*/
        random_3pts.clear();
        for (int j = 0; j < 3; j++) {
            int index = rand()%occupied_pts.size();
            Eigen::Vector3d rand_pt = occupied_pts[index];
            random_3pts.push_back(rand_pt);
        }

        plane_fitter_ptr_->solvePlane(random_3pts);

        /* find inlier */
        inlier_cnt = 0;
        for (auto &pt: occupied_pts) {
            double d = plane_fitter_ptr_->solveDistance(pt, plane_fitter_ptr_->current_plane_);
            if (d < plane_fitter_ptr_->sigma_) {
                inlier_cnt++;
            }
        }

        /* update best model */
        if (inlier_cnt > max_inlier_cnt) {
            max_inlier_cnt = inlier_cnt;
            last_best_plane = plane_fitter_ptr_->current_plane_;

            double k = (double)inlier_cnt / (double)(occupied_pts.size());
            iters = std::log(1-plane_fitter_ptr_->p_) / std::log(1-std::pow(k,3));
        }

        /* If model is good enough */
        if (inlier_cnt > 0.5*occupied_pts.size()) {
            break;
        }
    }

    /* get the point of most front in the best plane */
    double max_temp = 0;
    for (auto &pt: occupied_pts) {
        double d = plane_fitter_ptr_->solveDistance(pt, last_best_plane);
        if (d < plane_fitter_ptr_->sigma_) {
            double x_local = (body_r_m_.transpose() * (pt - body_pos_))(0);
            if (x_local > max_temp) {
                max_temp = x_local;
                last_best_plane.p_ = pt;
            }
        }
    }

    return last_best_plane;

}


void WallFollower::PlaneFitter::publicPlaneFittingArrow() 
{
    visualization_msgs::Marker arrow;
    geometry_msgs::Point start, end;
    if (have_plane_) {
        arrow.header.frame_id = "world";
        arrow.header.stamp = ros::Time();
        arrow.ns = "plane_fitting";
        arrow.id = 0;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;

        start.x = last_best_plane_.p_(0);
        start.y = last_best_plane_.p_(1);
        start.z = last_best_plane_.p_(2);
        Eigen::Vector3d v_end = last_best_plane_.p_ + last_best_plane_.v_;
        end.x = v_end(0);
        end.y = v_end(1);
        end.z = v_end(2);

        arrow.points.clear();
        arrow.points.push_back(start);
        arrow.points.push_back(end);

        arrow.scale.x = 0.1;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;
        arrow.color.a = 1.0; // Don't forget to set the alpha!
        arrow.color.r = 0.0;
        arrow.color.g = 1.0;
        arrow.color.b = 0.0;
        //only if using a MESH_RESOURCE marker type:
        // arrow.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        plane_fitting_arrow_pub_.publish(arrow);
    }
    else {
        /* clean */
        arrow.header.frame_id = "world";
        start.x = 0;
        start.y = 0;
        start.z = 0;
        arrow.points.clear();
        arrow.points.push_back(start);
        arrow.points.push_back(start);
        plane_fitting_arrow_pub_.publish(arrow);
    }
}


void WallFollower::PtsEndFov::publicPtsEndFov()
{
    if (pts_end_fov_pub_.getNumSubscribers() <= 0)
        return;

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::vector<Eigen::Vector3d> pts_end_world;

    pts_end_world = pts_end_world_;

    for (auto& pt_end_world: pts_end_world) {
        pt.x = pt_end_world(0);
        pt.y = pt_end_world(1);
        pt.z = pt_end_world(2);
        cloud.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = string("world");
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    pts_end_fov_pub_.publish(cloud_msg);

}


void WallFollower::publicWayPoint(Eigen::Vector3d waypoint)
{
    geometry_msgs::PoseStamped wp;

    wp.header.frame_id = "world";
    wp.header.stamp = ros::Time();

    wp.pose.position.x = waypoint(0);
    wp.pose.position.y = waypoint(1);
    wp.pose.position.z = waypoint(2);

    waypoint_pub_.publish(wp);

}

