
#include "path_planner_node.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string.h>
#include <tf/transform_datatypes.h>
#include <boost/bind.hpp>
#include <boost/shared_array.hpp>
#include "Astar.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <typeinfo>
#include <math.h>
#include <Eigen/Dense>
#include <unistd.h>


using namespace Astar_planner;
namespace PathPlanner {
PathPlannerNode::PathPlannerNode() {
  cmd_id = "";
  next_cmd_id = 0;
  tfListener.reset(new tf2_ros::TransformListener(tf_buffer));
  n.param<std::string>("base_frame", base_frame, "secondary_robot");
  n.param<std::string>("world_frame", world_frame, "map");
  path_publisher = n.advertise<nav_msgs::Path>("path", 10);
   path_publisher2 = n.advertise<nav_msgs::Path>("path2", 10);

  move_cmd_publisher = n.advertise<std_msgs::String>("/navigation/move_command", 10);
  goal_sub = n.subscribe("goal", 10, &PathPlannerNode::goalCallback, this);
  map_sub = n.subscribe("map", 10, &PathPlannerNode::mapCallback, this);
  move_response_sub = n.subscribe("/response", 10, &PathPlannerNode::responseCallback, this);
  ROS_INFO("Path planner node is init");
}
void PathPlannerNode::calculateGradient(cv::Mat &gx, cv::Mat &gy, cv::Mat cost_map)
{
  std::vector<uint8_t> map_data;
  int h = map.height;
    int w = map.width;
    for (int i = 0; i < h*w; i++)
     {
      if (map.data[i] == true)
        map_data.push_back(100);
      else
        map_data.push_back(0);
     }
    //const nav_msgs::OccupancyGrid& cost = map;

    cv::Mat cost_mat(h, w, CV_8UC1);
    memcpy(cost_mat.data, map_data.data(), map_data.size() * sizeof(uint8_t));

    /// Sobel Version
    //    double s = 1e-2;
    //    cv::Sobel(cost_mat, gx, CV_32F, 1, 0, 5, s);
    //    cv::Sobel(cost_mat, gy, CV_32F, 0, 1, 5, s);

    /// Minimum Search
    gx = cv::Mat(map.height, map.width, CV_32FC1, cv::Scalar::all(0));
    gy = cv::Mat(map.height, map.width, CV_32FC1, cv::Scalar::all(0));

    int d = 5;

    for(int y = 0; y < gx.rows; ++y) {
        for(int x = 0; x < gx.cols; ++x) {
            int cost = cost_mat.at<uchar>(y,x);

            if(cost == 0) {
                continue;
            }

            int min_x, min_y;
            int min_cost = 512;
            float min_dist = 2 * d;

            int starty = std::max(0, y - d);
            int endy = std::min(gx.rows - 1, y + d);
            for(int ny = starty; ny < endy; ++ny) {

                int startx = std::max(0, x - d);
                int endx = std::min(gx.rows - 1, x + d);
                for(int nx = startx; nx < endx; ++nx) {

                    int ncost = cost_mat.at<uchar>(ny,nx);
                    float dist_n = std::hypot(float(x-nx), float(y-ny));
                    if(ncost < min_cost || (ncost == min_cost && dist_n < min_dist)) {
                        min_x = nx;
                        min_y = ny;

                        min_cost = ncost;
                        min_dist = dist_n;
                    }
                }
            }

            if(min_cost < 512) {
                int dx = (x - min_x);
                int dy = (y - min_y);

                if(dx != 0 || dy != 0) {
                    float norm = std::hypot((float) dx, (float) dy);
                    float len = cost / 10.0;// std::abs(min_cost - cost) / 10.0;

                    float f = len / norm;// / norm;

                    dx *= f;
                    dy *= f;

                    gx.at<float>(y,x) = dx;
                    gy.at<float>(y,x) = dy;
                }
            }
        }
    }
}

bool PathPlannerNode::isFree(std::vector<uint8_t> cost_map, Pose2d segment_prev, Pose2d segment_next, Point segment_govno)
{
    //ROS_INFO("is Free&&");
   if (cost_map[segment_govno.x + segment_govno.y * map.width] > 10)
     return false;
//    std::size_t idx = segment_prev.y * map.width + segment_prev.x;
    if (map.data[segment_govno.x + segment_govno.y * map.width])
        return false;
    return true;
}

std::vector<Pose2d> PathPlannerNode::simplifyPath(std::vector<Pose2d> segment, std::vector<uint8_t> cost_map, std::vector<Point> segment_govno)
{
    std::vector<Pose2d> result = segment;
        for(std::size_t i = 1; i < segment.size() - 1;) {
            // check if i can be removed
            if(isFree(cost_map, result[i-1], result[i+1], segment_govno[i-1])) {
                ROS_INFO("CHECK COMPETED");
                result.erase(result.begin() + i);
                //ROS_INFO("CHECK COMPETED");
            } else {
                ++i;
            }
        }
    return result;
}
//std::vector<Pose2d> PathPlannerNode::optimizePathCost(const std::vector<Pose2d>& path_raw, cv::Mat cost_mat) {
//    if(!map) {
//        return path_raw;
//    }
//
//    std::vector<Pose2d> new_path = path_raw;
//
//    cv::Mat gx, gy;
//    calculateGradient(gx, gy, cost_mat);
//    double last_change = -2 * cost_optimization_tolerance;
//    double change = 0;
//
//    int offset = 2;
//
//
//    for(Pose2d segment : new_path) {
//        unsigned n = 1;
//        std::vector<Pose2d> optimized_segment = segment;
//
//        std::vector<double> gradients_x(n);
//        std::vector<double> gradients_y(n);
//        std::vector<double> magnitudes(n);
//
//        std::vector<double> dist_to_start(n);
//        std::vector<double> dist_to_goal(n);
//
//        std::vector<double> X(n);
//        std::vector<double> Y(n);
//
//        std::vector<double> lengths(n-1);
//
//        while(change > last_change + cost_optimization_tolerance) {
//            last_change = change;
//            change = 0;
//
//            for(unsigned i = 0; i < n; ++i) {
//                unsigned int x, y;
//                Pose2d pt_i = optimized_segment[i];
//                //map_info->Pose2d2cell(pt_i.x, pt_i.y, x, y);
//                X[i] = pt_i.x;
//                Y[i] = pt_i.y;
//            }
//
//            for(unsigned i = 0; i < n-1; ++i) {
//                lengths[i] = std::hypot(X[i] - X[i+1], Y[i] - Y[i+1]);
//            }
//
//            dist_to_start[0] = 0.0;
//            for(unsigned i = 0; i < n-1; ++i) {
//                dist_to_start[i+1] = dist_to_start[i] + lengths[i];
//            }
//
//            dist_to_goal[n-1] = 0.0;
//            for(unsigned i = n-1; i > 0; --i) {
//                dist_to_goal[i-1] = dist_to_goal[i] + lengths[i-1];
//            }
//
//            for(unsigned i = 0; i < n; ++i){
//                unsigned int x = X[i];
//                unsigned int y = Y[i];
//                int grad_x = gx.at<float>(y, x);
//                int grad_y = gy.at<float>(y, x);
//                double magnitude = hypot(grad_x, grad_y) / 255.0;
//                gradients_x[i] = grad_x;
//                gradients_y[i] = grad_y;
//                magnitudes[i] = magnitude;
//            }
//
//            for(unsigned i = offset; i < n-offset; ++i){
//                Pose2d path_i = segment[i];
//                Pose2d new_path_i = optimized_segment[i];
//                Pose2d new_path_ip1 = optimized_segment[i+1];
//                Pose2d new_path_im1 = optimized_segment[i-1];
//
//                double dist_border = std::min(dist_to_start[i], dist_to_goal[i]);
//                double border_damp = 1.0 + 5.0 / (0.01 + (dist_border / 5.0));
//
//                Pose2d deltaData = border_damp * cost_optimization_weight_data * (path_i - new_path_i);
//                new_path_i = new_path_i + deltaData;
//                double grad_x = gradients_x[i];
//                double grad_y = gradients_y[i];
//                //Pose2d deltaCost;
//                if(std::abs(grad_x) > 1e-3 && std::abs(grad_y) > 1e-3) {
//                    double magnitude = 0;
//                    int width = 0;
//                    for(int o = -offset+1;
//                        o <= offset-1; ++o) {
//                        magnitude += magnitudes[i+o];
//                        ++width;
//                    }
//
//                    magnitude /= width;
//
//                    Pose2d grad(-grad_x, -grad_y);
//                    //grad.x = -grad_x;
//                    //grad_y = -grad_y;
//                    Pose2d deltaCost =  cost_optimization_weight_cost * magnitude * grad;
//                    new_path_i = new_path_i + deltaCost;
//                }
//
//                Pose2d deltaSmooth =  cost_optimization_weight_smooth * (new_path_ip1 + new_path_im1 - 2* new_path_i);
//                new_path_i = new_path_i + deltaSmooth;
//
//                optimized_segment[i].x = new_path_i.x;
//                optimized_segment[i].y = new_path_i.y;
//
//                change += deltaData.distance_to_origin()
//                        + deltaSmooth.distance_to_origin()
//                        + deltaCost.distance_to_origin();
//            }
//        }
//
//        segment = optimized_segment;
//    }
//
//    return new_path;
//}
void PathPlannerNode::subdividePath(std::vector<Pose2d> &result, std::vector<Pose2d> low, std::vector<Pose2d> up, double max_distance) {
    double dx = low.back().x - up.back().x;
    double dy = low.back().y - up.back().y;
    double distance = std::sqrt(dx*dx + dy*dy);

    if(distance > max_distance) {
        // split half way between the lower and the upper node
        std::vector<Pose2d> halfway;
        halfway.push_back(low.back());
        halfway.back().x += (up.back().x - low.back().x) / 2.0;
        halfway.back().y += (up.back().y - low.back().y) / 2.0;
        // first recursive descent in lower part
        subdividePath(result, low, halfway, max_distance);
        // then add the half way Pose2d
        result.push_back(halfway.back());
        // then descent in upper part
        subdividePath(result, halfway, up, max_distance);
    }
}
std::vector<Pose2d> PathPlannerNode::interpolatePath(const std::vector<Pose2d> &path_raw, double max_distance)
{
    std::vector<Pose2d> result;
    std::vector<Pose2d> segment;
        //result.emplace_back();
        unsigned n = 1;

        segment = path_raw;

        for(unsigned i = 0; i < n; ++i){
            const std::vector<Pose2d> current = path_raw;

            // split the segment, iff it is to large
            subdividePath(segment, segment, current, max_distance);

            // add the end of the segment (is not done, when splitting)
            segment = current;
        }


    //return result;
    return segment;
}

std::vector<Pose2d> PathPlannerNode::smoothPathSegment( std::vector<Pose2d> path, double weight_data, double weight_smooth, double tolerance=1e-5)
{
    std::vector<Pose2d> new_path = path;
    std::vector<Pose2d> result_path;
    int n = path.size();

    double last_change = -2 * tolerance;
    double change = 0;

    int offset = 2;

    while(change > last_change + tolerance) {
        ROS_INFO("CHANGING PATH");
        ROS_INFO("%f", change);
        last_change = change;
        change = 0;

        for(int i = offset; i < n-offset; ++i){
            Pose2d path_i = path[i];
            Pose2d new_path_i = new_path[i];
            Pose2d new_path_ip1 = new_path[i+1];
            Pose2d new_path_im1 = new_path[i-1];

            Pose2d deltaData = (path_i - new_path_i) * weight_data;
            new_path_i = new_path_i + deltaData;

            Pose2d deltaSmooth = (new_path_ip1 + new_path_im1 - new_path_i*2)*weight_smooth;
            new_path_i = new_path_i + deltaSmooth;

            new_path[i].x = new_path_i.x;
            new_path[i].y = new_path_i.y;
            result_path.push_back(new_path[i]);
//            ROS_INFO("%i", deltaSmooth.x);
//            ROS_INFO("%i", deltaData.x);

            change += sqrt(deltaData.x*deltaData.x+deltaData.y*deltaData.y) + sqrt(deltaSmooth.x*deltaSmooth.x+deltaSmooth.y*deltaSmooth.y);
        }
    }

    // update orientations
    Pose2d current = new_path[0];
    Pose2d next = new_path[1];

    double dx = next.x - current.x;
    double dy = next.y - current.y;

    Eigen::Vector2d looking_dir_normalized(1, 0);
    Eigen::Vector2d delta(dx, dy);
    //const double theta_diff = std::acos((delta.x*looking_dir_normalized[0] + delta.y + looking_dir_normalized[1])/delta.norm() );

    // decide whether to drive forward or backward
    //bool is_backward = (theta_diff > M_PI_2 || theta_diff < -M_PI_2) ;

    for(int i = 1; i < n-1; ++i){
        Pose2d next = new_path[i+1];
        Pose2d prev = new_path[i-1];

        Pose2d delta = next - prev;
        double angle = std::atan2(delta.y, delta.x);

//        if(is_backward) {
//          //  angle = MathHelper::AngleClamp(angle + M_PI);
//        }

    }

    return new_path;
}

//std::vector<Pose2d> PathPlannerNode::smoothPath(const std::vector<Pose2d>& path_raw, double weight_data, double weight_smooth, double tolerance) {
//    std::vector<Pose2d> result;
//
//    for(const std::vector<Pose2d>& path : path_raw) {
//        std::vector<Pose2d> smoothed_segment = smoothPathSegment(path, weight_data, weight_smooth, tolerance);
//        result.push_back(smoothed_segment);
//    }
//    return result;
//}

std::vector<Pose2d> PathPlannerNode::postprocess(const std::vector<Pose2d>& path, std::vector<uint8_t> cost_map, std::vector<Point> segment_govno)
{
    ROS_INFO("postprocessing");
    std::vector<Pose2d> working_copy = path;

    std::vector<Pose2d> simplified_path = simplifyPath(working_copy, cost_map, segment_govno);
    ROS_INFO("next Step");
    //    ROS_INFO_STREAM("simplifying took " << sw.msElapsed() << "ms");

//    if(post_process_optimize_cost_) {
//        working_copy = optimizePathCost(working_copy, cost_map);
//    }
    std::vector<Pose2d> interpolated_path = interpolatePath(simplified_path, 0.5);
    std::vector<Pose2d> smoothed_path = smoothPathSegment(simplified_path, 0.6, 0.15);
    std::vector<Pose2d> final_interpolated_path = interpolatePath(smoothed_path, 0.1);
    std::vector<Pose2d> final_smoothed_path = smoothPathSegment(smoothed_path, 2.0, 0.4);
    return final_smoothed_path;
}

void PathPlannerNode::responseCallback(const std_msgs::String& response) {
 is_path_received = false;
 ROS_INFO("RESPONSE CALLBACK");
 if (response.data == "received")
    is_path_received = true;
  else if (response.data == "finished")
    is_move = false;
}
void PathPlannerNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
  map.width = map_msg->info.width;
  map.height = map_msg->info.height;
  resolution = map_msg->info.resolution;
  tf::poseMsgToTF(map_msg->info.origin, origin_tf);
  pre_process = true;
  post_process = true;
  map.mapSize = map.width * map.height;
  ROS_INFO("Map callback %d", map.mapSize);
  map.data.reset(new bool[map.mapSize]);
  for (int i = 0; i < map.mapSize; i++) {
    if (map_msg->data[i] < 50) {
      map.data[i] = true;
    } else {
      map.data[i] = false;
    }
  }
  is_map_init = true;
}

void PathPlannerNode::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg) {
  goal_position = goal_msg->pose;
  ROS_INFO("Goal callback");
  pre_process = true;

  startMoving();
}

void PathPlannerNode::startMoving() {
  ROS_INFO("Path planner start move");
  if (is_move) {
    terminateMoving();
  }
  move_timer = n.createTimer(ros::Duration(1), boost::bind(&PathPlannerNode::pathPlannerTimerCallback, this, _1));
  move_timer.start();

}

void PathPlannerNode::terminateMoving() {
  ROS_INFO("Path planner terminate move");
  std_msgs::String cmd;
  cmd.data = cmd_id + " terminate";
  move_cmd_publisher.publish(cmd);
  move_timer.stop();
  is_move = false;
}

void PathPlannerNode::pathPlannerTimerCallback(const ros::TimerEvent&) {
  if (!is_map_init) {
    return;
  }
  ROS_INFO("------------------------------------");
  ROS_INFO("-------Path planner iteration-------");
  getPosition();
  tf::Pose goal_tf;
  tf::Pose start_tf;
  tf::poseMsgToTF(goal_position, goal_tf);
  tf::poseMsgToTF(robot_position, start_tf);
  Point start(static_cast<int>(start_tf.getOrigin()[0] / resolution),
              static_cast<int>(start_tf.getOrigin()[1] / resolution));
  Point goal(static_cast<int>(goal_tf.getOrigin()[0] / resolution),
             static_cast<int>(goal_tf.getOrigin()[1] / resolution));
  vector<Point> path;
  path.clear();
  ROS_INFO("Start Pose2d is    %d, %d", start.x, start.y);
  ROS_INFO("Goal  Pose2d is    %d, %d", goal.x, goal.y);
  cv::Mat map_cv;
  std::vector<uint8_t> map_data;
  if (pre_process)
  {
    ROS_INFO("START PREPORC");
    int h = map.height;
    int w = map.width;
    for (int i = 0; i < h*w; i++)
     {
      if (map.data[i] == true)
        map_data.push_back(100);
      else
        map_data.push_back(0);
     }
//    map_data.resize(h, w);
    cv::Mat map_cv(h, w, CV_8UC1);

    memcpy(map_cv.data, map_data.data(), map_data.size() * sizeof(uint8_t));
//    map_cv = 1 - map_cv;
    cv::Mat distance;
    cv::distanceTransform(map_cv, distance, CV_DIST_L2, CV_DIST_MASK_PRECISE);
    cv::imwrite("costmap.png", distance);
    double scale = 10000.0;
    double max_distance_meters = 0.1;
//    double factor = (scale * resolution / max_distance_meters);
    distance.convertTo(map_cv, CV_8UC1);
    cv::Mat dst;
    cv::threshold(map_cv, dst, 10, 255, 0);
    cv::imwrite("nbdkdjf.png", dst);
    map_data.assign(map_cv.begin<uint8_t>(), map_cv.end<uint8_t>());
    for (int i= 0; i < h*w; i++)
    {
      if (map_data[i] < 10)
        map.data[i] = false;
      else
        map.data[i] = true;
    }
    pre_process = false;
  }

  if (planner.makePlan(map, start, goal, path)) {
    nav_msgs::Path ros_path;
    nav_msgs::Path ros_path_;
    //Pose2d poses;
    cvtPathToROS(path, ros_path);
    std::vector<Pose2d> final_path;
        std::vector<Pose2d> final_path_;
        path_publisher2.publish<nav_msgs::Path>(ros_path);
    cvtRosToPose2d(final_path, path, ros_path);
    final_path_ = PathPlannerNode::postprocess(final_path, map_data, path);
    cvtPathToROS(final_path_, ros_path_);
    path_publisher.publish<nav_msgs::Path>(ros_path_);
  cmd_id = std::to_string(next_cmd_id);
  next_cmd_id++;
  std_msgs::String cmd;
  cmd.data = cmd_id + " trajectory_following 0.4 0.4 0";
  //sleep(10);
  std::cout << is_path_received << std::endl;
  if ((is_path_received) && (!is_move))
  {
    ROS_INFO("--------COMMAND SENDING-----");
    std_msgs::String cmd;
    cmd.data = "11 trajectory_following 0.4 0.4 0";
    move_cmd_publisher.publish(cmd);
    is_move = true;
  }
  }
}

void PathPlannerNode::getPosition() {
  try {
    auto transform_stamped = tf_buffer.lookupTransform("map", "secondary_robot", ros::Time(0));
    tf::Transform transform;
    tf::transformMsgToTF(transform_stamped.transform, transform);
    tf::poseTFToMsg(transform, robot_position);
  } catch (tf2::TransformException& ex) {
    ROS_WARN("Could not lookup pathplanner to tf2. %s", ex.what());
  }
}

void PathPlannerNode::cvtPathToROS(vector<Point>& path, nav_msgs::Path& ros_path) {
  ros_path.header.frame_id = "map";
  ros_path.header.stamp = ros::Time::now();
  for (int i = 0; i < path.size(); i++) {
    geometry_msgs::Pose pose;
    tf::Pose pose_tf;
    pose_tf.setIdentity();
    pose_tf.setOrigin(tf::Vector3(resolution * path[i].x, resolution * path[i].y, 0));
    tf::poseTFToMsg(pose_tf, pose);
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    ros_path.poses.push_back(pose_stamped);
  }
}
void PathPlannerNode::cvtPathToROS(vector<Pose2d>& path, nav_msgs::Path& ros_path) {
  ros_path.header.frame_id = "map";
  ros_path.header.stamp = ros::Time::now();

  tf::Quaternion q_start(robot_position.orientation.x,
                        robot_position.orientation.y,
                        robot_position.orientation.z,
                        robot_position.orientation.w);
  tf::Matrix3x3 mat(q_start);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  double start = yaw;
  tf::Quaternion q_goal(goal_position.orientation.x,
                        goal_position.orientation.y,
                        goal_position.orientation.z,
                        goal_position.orientation.w);
  tf::Matrix3x3 mat_(q_goal);
  double roll_, pitch_, yaw_;
  mat_.getRPY(roll_, pitch_, yaw_);
  double goal = yaw_;
  int num = path.size();
  std::vector<double> angles = linspace(start, goal, num);
  for (int i = 0; i < path.size(); i++) {
    geometry_msgs::Pose pose;
    tf::Quaternion q;
    q.setRPY(0, 0, angles[i]);
    tf::Pose pose_tf;
    pose_tf.setIdentity();
    pose_tf.setOrigin(tf::Vector3(path[i].x, path[i].y, 0));
    tf::poseTFToMsg(pose_tf, pose);
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    pose_stamped.pose.orientation.x = q[0];
    pose_stamped.pose.orientation.y = q[1];
    pose_stamped.pose.orientation.z = q[2];
    pose_stamped.pose.orientation.w = q[3];
    ros_path.poses.push_back(pose_stamped);

//   for (int i =0 ; i < path.size(); i++)
//        std::cout << angles[i] << std::endl;
  }
}
std::vector<double> PathPlannerNode::linspace(double start_in, double end_in, int num_in)
{

  std::vector<double> linspaced;

  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in) - static_cast<double>(start_in);
  end = fmod((end + Pi),Pi_2) - Pi;
  double num = static_cast<double>(num_in);
  double start_ls = 0;
  if (num == 0) { return linspaced; }
  if (num == 1)
    {
      linspaced.push_back(start);
      return linspaced;
    }

  double delta = (end_in - start_in) / (num - 1);
  double angle;
  for(int i=0; i < num-1; ++i)
    {
      angle = fmod((delta * i + Pi), Pi_2) - Pi + start_in;
      linspaced.push_back(angle);
        //linspaced.push_back((start + delta * i);
    }
  linspaced.push_back(end_in); // I want to ensure that start and end
  //std::reverse(linspaced.begin(), linspaced.end());
  return linspaced;
}
void PathPlannerNode::cvtRosToPose2d(std::vector<Pose2d> &final_path, std::vector<Point>& path, nav_msgs::Path& ros_path)
{
   for (int i = 0; i < path.size(); i++) {
        double x = ros_path.poses[i].pose.position.x;
        double y = ros_path.poses[i].pose.position.y;
        Pose2d p(x, y);
        final_path.push_back(p);
        }
}
}  // namespace PathPlanner



int main(int argc, char** argv) {
  ros::init(argc, argv, "path_planner_node");
  PathPlanner::PathPlannerNode node;
  ros::spin();
  return 0;
}