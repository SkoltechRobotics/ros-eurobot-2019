#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include "Astar.h"
#include <string>
#include <opencv2/opencv.hpp>

using namespace Astar_planner;
namespace PathPlanner {

class PathPlannerNode {
 public:
  PathPlannerNode();
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg);
  void cmdCallback(const std_msgs::String& cmd);
  void responseCallback(const std_msgs::String& response);
  void pathPlannerTimerCallback(const ros::TimerEvent&);
  void startMoving();
  void terminateMoving();

 private:
  int next_cmd_id;
  string cmd_id;
  bool pre_process;
  bool post_process;
  AstarPlanner planner;
  ros::Timer move_timer = ros::Timer();
  ros::Publisher path_publisher;
  ros::Publisher path_publisher2;

  ros::Publisher move_cmd_publisher;
  nav_msgs::Path smoothPath(const nav_msgs::Path& path, double weight_data, double weight_smooth, double tolerance);
  nav_msgs::Path smoothPathSegment(const nav_msgs::Path& path, double weight_data, double weight_smooth, double tolerance);
  void getPosition();
  void calculateGradient(cv::Mat &gx, cv::Mat &gy, cv::Mat cost_mat);
  std::vector<Pose2d> optimizePathCost(const std::vector<Pose2d>& path_raw, cv::Mat cost_mat);
  void subdividePath(std::vector<Pose2d> &result, std::vector<Pose2d> low, std::vector<Pose2d> up, double max_distance);
  std::vector<Pose2d> interpolatePath(const std::vector<Pose2d> &path_raw, double max_distance);
  std::vector<Pose2d> smoothPathSegment( std::vector<Pose2d> path, double weight_data, double weight_smooth, double tolerance);
  std::vector<Pose2d> smoothPath(const std::vector<Pose2d>& path_raw, double weight_data, double weight_smooth, double tolerance);
  std::vector<Pose2d> postprocess(const std::vector<Pose2d>& path, std::vector<uint8_t> cost_map, std::vector<Point> segment_govno);
  void cvtPathToROS(vector<Point>& path, nav_msgs::Path& ros_path);
  void cvtRosToPose2d(std::vector<Pose2d> &final_path, std::vector<Point>& path, nav_msgs::Path& ros_path);
  void cvtPathToROS(vector<Pose2d>& path, nav_msgs::Path& ros_path);
  bool isFree(std::vector<uint8_t> cost_map, Pose2d segment_prev, Pose2d segment_next, Point segment_govno);
  std::vector<double> linspace(double start_in, double end_in, int num_in);
  std::vector<Pose2d> simplifyPath(const std::vector<Pose2d> segment, std::vector<uint8_t> cost_map, std::vector<Point> segment_govno);
  OccupancyGrid map;
  geometry_msgs::Pose goal_position, robot_position;
  ros::NodeHandle n;
  ros::Subscriber map_sub;
  ros::Subscriber goal_sub;
  ros::Subscriber move_response_sub;
  bool is_sending_path = false;
  bool post;
  tf2_ros::Buffer tf_buffer;
  boost::shared_ptr<tf2_ros::TransformListener> tfListener;
  float cost_optimization_tolerance = 1e-5;
  float cost_optimization_weight_data = 0.9;
  float cost_optimization_weight_smooth = 0.3;
  std::string world_frame, base_frame;
  bool is_map_init = false;
  bool is_move = false;
  double Pi = 3.141592653589793;
  double Pi_2 = 6.283185307179586;
  bool is_path_received = false;
  tf::Pose origin_tf;
  float resolution;
};
}; // namespace PathPlanner