/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW1_CLASS_H_
#define CW1_CLASS_H_

// system includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// include PCL related package (inspired by PCL demo repo)
// standard library required by PCL
#include <stdlib.h>
#include <iostream>
#include <cmath>

// other libraries required by PCL
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h> 
#include <pcl/filters/filter_indices.h> 
#include "pcl/segmentation/region_growing_rgb.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/conditional_removal.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_normal_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>

// standard c++ library includes (std::string, std::vector)
#include <string>
#include <vector>

// segmentation clustering library
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_cloud.h> 
#include <pcl/point_types.h> 
#include <pcl/segmentation/extract_clusters.h> 
#include <pcl/filters/extract_indices.h>

// include services from the spawner package - we will be responding to these
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"

// // include any services created in this package
// #include "cw1_team_3/example.h"

#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

//PCL data type definition
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;


class cw1
{
public:

  /* ----- class member functions ----- */

  /** \brief Empty constructor.
    *
    * \input[in] nh the ROS node
    */
  cw1 (ros::NodeHandle &nh);

  /** \brief MoveIt function for moving the move_group to the target position.
    *
    * \input[in] target_pose: pose to move the arm to
    *
    * \return true if moved to target position 
    */
  bool 
  moveArm(geometry_msgs::Pose target_pose);

  /** \brief MoveIt function for moving the gripper fingers to a new position. 
    *
    * \input[in] width: desired gripper finger width
    *
    * \return true if gripper fingers are moved to the new position
    */
  bool 
  moveGripper(float width);

  /** \brief MoveIt function for adding a cuboid collision object in RViz
    * and the MoveIt planning scene.
    *
    * \input[in] object_name: name for the new object to be added
    * \input[in] centre point: at which to add the new object
    * \input[in] dimensions: dimensions of the cuboid to add in x,y,z
    * \input[in] orientation: rotation to apply to the cuboid before adding
    */
  void
  addCollisionObject(std::string object_name, geometry_msgs::Point centre, 
    geometry_msgs::Vector3 dimensions, geometry_msgs::Quaternion orientation);

  /** \brief Remove a collision object from the planning scene.
    * 
    * \input[in] object_name: for the object to be removed
    */
  void 
  removeCollisionObject(std::string object_name);

  /** \brief Pick an object up with a given position and orientation.
    * 
    * \input[in] position: the xyz coordinates where the gripper converges
    * \input[in] orientation: the xyzw quaternion orientation where the gripper converges
    */
  bool
  pick_precise (geometry_msgs::Point position,  geometry_msgs::Quaternion object_orientation);

  /** \brief Place an object at a given position.
    * 
    * \input[in] position the xyz coordinates where the gripper converges
    * 
    */
  bool
  place(geometry_msgs::Point position);

  /** \brief Transfrom point cloud points from in_frame_name to out_frame_name.
    * 
    * \input[in] in_frame_name input frame name to transfrom from
    * \input[in] out_frame_name output frame name to transfrom to
    * \input[in] in_cloud_ptr input point cloud to transform from
    * \input[in] out_cloud_ptr output point cloud to transform to
    */
  void
  transform_to_frame(std::string in_frame_name, std::string out_frame_name,
                     PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);

  /** \brief Extract clusters with given cluster size range.
    * 
    * \input[in] cluster_size_min minimum cluster size
    * \input[in] cluster_size_max maximum cluster size
    * \input[in] in_cloud_ptr input point cloud to extract
    * \input[in] out_clusters output a point clouds vector
    */
  void
  extract_clusters(int cluster_size_min, int cluster_size_max, 
                    PointCPtr &in_cloud_ptr, std::vector<PointCPtr> &out_clusters);

  void
  find_orientation (const pcl::PointCloud<pcl::PointXYZRGBA> &single_cluster, double centre_x, double centre_y);

  /** \brief Performing picking-placing tasks specified in Task 3.
    * 
    * \input[in] box_clusters box cluster point cloud
    * \input[in] basket_clusters basket cluster point cloud
    * \input[in] box_count box counts in corresponding baskets
    */
  void
  pickAndPlacing(std::vector<PointCPtr> box_clusters, 
                  std::vector<PointCPtr> basket_clusters, 
                  std::vector<int> box_counts);

  /** \brief Compute string colour of the point cloud.
    * 
    * \input[in] in_cloud_ptr the input PointCloud2 pointer
    */
  std::string
  pointCloudColourToString(PointCPtr &in_cloud_ptr);

  /** \brief Point Cloud CallBack function for Task 2.
    * 
    * \input[in] cloud_input a PointCloud2 sensor_msgs const pointer
    */
  void 
  cloud_callback_task2(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);

  /** \brief Point Cloud CallBack function for Task 3.
    * 
    * \input[in] cloud_input a PointCloud2 sensor_msgs const pointer
    */
  void 
  cloud_callback_task3(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);

  /** \brief Apply Pass Through filtering.
  * 
  * \input[in] in_cloud_ptr the input PointCloud2 pointer
  * \input[out] out_cloud_ptr the output PointCloud2 pointer
  */
  void 
  applyPT_task2 (PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);

  /** \brief Apply Pass Through filtering for Task 3.
  * 
  * \input[in] in_cloud_ptr the input PointCloud2 pointer
  * \input[out] out_cloud_ptr the output PointCloud2 pointer
  */
  void
  applyPT_task3 (PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);
    


  /* ----- Variables ----- */

  /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
  * these are defined in urdf. */
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

  /** \brief MoveIt interface to interact with the moveit planning scene 
    * (eg collision objects). */
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  /** \brief Pass Through filter. */
  pcl::PassThrough<PointT> g_pt;

  /** \brief Point Cloud (input). */
  pcl::PCLPointCloud2 g_PCLPointCloud2;

  /** \brief Point Cloud (input) pointer. */
  PointCPtr g_PCLPointCloudRGBA;

  /** \brief Point Cloud (filtered) pointer. */
  PointCPtr g_cloud_filtered;

  /**\brief KDTree for nearest neighborhood search. */
  pcl::search::KdTree<PointT>::Ptr g_tree_ptr;

  /** \brief The input point cloud frame id. */
  std::string g_cloud_frame_id_;

  /** \brief Result colour list for Task 2. */
  std::vector<std::string> g_colour_list;

  /** \brief basket clusters for Task 3. */
  std::vector<PointCPtr> g_basket_clusters;

  /** \brief box clusters for Task 3. */
  std::vector<PointCPtr> g_box_clusters;

  /** \brief yaw angles of box clusters for Task 3. */
  double box_angle;
  
  // service callbacks for tasks 1, 2, and 3
  bool 
  t1_callback(cw1_world_spawner::Task1Service::Request &request,
    cw1_world_spawner::Task1Service::Response &response);
  bool 
  t2_callback(cw1_world_spawner::Task2Service::Request &request,
    cw1_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw1_world_spawner::Task3Service::Request &request,
    cw1_world_spawner::Task3Service::Response &response);

  /* ----- class member variables ----- */

  /** \brief Node handle. */
  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;


protected:
  /** \brief Debug mode. */
  bool debug_;
};


#endif // end of include guard for CW1_CLASS_H_
