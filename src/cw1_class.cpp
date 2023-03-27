/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include <cw1_class.h>
#include <config.h>

// initialize a point cloud cluster viewer for visualisation
pcl::visualization::CloudViewer viewer ("Cluster viewer");


///////////////////////////////////////////////////////////////////////////////

// function to unsubscribe the point cloud callback topic
void
unsubscribeFromPointCloudTopic ()
{
  pc_sub.shutdown();
}

///////////////////////////////////////////////////////////////////////////////

cw1::cw1 (ros::NodeHandle &nh):
  g_PCLPointCloudRGBA (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  g_tree_ptr (new pcl::search::KdTree<PointT> ()), // KdTree
  debug_ (false)
{
  /* class constructor */

  nh_ = nh;

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw1::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw1::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw1::t3_callback, this);

  // namespace for our ROS services,
  // they will appear as "/namespace/srv_name"
  std::string service_ns = "/cw1_team_3";

  ROS_INFO("cw1 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t1_callback (cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response)
{
  /* function which should solve task 2 */
  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  ROS_INFO_STREAM("The request message is" << request);

  // Initialisation for picking and placing position
  geometry_msgs::Point pick_position;
  geometry_msgs::Point place_position;

  // Extract the position information from the request message
  pick_position.x = request.object_loc.pose.position.x;
  pick_position.y = request.object_loc.pose.position.y;
  pick_position.z = request.object_loc.pose.position.z;

   // Extract the position information from the request message
  place_position.x = request.goal_loc.point.x;
  place_position.y = request.goal_loc.point.y;
  place_position.z = request.goal_loc.point.z;

  // Log the extracted coordinate information
  ROS_INFO("Received place position: (%.2f, %.2f, %.2f)", 
    place_position.x, place_position.y, place_position.z);

  // Log the extracted coordinate information
  ROS_INFO("Received position: (%.2f, %.2f, %.2f)", 
    pick_position.x, pick_position.y, pick_position.z);

  // Define the collision object name, for the box region
  std::string basket = "collision_object";
  std::string tile = "collision_object_tile";

  // Define the object centre point
  geometry_msgs::Point centre;
  centre.x = place_position.x;
  centre.y = place_position.y;
  centre.z = place_position.z + 0.02;

  // Define the tile centre point
  geometry_msgs::Point tile_centre;
  tile_centre.x = 0.45;
  tile_centre.y = 0;
  tile_centre.z = 0;

  // Define the dimensions of the object, 0.1 is the length of the box
  geometry_msgs::Vector3 dimensions;
  dimensions.x = 0.1;
  dimensions.y = 0.1;
  dimensions.z = 0.1;

  // Define the dimensions of the tile, 0.1 is the length of the box
  geometry_msgs::Vector3 tile_dimensions;
  tile_dimensions.x = 0.50;
  tile_dimensions.y = 1.36;
  tile_dimensions.z = 0.02;

  // Define the orientation of the object
  geometry_msgs::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = 0.0;
  orientation.w = 1.0;

 // Define the orientation of the tile
  geometry_msgs::Quaternion collision_orientation;
  collision_orientation.x = 0.0;
  collision_orientation.y = 0.0;
  collision_orientation.z = 0.0;
  collision_orientation.w = 1.0;

  // Define the orientation of the object
  geometry_msgs::Quaternion object_orientation;
  object_orientation.x = request.object_loc.pose.orientation.x;
  object_orientation.y = request.object_loc.pose.orientation.y;
  object_orientation.z = request.object_loc.pose.orientation.z;
  object_orientation.w = request.object_loc.pose.orientation.w; 

  // Add the object to RViz and the MoveIt planning scene
  addCollisionObject(basket, centre, dimensions, object_orientation);

  // Add the tile to RViz and the MoveIt planning scene
  addCollisionObject(tile, tile_centre, tile_dimensions, collision_orientation);

  // Perform picking
  bool success = pick_precise(pick_position, object_orientation);

  ROS_INFO("the cube is picked up succesfully");

  // Perform placing
  success = place(place_position);

  ROS_INFO("the cube is placed in the basket succesfully");

  // Remove collision object and tile
  removeCollisionObject(basket);
  removeCollisionObject(tile);

  ROS_INFO("the collision object is removed for next task planning");

  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  return success;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t2_callback (cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */
  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  // determine the placing orientation
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, ANGLE_OFFSET);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion place_orientation = tf2::toMsg(q_result);

  // set the desired pre-grasping pose
  geometry_msgs::Pose approach_pose;
  approach_pose.orientation = place_orientation;

  // add tile collision
  // Define the collision object name, for the box region
  std::string tile = "collision_object_tile";

  // Define the tile centre point
  geometry_msgs::Point tile_centre;
  tile_centre.x = 0.45;
  tile_centre.y = 0;
  tile_centre.z = 0;

  // Define the dimensions of the tile, 0.1 is the length of the box
  geometry_msgs::Vector3 tile_dimensions;
  tile_dimensions.x = 0.50;
  tile_dimensions.y = 1.36;
  tile_dimensions.z = 0.022;

 // Define the orientation of the tile
  geometry_msgs::Quaternion collision_orientation;
  collision_orientation.x = 0.0;
  collision_orientation.y = 0.0;
  collision_orientation.z = 0.0;
  collision_orientation.w = 1.0;

  // Add the tile to RViz and the MoveIt planning scene
  addCollisionObject(tile, tile_centre, tile_dimensions, collision_orientation);

  // Add the object to RViz and the MoveIt planning scene
  for (int i = 0; i < request.basket_locs.size(); i++)
  {
    // get the locations to move to and perform moving
    approach_pose.position.x = request.basket_locs[i].point.x; 
    approach_pose.position.y = request.basket_locs[i].point.y;

    // add collision object (each basket)
    // Define the collision object name, for the box region
    std::string basket = "collision_object/" + std::to_string(i);

    // Define the centre point
    geometry_msgs::Point centre;
    centre.x = approach_pose.position.x;
    centre.y = approach_pose.position.y;
    centre.z = 0.07;

    // Define the dimensions of the object, 0.1 is the length of the basket
    geometry_msgs::Vector3 dimensions;
    dimensions.x = 0.1;
    dimensions.y = 0.1;
    dimensions.z = 0.1;


    // Define the orientation of the object
    geometry_msgs::Quaternion orientation;
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = 0.0;
    orientation.w = 1.0;

    // Add the collision object to RViz and the MoveIt planning scene
    addCollisionObject(basket, centre, dimensions, orientation);
  }

  // move the Arm to specific locations
  for (int i = 0; i < request.basket_locs.size(); i++)
  {
    // get the locations to move to and perform moving
    approach_pose.position.x = request.basket_locs[i].point.x; 
    approach_pose.position.y = request.basket_locs[i].point.y;

    // make it higher to take the whole scene in
    approach_pose.position.z = 0.4; 

    // Perform moving
    moveArm(approach_pose);

    ros::Duration(1.0).sleep();

    // Subscribe to the point cloud generation topic, and
    // callback the point cloud method
    pc_sub = nh_.subscribe("/r200/camera/depth_registered/points", 4,
      &cw1::cloud_callback_task2, this);
  }

  ros::Duration(10.0).sleep();

  ROS_INFO_STREAM("task 2 finished");

  // remove collision object
  for (int i = 0; i < request.basket_locs.size(); i++)
  {
    // Define the collision object name, for the box region
    std::string basket = "collision_object/" + std::to_string(i);
    removeCollisionObject(basket);
  }

  // remove collision tile
  removeCollisionObject(tile);

  // Populate result
  response.basket_colours = g_colour_list;

  ROS_INFO_STREAM("Output response is: " << response);

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t3_callback (cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");


  // determine the placing orientation
  tf2::Quaternion q_object;
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  q_object.setRPY(0, 0, ANGLE_OFFSET);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion place_orientation = tf2::toMsg(q_result);

  // Move arm to a higher place to scan objects
  geometry_msgs::Pose approach_pose;
  approach_pose.position.x = 0.45;
  approach_pose.position.y = 0;
  approach_pose.position.z = 0.83;
  approach_pose.orientation = place_orientation;

  // Perform moving
  moveArm(approach_pose);

  // Subscribe to the point cloud generation topic, and
  // callback the point cloud method
  pc_sub = nh_.subscribe("/r200/camera/depth_registered/points", 4,
    &cw1::cloud_callback_task3, this);

  ros::Duration(1.0).sleep();

  return true;
}

///////////////////////////////////////////////////////////////////////////////

void
cw1::addCollisionObject (std::string object_name,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation)
{
  /* add a collision object in RViz and the MoveIt planning scene */

  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input header information
  collision_object.id = object_name;
  collision_object.header.frame_id = BASE_FRAME;

  // define the primitive and its dimensions
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dimensions.x;
  collision_object.primitives[0].dimensions[1] = dimensions.y;
  collision_object.primitives[0].dimensions[2] = dimensions.z;

  // define the pose of the collision object
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = centre.x;
  collision_object.primitive_poses[0].position.y = centre.y;
  collision_object.primitive_poses[0].position.z = centre.z;
  collision_object.primitive_poses[0].orientation = orientation;

  // define that we will be adding this collision object 
  collision_object.operation = collision_object.ADD;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}

///////////////////////////////////////////////////////////////////////////////

void
cw1::removeCollisionObject (std::string object_name)
{
  /* remove a collision object from the planning scene */

  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input the name and specify we want it removed
  collision_object.id = object_name;
  collision_object.operation = collision_object.REMOVE;

  // apply this collision object removal to the scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::moveArm (geometry_msgs::Pose target_pose)
{
  /* This function moves the panda arm to the target position */

  // assign the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan arm_trajectory_plan;
  bool success = (arm_group_.plan(arm_trajectory_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::moveGripper (float width)
{
  /* this function closes or opens the gripper joints, The Joints are:
      - panda_finger_joint1
      - panda_finger_joint2
  */

  // for safety, assert and assign the width within the gripper open and closed ranges 
  if (width > GRIPPER_OPEN) width = GRIPPER_OPEN;
  if (width < GRIPPER_CLOSED) width = GRIPPER_CLOSED;

  // calculate the joint targets as half each of the requested distance
  double Joint_width = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = Joint_width;
  gripperJointTargets[1] = Joint_width;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan gripper_trajectory_plan;
  bool success = (hand_group_.plan(gripper_trajectory_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  hand_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::pick_precise (geometry_msgs::Point position,  
geometry_msgs::Quaternion object_orientation)
{
  /* This function picks up an object using a pose (orientation and position). 
  The given point is where the centre of the gripper fingers will converge */

  // define grasping as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);


  // determine the grasping orientation
  tf2::Quaternion q_offset;
  q_offset.setRPY(0, 0, ANGLE_OFFSET);

  // transform the grasping orientation

  // convert geometry_msgs::Quaternion to tf2::Quaternion
  tf2::Quaternion q_object;
  tf2::fromMsg(object_orientation, q_object);

  // perform the multiplication to determine the final grasp position
  tf2::Quaternion q_result = q_x180deg * q_offset * q_object;

  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  // set the desired grasping pose
  geometry_msgs::Pose grasp_pose;
  grasp_pose.position = position;
  grasp_pose.orientation = grasp_orientation;
  grasp_pose.position.z += Z_OFFSET;

  // set the desired pre-grasping pose
  geometry_msgs::Pose approach_pose;
  approach_pose = grasp_pose;
  approach_pose.position.z += APPROACH_DISTANCE;

  /* Now perform the pick */

  bool success = true;

  ROS_INFO("Begining pick operation");

  // move the arm above the object
  success *= moveArm(approach_pose);

  if (not success) 
  {
    ROS_ERROR("Moving arm to pick approach pose failed");
    return false;
  }

  // open the gripper
  success *= moveGripper(GRIPPER_OPEN);

  if (not success) 
  {
    ROS_ERROR("Opening gripper prior to pick failed");
    return false;
  }

  // approach to grasping pose
  success *= moveArm(grasp_pose);

  if (not success) 
  {
    ROS_ERROR("Moving arm to grasping pose failed");
    return false;
  }

  // grasp!
  success *= moveGripper(GRIPPER_CLOSED);

  if (not success) 
  {
    ROS_ERROR("Closing gripper to grasp failed");
    return false;
  }

  // retreat with object
  success *= moveArm(approach_pose);

  if (not success) 
  {
    ROS_ERROR("Retreating arm after picking failed");
    return false;
  }

  ROS_INFO("Pick operation successful");

  return true;
}


///////////////////////////////////////////////////////////////////////////////

bool
cw1::place (geometry_msgs::Point position)
{
  /* follow up function to place the cube to the assigned position */ 

  // define placing position as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the placing orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, ANGLE_OFFSET);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion place_orientation = tf2::toMsg(q_result);

  // set the desired approach pose for dropping the cube from above
  geometry_msgs::Pose approach_pose;
  approach_pose.position = position;
  approach_pose.orientation = place_orientation;
  approach_pose.position.z += 0.3; // offset to ground to avoid collision

  /* Now performing the place operation of panda arm */

  bool success = true;

  ROS_INFO("Begining place operation");

  // move the arm above the object
  success *= moveArm(approach_pose);

  if (not success) 
  {
    ROS_ERROR("Moving arm to pick approach pose failed");
    return false;
  }

  // open the gripper
  success *= moveGripper(GRIPPER_OPEN);

  if (not success) 
  {
    ROS_ERROR("Opening gripper to drop the cube failed");
    return false;
  }


  ROS_INFO("Pick operation successful");

  return true;
    
}

///////////////////////////////////////////////////////////////////////////////

void
cw1::cloud_callback_task2
(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg)
{
  ros::Duration(2.0).sleep();

  ROS_INFO("PointCloud2 message received");

  // Extract input point cloud info
  g_cloud_frame_id_ = cloud_input_msg->header.frame_id;

  ROS_INFO("Task2 cloud frame ID: %s", g_cloud_frame_id_.c_str());

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_input_msg, g_PCLPointCloud2);

  // Convert to PCL Point Cloud2 data type
  pcl::fromPCLPointCloud2 (g_PCLPointCloud2, *g_PCLPointCloudRGBA);

  // Filter the data
  applyPT_task2 (g_PCLPointCloudRGBA, g_cloud_filtered);

  ros::Duration(3.0).sleep();

  viewer.showCloud(g_cloud_filtered);

  ros::Duration(3.0).sleep();

  unsubscribeFromPointCloudTopic();

  ROS_INFO("topic unsubsribed for the current step");

  // Compute colour based on RGB value
  // and add to colour list
  g_colour_list.push_back(pointCloudColourToString(g_cloud_filtered));

}

///////////////////////////////////////////////////////////////////////////////

std::string
cw1::pointCloudColourToString (PointCPtr &in_cloud_ptr)
{
  // Extract the RGB data from the input point cloud
  double red = 0, green = 0, blue = 0;
  double count = 0;
  for (PointT point : *in_cloud_ptr)
  {
    count++;
    red += point.r;
    green += point.g;
    blue += point.b;
  }

  // Compute RGB values
  red = (red / count);
  blue = (blue / count );
  green = (green / count);

  // Determine which colour it is and return its string format
  if (red > COLOUR_THRESHOLD && red > green && red > blue)
  {
    ROS_INFO("this is a red box");
    return "red";
  }
  else if (blue > COLOUR_THRESHOLD && blue > red && blue > green)
  {
    ROS_INFO("this is a blue box");
    return "blue";
  }
  else if (red > COLOUR_THRESHOLD && blue > COLOUR_THRESHOLD 
            && green < red && green < blue)
  {
    ROS_INFO("this is a purple box");
    return "purple";
  }
  else
  {
    ROS_INFO("this is no box (green)");
    return "empty";
  }
}

///////////////////////////////////////////////////////////////////////////////

void
cw1::applyPT_task2 (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  // filter out x coordinate
  g_pt.setInputCloud (in_cloud_ptr);
  g_pt.setFilterFieldName ("x");
  g_pt.setFilterLimits (-0.05, 0.05);
  g_pt.filter (*out_cloud_ptr);

  // filter out y coordinate
  g_pt.setInputCloud (out_cloud_ptr);
  g_pt.setFilterFieldName ("y");
  g_pt.setFilterLimits (0, 0.1);
  g_pt.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw1::transform_to_frame (std::string in_frame_name, std::string out_frame_name, 
PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr)
{
  // Create a TransformListener object
  tf::TransformListener listener;

  for (PointT point : *in_cloud_ptr)
  {
    // Initialise transformed result and current point
    geometry_msgs::PointStamped p_transformed;
    geometry_msgs::PointStamped p;

    p.header.frame_id = in_frame_name;
    // ROS_INFO_STREAM("x:" << point.x <<"y:" << point.y <<"z" << point.z);
    p.point.x = point.x;
    p.point.y = point.y;
    p.point.z = point.z;

    /* Perform transforming */
    try
    {
      listener.waitForTransform(out_frame_name, p.header.frame_id, 
                                  ros::Time(0), ros::Duration(3.0));
      listener.transformPoint(out_frame_name, p, p_transformed);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    // Recover RGBA point
    PointT transformed_point;
    transformed_point.x = p_transformed.point.x;
    transformed_point.y = p_transformed.point.y;
    transformed_point.z = p_transformed.point.z;
    transformed_point.rgba = point.rgba;

    // Append transformed points to output point cloud
    out_cloud_ptr->push_back(transformed_point);
  }
}

///////////////////////////////////////////////////////////////////////////////

void
cw1::extract_clusters (int cluster_size_min, int cluster_size_max, 
PointCPtr &in_cloud_ptr, std::vector<PointCPtr> &out_clusters)
{
  // Set up the euclidean cluster extraction object
  pcl::EuclideanClusterExtraction<PointT> ec;

  g_tree_ptr->setInputCloud (g_cloud_filtered);
  ec.setClusterTolerance (0.01);
  ec.setMinClusterSize (cluster_size_min); 
  ec.setMaxClusterSize (cluster_size_max);
  ec.setSearchMethod (g_tree_ptr);
  ec.setInputCloud (g_cloud_filtered);

  // Extract the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  ec.extract(cluster_indices);

  // Iterate through each cluster and process it
  for (std::vector<pcl::PointIndices>::const_iterator 
        it = cluster_indices.begin();
        it != cluster_indices.end (); ++it)
  {
    // Initialise cluster
    PointCPtr cluster (new PointC);

    //  Add each point in the cluster to a new point cloud
    for (std::vector<int>::const_iterator pit = it->indices.begin (); 
      pit != it->indices.end (); 
      ++pit)
    {
      cluster->points.push_back (g_cloud_filtered->points[*pit]); 
    }

    // Set cluster properties
    cluster->width = cluster->points.size ();
    cluster->height = 1;
    cluster->is_dense = true;

    // Append current cluster to output cluster list
    out_clusters.push_back(cluster);
  }
}

///////////////////////////////////////////////////////////////////////////////

void
cw1::cloud_callback_task3
(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg)
{
  // define placing position as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the placing orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, ANGLE_OFFSET);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion place_orientation = tf2::toMsg(q_result);

  // Create a TransformListener object
  // tf::TransformListener listener;
  PointCPtr transformed_cloud(new PointC);

  ros::Duration(2.0).sleep();

  ROS_INFO("PointCloud2 message received");

  // Extract input point cloud info
  g_cloud_frame_id_ = cloud_input_msg->header.frame_id;

  ROS_INFO("Task3 cloud frame ID: %s", g_cloud_frame_id_.c_str());

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_input_msg, g_PCLPointCloud2);

  // Convert to PCL Point Cloud2 data type
  pcl::fromPCLPointCloud2 (g_PCLPointCloud2, *g_PCLPointCloudRGBA);

  ros::Duration(3.0).sleep();

  // Transform camera frame to world frame
  transform_to_frame(g_cloud_frame_id_, WORLD_FRAME, 
                      g_PCLPointCloudRGBA, transformed_cloud);

  // Filter the green platform in the scene
  applyPT_task3 (transformed_cloud, g_cloud_filtered);

  // Show cloud
  viewer.showCloud(g_cloud_filtered);

  // Extract box clusters
  extract_clusters(BOX_CLUSTER_SIZE_MIN, BOX_CLUSTER_SIZE_MAX, 
                    g_cloud_filtered, g_box_clusters);

  // Extract basket clusters
  extract_clusters(BASKET_CLUSTER_SIZE_MIN, BASKET_CLUSTER_SIZE_MAX, 
                    g_cloud_filtered, g_basket_clusters);
  
  // Add tile collision 
  // Define the collision object name, for the box region
  std::string tile = "collision_object_tile";

  // Define the tile centre point
  geometry_msgs::Point tile_centre;
  tile_centre.x = 0.45;
  tile_centre.y = 0;
  tile_centre.z = 0;

  // Define the dimensions of the tile, 0.1 is the length of the box
  geometry_msgs::Vector3 tile_dimensions;
  tile_dimensions.x = 0.50;
  tile_dimensions.y = 1.36;
  tile_dimensions.z = 0.022;

 // Define the orientation of the tile
  geometry_msgs::Quaternion collision_orientation;
  collision_orientation.x = 0.0;
  collision_orientation.y = 0.0;
  collision_orientation.z = 0.0;
  collision_orientation.w = 1.0;

  // Add the tile to RViz and the MoveIt planning scene
  addCollisionObject(tile, tile_centre, tile_dimensions, collision_orientation);

  // create a vector to record box counts in the baskets 
  std::vector<int> box_counts = {};
  // add collision objects and a vector to record box counts in the baskets 
  for (int i = 0; i < g_basket_clusters.size(); i++)
  {
    // Define the collision object name, for the box region
    std::string basket = "collision_object/" + std::to_string(i);
    Eigen::Vector4f centroid_in;

    box_counts.push_back(0);
    // compute the box centroid
    pcl::compute3DCentroid(*g_basket_clusters[i], centroid_in);

    // Define the centre point
    geometry_msgs::Point centre;
    centre.x = centroid_in[0];
    centre.y = centroid_in[1];
    centre.z = 0.07;

    // Define the dimensions of the object, 0.1 is the length of the basket
    geometry_msgs::Vector3 dimensions;
    dimensions.x = 0.1;
    dimensions.y = 0.1;
    dimensions.z = 0.1;


    // Define the orientation of the object
    geometry_msgs::Quaternion orientation;
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = 0.0;
    orientation.w = 1.0;

    // Add the collision object to RViz and the MoveIt planning scene
    addCollisionObject(basket, centre, dimensions, orientation);
  }

  /* Performing Task 3 operations. */
  pickAndPlacing(g_box_clusters, g_basket_clusters, box_counts);

   // remove collision objects
  for (int i = 0; i < g_basket_clusters.size(); i++)
  {
    // Define the collision object name, for the box region
    std::string basket = "collision_object/" + std::to_string(i);
    removeCollisionObject(basket);
  }

  // remove collision objects tile
  removeCollisionObject(tile);
  
  ROS_INFO_STREAM("total number of basket cluster: " << g_basket_clusters.size());

  ROS_INFO_STREAM("total number of box cluster: " << g_box_clusters.size());

  ros::Duration(3.0).sleep();

  unsubscribeFromPointCloudTopic();

  ROS_INFO("topic unsubsribed for the current step");

}



////////////////////////////////////////////////////////////////////////////////

void
cw1::applyPT_task3 (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  // filter out z coordinate
  g_pt.setInputCloud (in_cloud_ptr);
  g_pt.setFilterFieldName ("z");
  g_pt.setFilterLimits (0.022, 1.3);
  g_pt.filter (*out_cloud_ptr);
  
  ROS_INFO("applyPT_task3 finished!");

  return;
}

////////////////////////////////////////////////////////////////////////////////
void
cw1::find_orientation (const pcl::PointCloud<pcl::PointXYZRGBA> &single_cluster, double centre_x, double centre_y) 
{
// find the maximum x val of a cluster 
double x_max = -1000;
double y_corresponding;


// // dereference the point cloud shared pointer to get a reference to the point cloud
// const pcl::PointCloud<pcl::PointXYZRGBA>& cloud = *single_cluster;

//  Iterate through the point clouds in a single cluster (i.e. cluster of each box)
for (int count = 0; 
  count < single_cluster.size(); 
  ++count)
{
  // substitute the x_max if a larger value of x is found 
  if (single_cluster[count].x > x_max) {
    x_max = single_cluster[count].x;
    y_corresponding = single_cluster[count].y;
  }
}

// assign the obtained tangent radian val to the public double radian val box_angle
// add a 45 degree offset to the orienation of picking
box_angle = atan2(y_corresponding - centre_y, x_max - centre_x) + 0.7854;

ROS_INFO_STREAM("angle in radians of the box orientation = " << box_angle);

}

///////////////////////////////////////////////////////////////////////////////

void 
cw1::pickAndPlacing (std::vector<PointCPtr> box_clusters, 
std::vector<PointCPtr> basket_clusters, std::vector<int> box_counts)
{
  // Iterate through box to place
  for (int i = 0; i < box_clusters.size(); i++)
  {
    bool find_basket = false; // check if box has place to be put
    std::string cur_box_colour = pointCloudColourToString(box_clusters[i]);

    // Iterate through basket to find the same colour basket-box pair
    for (int j = 0; j < basket_clusters.size(); j++)
    {
      std::string cur_basket_colour = pointCloudColourToString
                                        (basket_clusters[j]);

      // if colours are same
      if (cur_box_colour == cur_basket_colour)
      {
        // check if current basket has reach its capacity
        if (box_counts[j] >= BASKET_CAPACITY)
        {
          ROS_INFO_STREAM("Basket " << cur_basket_colour 
            << "has reached its capacity, looking for next "
            << cur_basket_colour << " basket");
          continue;
        }

        // if not, 
        // calculate starting point and destination to perform task
        geometry_msgs::Point starting_point;
        geometry_msgs::Point destination;
        Eigen::Vector4f centroid_box;
        Eigen::Vector4f centroid_basket;

        // compute the box centroid
        pcl::compute3DCentroid(*box_clusters[i], centroid_box);
        starting_point.x = centroid_box[0];
        starting_point.y = centroid_box[1];
        starting_point.z = 0.03;

        // compute the orientation of the box, box_cluster is a vector storing PointCPtr, dereference it
        // gives pcl::PointCloud<pcl::PointXYZRGBA>
        find_orientation (*box_clusters[i], centroid_box[0], centroid_box[1]);

        // compute the box orientation from the RPY angle
        tf2::Quaternion q_box;
        q_box.setRPY(0, 0, box_angle);

        // transform the tf2 orientation to geometry orientation
        geometry_msgs::Quaternion box_orientation = tf2::toMsg(q_box);
  
        // pick box using its precise centroid location and orientation
        pick_precise(starting_point, box_orientation);

        // compute the basket centroid
        pcl::compute3DCentroid(*basket_clusters[j], centroid_basket);
        destination.x = centroid_basket[0];
        destination.y = centroid_basket[1];
        destination.z = 0.1;

        // place to basket
        place(destination);

        // if this box is placed successfully, 
        // this basket's box_count +1, then break the inner loop
        find_basket = true;
        box_counts[j]++;
        break;
      }
    }

    // if current box can't find a box, there're two reasons:
    // 1. basket(s) with the same colour is(are) full.
    // 2. no basket with same colour with box.
    if (!find_basket)
    {
      ROS_INFO_STREAM("The " << cur_box_colour 
        << " box failed to find a basket to be put in, because there is no " 
        << cur_box_colour << " basket or the " << cur_box_colour 
        << " basket(s) has reach its capacity");
    }
  }  
}

///////////////////////////////////////////////////////////////////////////////
