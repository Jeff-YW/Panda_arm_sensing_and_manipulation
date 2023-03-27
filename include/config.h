#ifndef CONFIG_H_
#define CONFIG_H_

/** \brief Subscriber and shutdown subscriber function. */
ros::Subscriber pc_sub;
void unsubscribeFromPointCloudTopic();

/** \brief Gripper constants */
const double GRIPPER_OPEN = 80e-3;
const double GRIPPER_CLOSED = 0.0;

/** \brief Parameters to define the pick operation */
const double Z_OFFSET = 0.125;
const double ANGLE_OFFSET = 3.14159 / 4.0;
const double APPROACH_DISTANCE = 0.1;

/** \brief Maximum number of boxes that a basket can take */
const int BASKET_CAPACITY = 3;

/** \brief Box cluster size constants. */
const int BOX_CLUSTER_SIZE_MIN = 100;
const int BOX_CLUSTER_SIZE_MAX = 4000;

/** \brief Basket cluster size constants. */
const int BASKET_CLUSTER_SIZE_MIN = 6000;
const int BASKET_CLUSTER_SIZE_MAX = 25000;

/** \brief Colour threshold. */
const double COLOUR_THRESHOLD = 80.0;

/** \brief world frame. */
const std::string WORLD_FRAME = "world";

/** \brief base frame. */
const std::string BASE_FRAME = "panda_link0";

#endif