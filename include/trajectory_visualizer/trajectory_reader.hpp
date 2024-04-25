#ifndef TRAJECTORY_VISUALIZER_TRAJECTORY_READER_HPP
#define TRAJECTORY_VISUALIZER_TRAJECTORY_READER_HPP

#include "trajectory_visualizer/ReadTrajectory.h"
#include "trajectory_visualizer/trajectory_publisher.hpp"
#include "trajectory_visualizer/transform_broadcaster.hpp"

/*  Params          Type      Default Value
 *  map_frame      string      /map
 *  base_frame     string      /base_link
 */
class TrajectoryReader {
public:
    // Constructor.
    TrajectoryReader(std::string topic = "trajectory_marker");
    TrajectoryReader(const ros::NodeHandlePtr& nh, const ros::NodeHandlePtr& pnh, std::string topic = "trajectory_marker");

    // Initialize.
    void init();
    // Update Loop of single replay.
    void updateLoop(size_t start_index, size_t end_index, std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> trajectory);
    // Calculate start and end index based on interval.
    void calculateIndex(const std::vector<geometry_msgs::PoseStamped>& trajectory, int start_time_rel, int end_time_rel, size_t& start_index, size_t& end_index);
    // Trajectory reader service.
    bool readerService(trajectory_visualizer::ReadTrajectory::Request& req, trajectory_visualizer::ReadTrajectory::Response& res);

private:
    // Node Handle.
    ros::NodeHandlePtr nh_;
    // Private Node Handle.
    ros::NodeHandlePtr pnh_;
    // Frequency.
    double frequency_;
    // Map frame name.
    std::string map_frame_;
    // Base frame name.
    std::string base_frame_;
    // Trajectory Reader service.
    ros::ServiceServer service_;
    // Transform broadcaster.
    std::shared_ptr<TransformBroadcaster> tf_broadcaster_;
    // Trajectory publisher.
    TrajectoryPublisher traj_pub_;
};

#endif
