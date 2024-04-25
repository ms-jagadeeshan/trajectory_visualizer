#include "trajectory_visualizer/trajectory_saver.hpp"
#include "trajectory_visualizer/trajectory_io.hpp"

TrajectorySaver::TrajectorySaver()
    : nh_(new ros::NodeHandle()), pnh_(new ros::NodeHandle("~"))
{
    init();
}

TrajectorySaver::TrajectorySaver(const ros::NodeHandlePtr& nh, const ros::NodeHandlePtr& pnh)
    : nh_(nh), pnh_(pnh)
{
    init();
}
void TrajectorySaver::init()
{
    this->service_ = nh_->advertiseService("save_trajectory", &TrajectorySaver::saverService, this);
    pnh_->param<std::string>("map_frame", map_frame_, "/map");
    pnh_->param<std::string>("base_frame", base_frame_, "/base_link");
    pnh_->param<double>("publish_rate", frequency_, 10);

    tf_listener_ = std::make_shared<TransformListener>(map_frame_, base_frame_, frequency_);
}

bool TrajectorySaver::saverService(trajectory_visualizer::SaveTrajectory::Request& req, trajectory_visualizer::SaveTrajectory::Response& res)
{
    std::string filename = req.filename;
    auto duration = ros::Duration(req.duration);
    std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> poses = tf_listener_->getPoses(duration);
    bool result = TrajectoryIO::saveTrajectory(filename, poses);
    res.result = result;
    return true;
}
