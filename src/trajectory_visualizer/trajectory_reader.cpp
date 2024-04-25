#include "trajectory_visualizer/trajectory_reader.hpp"
#include "trajectory_visualizer/trajectory_io.hpp"

TrajectoryReader::TrajectoryReader(std::string topic /* = "trajectory_marker" */)
    : nh_(new ros::NodeHandle()), pnh_(new ros::NodeHandle("~")), traj_pub_(topic)
{
    init();
}

TrajectoryReader::TrajectoryReader(const ros::NodeHandlePtr& nh, const ros::NodeHandlePtr& pnh, std::string topic /* = "trajectory_marker" */)
    : nh_(nh), pnh_(pnh), traj_pub_(topic)
{
    init();
}

void TrajectoryReader::init()
{
    this->service_ = nh_->advertiseService("read_trajectory", &TrajectoryReader::readerService, this);
    pnh_->param<std::string>("map_frame", map_frame_, "/map");
    pnh_->param<std::string>("base_frame", base_frame_, "/base_link");
    tf_broadcaster_ = std::make_shared<TransformBroadcaster>(map_frame_, base_frame_);
}

void TrajectoryReader::updateLoop(size_t start_index, size_t end_index, std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> trajectory)
{
    ros::Time current_time = ros::Time::now();
    ros::Time last_time_og;
    ros::Time last_time;
    ros::Duration d(0.1);
    double time_diff = 0.1;

    for (size_t i = start_index; i <= end_index; ++i)
    {
        geometry_msgs::PoseStamped pose = trajectory->at(i);
        if (i == start_index)
        {
            last_time_og = ros::Time(pose.header.stamp);
            pose.header.stamp = current_time;
        }
        else
        {
            time_diff = (pose.header.stamp - last_time_og).toSec();
            pose.header.stamp = last_time + ros::Duration(time_diff);
        }

        tf_broadcaster_->broadcast(pose);
        traj_pub_.updateTrajectory(pose);
        d.sleep();
        last_time = pose.header.stamp;
    }
}

bool TrajectoryReader::readerService(trajectory_visualizer::ReadTrajectory::Request& req, trajectory_visualizer::ReadTrajectory::Response& res)
{
    traj_pub_.clearAll();
    std::string filename = req.filename;
    int start_time_rel = req.start_time;
    int end_time_rel = req.end_time;
    bool loop = req.loop;

    std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> trajectory = TrajectoryIO::readTrajectory(filename);
    if (trajectory == nullptr || trajectory->empty())
    {
        ROS_INFO("Trajectory is null or empty");
        res.result = false;
        return true;
    }

    size_t start_index = 0;
    size_t end_index = trajectory->size() - 1;

    if (start_time_rel > 0 || end_time_rel > 0)
    {
        double start_time_abs = trajectory->front().header.stamp.toSec();
        double end_time_abs = trajectory->back().header.stamp.toSec();
        start_time_rel = start_time_rel < 0 ? 0 : start_time_rel;
        end_time_rel = end_time_rel < 0 ? end_time_abs : end_time_rel;
        double difference = end_time_abs - start_time_abs;
        if (start_time_rel >= difference)
        {
            ROS_ERROR("Start time exceeding the total duration");
            res.result = false;
            return true;
        }

        double start_time_final = std::max(start_time_abs, start_time_abs + start_time_rel);
        double end_time_final = std::min(end_time_abs, start_time_abs + end_time_rel);

        for (size_t i = 0; i < trajectory->size(); ++i)
        {
            if (trajectory->at(i).header.stamp.toSec() >= start_time_final)
            {
                start_index = i;
                break;
            }
        }

        for (size_t i = start_index; i < trajectory->size(); ++i)
        {
            if (trajectory->at(i).header.stamp.toSec() > end_time_final)
            {
                end_index = i - 1;
                break;
            }
        }
    }
    ROS_INFO("Start index: %zu, End index: %zu", start_index, end_index);
    try
    {
        while (true)
        {
            updateLoop(start_index, end_index, trajectory);
            if (!loop)
                break;

            traj_pub_.clearAll();
        }
    }
    catch (std::exception& e)
    {
        ROS_ERROR_STREAM("Caught error" << e.what());
    }
    res.result = true;

    return true;
}
