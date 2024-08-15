#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <signal.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud; // Use PointXYZI to include intensity

class PCDSaver {
public:
    PCDSaver() : nh_("~"), max_points_(1000000) {
        // Get the topic and filename from parameters
        nh_.param<std::string>("input_topic", input_topic_, "/default_pointcloud_topic");
        nh_.param<std::string>("output_file", output_file_, "output.pcd");
        nh_.param<int>("max_points", max_points_, max_points_);

        // Subscribe to the point cloud topic
        sub_ = nh_.subscribe<PointCloud>(input_topic_, 1, &PCDSaver::callback, this);

        // Setup signal handler to flush data when program is terminated
        signal(SIGINT, PCDSaver::signalHandler);
    }

    static void signalHandler(int signum) {
        ROS_INFO("Signal %d received, saving point cloud to file...", signum);
        getInstance().flushToFile(true);
        ros::shutdown();
    }

    void callback(const PointCloud::ConstPtr& msg) {
        // Accumulate points in the internal point cloud
        cloud_ += *msg;

        // Check if the accumulated point cloud exceeds the maximum allowed points
        if (cloud_.points.size() >= max_points_) {
            ROS_INFO("Max points reached, flushing data to file...");
            flushToFile(false);
        }
    }

    void flushToFile(bool final_flush) {
        if (cloud_.points.empty()) {
            ROS_INFO("No points to save, skipping file write.");
            return;
        }

        // Load the existing PCD file if it exists, or create a new one
        PointCloud existing_cloud;
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(output_file_, existing_cloud) == -1) {
            ROS_WARN("Couldn't read file %s. A new file will be created.", output_file_.c_str());
        }

        // Append new points to the existing cloud
        existing_cloud += cloud_;

        // Save the updated cloud back to the PCD file
        pcl::io::savePCDFileBinary(output_file_, existing_cloud); // Save in binary format for efficiency
        ROS_INFO("Saved %lu data points to %s.", existing_cloud.points.size(), output_file_.c_str());

        // Clear the cloud to free up memory
        cloud_.clear();
    }

    static PCDSaver& getInstance() {
        static PCDSaver instance;
        return instance;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::string input_topic_;
    std::string output_file_;
    PointCloud cloud_;
    int max_points_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_saver", ros::init_options::NoSigintHandler);
    PCDSaver pcd_saver;
    ros::spin();
    return 0;
}
