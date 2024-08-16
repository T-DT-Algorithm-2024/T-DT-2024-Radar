#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <vision_interface/msg/match_info.hpp>
#include <thread>

using namespace std::chrono_literals;

void on_exit([[maybe_unused]] int sig) {
    RCUTILS_LOG_INFO("Exit by Ctrl+C");
    rclcpp::shutdown();
    exit(0);
}

class RosbagPlayer : public rclcpp::Node {
public:
    RosbagPlayer(const rclcpp::NodeOptions & options)
        : Node("rosbag_player_node", options) {
        this->declare_parameter<std::string>("rosbag_file", "");
        this->get_parameter("rosbag_file", rosbag_file);

        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10);
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_image", rclcpp::SensorDataQoS());
        match_info_publisher_ = this->create_publisher<vision_interface::msg::MatchInfo>("/match_info", 10);
        signal(SIGINT, on_exit);
        // 创建一个新的线程来处理bag文件
            reader_.open(rosbag_file);

        processing_thread_ = std::make_shared<std::thread>(&RosbagPlayer::play_bag, this);
    }

    ~RosbagPlayer() {
        if (processing_thread_ && processing_thread_->joinable()) {
            processing_thread_->join();
        }
    }

private:
    void play_bag() {
        while (rclcpp::ok()) {
            if(!reader_.has_next())
            reader_.open(rosbag_file);

            auto start_time = std::chrono::high_resolution_clock::now();
            auto bag_message = reader_.read_next();
            auto ros_time = rclcpp::Clock().now();  
            
            // 处理 PointCloud2 消息
            if (bag_message->topic_name == "/livox/lidar") {
                auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(&serialized_msg, pointcloud_msg.get());
                pointcloud_msg->header.stamp = ros_time;
                pointcloud_publisher_->publish(*pointcloud_msg);
            }
            // 处理 CompressedImage 消息
            else if (bag_message->topic_name == "/compressed_image") {
                auto image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
                rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serialization;
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(&serialized_msg, image_msg.get());
                auto img = cv::imdecode(image_msg->data, cv::IMREAD_COLOR);
                auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
                msg->header.stamp = ros_time;
                image_publisher_->publish(*msg);
            }
            // 处理match_info消息
            else if (bag_message->topic_name == "/match_info") {
                // RCLCPP_INFO(this->get_logger(), "Received match_info message");
                auto match_info_msg = std::make_shared<vision_interface::msg::MatchInfo>();
                rclcpp::Serialization<vision_interface::msg::MatchInfo> serialization;
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                // std::cout<<"serialized_msg.size():"<<serialized_msg.size()<<std::endl;
                serialization.deserialize_message(&serialized_msg, match_info_msg.get());
                match_info_publisher_->publish(*match_info_msg);
            }

            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            // RCLCPP_INFO(this->get_logger(), "Processed one message, duration: %d ms", duration);
            if((duration < 100)&&(duration > 1)) {
                std::this_thread::sleep_for(100ms - std::chrono::milliseconds(duration));
            }
        }
        RCLCPP_INFO(this->get_logger(), "No more messages in the bag.");
        rclcpp::shutdown();
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<vision_interface::msg::MatchInfo>::SharedPtr match_info_publisher_;
    rosbag2_cpp::Reader reader_;
    std::shared_ptr<std::thread> processing_thread_;
    std::string rosbag_file;

};

RCLCPP_COMPONENTS_REGISTER_NODE(RosbagPlayer)
