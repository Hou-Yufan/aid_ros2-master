#include <cstdio>
#include <vector>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "opencv2/opencv.hpp"
#include "base64.h"

using namespace std::chrono_literals;

class MapTransform : public rclcpp::Node {
public:
    MapTransform() : Node("map_transform") {
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            1,
            std::bind(&MapTransform::callback, this, std::placeholders::_1)
        );
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/map_base64",
            1
        );
    }

private:
    void callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) const {
        // Convert data field to 2D vector
        std::vector<std::vector<int>> array2d;
        int width = msg->info.width;
        int height = msg->info.height;
        for (int i = 0; i < height; i++) {
            std::vector<int> row;
            for (int j = 0; j < width; j++) {
                if (msg->data[i * width + j] == -1) {
                    row.push_back(127);
                } else {
                    row.push_back((100 - msg->data[i * width + j]) * 2);
                }
            }
            array2d.push_back(row);
        }

        // Convert 2D vector to grayscale image
        cv::Mat img(height, width, CV_8UC1);
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                img.at<uchar>(height - i - 1, j) = array2d[i][j];
            }
        }

        // Convert image to JPEG format
        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
        cv::imencode(".jpg", img, buffer, params);

        // Convert JPEG data to base64 format
        std::string base64_data = base64_encode(buffer.data(), buffer.size());

        // Convert base64_data to sequence of uint8_t
        std::vector<uint8_t> int_array(base64_data.begin(), base64_data.end());

        // Create new OccupancyGrid message
        auto send_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        send_msg->header = msg->header;
        send_msg->info = msg->info;

        // Convert uint8_t vector to C-style array
        uint8_t* data_array = new uint8_t[int_array.size()];
        std::copy(int_array.begin(), int_array.end(), data_array);

        // Assign C-style array to send_msg.data
        send_msg->data = std::vector<uint8_t>(data_array, data_array + int_array.size());

        // Publish new message
        publisher_->publish(*send_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapTransform>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
