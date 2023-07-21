#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <vector>

#include "aid_robot_msgs/srv/draw_picture.hpp"
#include "aid_robot_msgs/srv/get_current_forbidden.hpp"
#include "aid_robot_msgs/srv/get_current_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
using namespace std::placeholders;
typedef struct gridindex_ {
  int x;
  int y;

  void SetIndex(int x_, int y_) {
    x = x_;
    y = y_;
  }
} GridIndex;

/**
 * @brief Increments all the grid cells from (x0, y0) to (x1, y1);
 *
 * @param x0
 * @param y0
 * @param x1
 * @param y1
 * @return std::vector<GridIndex>
 */
std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1) {
  GridIndex tmpIndex;
  std::vector<GridIndex> gridIndexVector;
  bool steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }
  int deltaX = x1 - x0;
  int deltaY = abs(y1 - y0);
  int error = 0;
  int ystep;
  int y = y0;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }
  int pointX;
  int pointY;
  for (int x = x0; x <= x1; x++) {
    if (steep) {
      pointX = y;
      pointY = x;
    } else {
      pointX = x;
      pointY = y;
    }
    error += deltaY;
    if (2 * error >= deltaX) {
      y += ystep;
      error -= deltaX;
    }
    if (pointX == x1 && pointY == y1) continue;
    tmpIndex.SetIndex(pointX, pointY);
    gridIndexVector.push_back(tmpIndex);
  }

  return gridIndexVector;
}

class ForbiddenMapCreate : public rclcpp::Node {
 public:
  ForbiddenMapCreate() : Node("forbidden_map_create") {
    is_get_map_info_ = false;
    map_id_ = UINT32_MAX;
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/localization_map", 1,
        std::bind(&ForbiddenMapCreate::MapCallback, this,
                  std::placeholders::_1));
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/forbidden_map", rclcpp::QoS(10).keep_last(1));
    draw_forbidden_server_ =
        this->create_service<aid_robot_msgs::srv::DrawPicture>(
            "aid_draw_forbidden_line",
            std::bind(&ForbiddenMapCreate::DrawFobbiddenLineCallback, this, _1,
                      _2));
  }

 private:
  void MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    is_get_map_info_ = true;
    static size_t last_get_subscription_count = 0;
    map_sub_ = (*msg);
    // MapCreate();

    uint32_t map_id = 0;
    GetCurrentMapIp(map_id);
    if ((map_id != map_id_) ||
        (publisher_->get_subscription_count() != last_get_subscription_count)) {
      GetForbiddenAndDraw();
      publisher_->publish(map_pub_);
      map_id_ = map_id;
      last_get_subscription_count = publisher_->get_subscription_count();
    }
  }

  bool GetForbiddenAndDraw() {
    map_pub_ = map_sub_;
    auto node = rclcpp::Node::make_shared("get_forbidden_client");
    auto get_forbidden_client =
        node->create_client<aid_robot_msgs::srv::GetCurrentForbidden>(
            "get_current_forbidden");
    auto request =
        std::make_shared<aid_robot_msgs::srv::GetCurrentForbidden::Request>();

    if (!get_forbidden_client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_INFO(rclcpp::get_logger("get_forbidden_client"),
                  "service get_forbidden_client not available");
      return false;
    }

    auto result = get_forbidden_client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result,
                                           std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("get_forbidden_client"),
                         "message:" << result.get()->success);
      if (result.get()->success == true) {
        size_t line_num = result.get()->message.size();
        for (size_t i = 0; i < line_num; i++) {
          DrawLineToMap(result.get()->message[i]);
        }
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("get_forbidden_client"),
                     "Call service get_forbidden_client,return false");
        return false;
      }

    } else {
      RCLCPP_ERROR(rclcpp::get_logger("get_forbidden_client"),
                   "Failed to call service get_forbidden_client");
      return false;
    }
    return true;
  }

  bool GetCurrentMapIp(uint32_t &map_id) {
    auto node = rclcpp::Node::make_shared("get_current_map_id_client");
    auto get_current_map_id_client =
        node->create_client<aid_robot_msgs::srv::GetCurrentMap>(
            "get_current_map_id");
    auto request =
        std::make_shared<aid_robot_msgs::srv::GetCurrentMap ::Request>();

    if (!get_current_map_id_client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_INFO(rclcpp::get_logger("get_current_map_id"),
                  "service get_current_map_id not available");
      return false;
    }

    auto result = get_current_map_id_client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result,
                                           std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("get_current_map_id"),
                         "message:" << result.get()->success);
      if (result.get()->success == true) {
        map_id = result.get()->map_id;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("get_current_map_id"),
                     "Call service get_forbidden_client,return false");
        return false;
      }

    } else {
      RCLCPP_ERROR(rclcpp::get_logger("get_current_map_id"),
                   "Failed to call service get_current_map_id");
      return false;
    }
    return true;
  }

  void DrawFobbiddenLineCallback(
      const std::shared_ptr<aid_robot_msgs::srv::DrawPicture::Request> request,
      std::shared_ptr<aid_robot_msgs::srv::DrawPicture::Response> response) {
    if (is_get_map_info_ == false) {
      response->success = false;
      response->message = "No map get.";
      return;
    }
    map_pub_ = map_sub_;
    // size_t map_data_size = map_pub_.data.size();
    // for (size_t i = 0; i < map_data_size; i++) {
    //     map_pub_.data[i] = 0;
    // }
    // map_pub_.data;
    size_t line_num = request->data.size();
    for (size_t i = 0; i < line_num; i++) {
      DrawLineToMap(request->data[i]);
    }
    response->success = true;
    response->message = "success";
    // Publish new message
    publisher_->publish(map_pub_);
  }

  void DrawLineToMap(aid_robot_msgs::msg::StartToEndPoint line) {
    auto start_index =
        ConvertWorld2GridIndex(map_pub_, line.start.x, line.start.y);
    auto end_index = ConvertWorld2GridIndex(map_pub_, line.end.x, line.end.y);
    auto line_points =
        TraceLine(start_index.x, start_index.y, end_index.x, end_index.y);
    for (size_t i = 0; i < line_points.size(); i++) {
      GridIndex point = line_points[i];
      if (IsValidGridIndex(map_pub_, point) == false) {
        continue;
      }
      int index = GridIndexToLinearIndex(map_pub_, point);
      map_pub_.data[index] = 100;
    }
  }
  /**
   * @brief Convert linear index to world
   *
   * @param map Input map
   * @param linear_index Input index
   * @param x Out x
   * @param y Out y
   */
  void ConvertLinearIndex2World(const nav_msgs::msg::OccupancyGrid &map,
                                int linear_index, double *x, double *y) {
    GridIndex index;
    index.x = linear_index % map.info.width;
    index.y = linear_index / map.info.width;
    *x = static_cast<double>(index.x) * map.info.resolution -
         0.5 * map.info.resolution + map.info.origin.position.x;
    *y = static_cast<double>(index.y) * map.info.resolution -
         0.5 * map.info.resolution + map.info.origin.position.y;
  }
  /**
   * @brief Check whether index is valid
   *
   * @param map
   * @param index
   * @return true
   * @return false
   */
  bool IsValidGridIndex(const nav_msgs::msg::OccupancyGrid &map,
                        GridIndex index) {
    if (index.x >= 0 && (unsigned int)index.x < map.info.width &&
        index.y >= 0 && (unsigned int)index.y < map.info.height)
      return true;

    return false;
  }

  /**
   * @brief Convert from world to raster coordinates
   *
   * @param map
   * @param x
   * @param y
   * @return GridIndex
   */
  GridIndex ConvertWorld2GridIndex(const nav_msgs::msg::OccupancyGrid &map,
                                   double x, double y) {
    GridIndex index;
    index.x = std::ceil((x - map.info.origin.position.x) / map.info.resolution);
    index.y = std::ceil((y - map.info.origin.position.y) / map.info.resolution);
    return index;
  }
  /**
   * @brief Grid index to linear index
   *
   * @param map
   * @param index
   * @return int
   */
  int GridIndexToLinearIndex(const nav_msgs::msg::OccupancyGrid &map,
                             GridIndex index) {
    int linear_index;
    linear_index = index.x + index.y * map.info.width;
    return linear_index;
  }

  void MapCreate() {
    map_pub_ = map_sub_;
    // map_pub_.data;
    auto line_points =
        TraceLine(0, 0, map_pub_.info.width, map_pub_.info.height);
    for (size_t i = 0; i < line_points.size(); i++) {
      GridIndex point = line_points[i];
      if (IsValidGridIndex(map_pub_, point) == false) {
        continue;
      }
      int index = GridIndexToLinearIndex(map_pub_, point);
      map_pub_.data[index] = 100;
    }

    // Publish new message
    publisher_->publish(map_pub_);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Service<aid_robot_msgs::srv::DrawPicture>::SharedPtr
      draw_forbidden_server_;
  nav_msgs::msg::OccupancyGrid map_sub_;
  nav_msgs::msg::OccupancyGrid map_pub_;
  bool is_get_map_info_;
  uint32_t map_id_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ForbiddenMapCreate>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
