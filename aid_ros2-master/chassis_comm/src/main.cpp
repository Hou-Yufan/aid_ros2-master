#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "chassis_comm/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#define BAUD (B115200)
#define GRA_ACC (9.8)
#define DEG_TO_RAD (0.01745329)
#define BUF_SIZE (10240)

using robotis::chassis_comm::Odometry;
using namespace std::chrono_literals;
using namespace std;

typedef struct RC_s {
  uint8_t Type;
  uint8_t Mode;
  float X;
  float Y;
} RC_s;

typedef struct {
  float angle[3];
  float Acc[3];
  float Gyro[3];
  float wheel_circle[2];
  float wheel_rpm[2];
} sensor_data_all_t;

typedef struct {
  float vbat;
  float current;
  float bat;
  double mah;
  double max_mah;
} batt_s;

class ChassisComm : public rclcpp::Node {
 private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::Node> nh_;
  std::shared_ptr<sensor_msgs::msg::JointState> joint_;
  std::unique_ptr<Odometry> odometry_;
  float wheel_seperation_;
  float wheel_radius_;
  float whell_circle_data[2];
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_front_pub;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr
      ultrasonic_front_left_pub;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr
      ultrasonic_front_right_pub;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_rear_pub;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr
      ultrasonic_rear_left_pub;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr
      ultrasonic_rear_right_pub;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr laser_front_left_pub;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr laser_front_right_pub;
  uint8_t buf[BUF_SIZE] = {0};
  int fd = 0;
  float goal_linear_velocity_;
  float goal_angular_velocity_;
  std::string serial_port_name;

 public:
  ChassisComm() : Node("Chassis_comm_node") {
    init_parameters();
    fd = open_serial();
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/Imu_data", 20);
    battery_pub = this->create_publisher<sensor_msgs::msg::BatteryState>(
        "/battery_data", 20);
    ultrasonic_front_pub = this->create_publisher<sensor_msgs::msg::Range>(
        "/ultrasonic_front", 20);
    ultrasonic_front_left_pub = this->create_publisher<sensor_msgs::msg::Range>(
        "/ultrasonic_front_left", 20);
    ultrasonic_front_right_pub =
        this->create_publisher<sensor_msgs::msg::Range>(
            "/ultrasonic_front_right", 20);
    ultrasonic_rear_pub =
        this->create_publisher<sensor_msgs::msg::Range>("/ultrasonic_rear", 20);
    ultrasonic_rear_left_pub = this->create_publisher<sensor_msgs::msg::Range>(
        "/ultrasonic_rear_left", 20);
    ultrasonic_rear_right_pub = this->create_publisher<sensor_msgs::msg::Range>(
        "/ultrasonic_rear_right", 20);
    laser_front_left_pub = this->create_publisher<sensor_msgs::msg::Range>(
        "/laser_front_left", 20);
    laser_front_right_pub = this->create_publisher<sensor_msgs::msg::Range>(
        "/laser_front_right", 20);
    timer_ = this->create_wall_timer(
        2ms, std::bind(&ChassisComm::timer_callback, this));
    nh_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", qos,
        std::bind(&ChassisComm::command_velocity_callback, this,
                  std::placeholders::_1));
    odometry_ =
        std::make_unique<Odometry>(nh_, wheel_seperation_, wheel_radius_);
    joint_ = std::shared_ptr<sensor_msgs::msg::JointState>();
  }
  ~ChassisComm() {}

  void init_parameters() {
    this->declare_parameter("chassis_comm.separation");
    this->declare_parameter("chassis_comm.radius");
    this->declare_parameter("chassis_comm.serial_port");

    this->get_parameter_or<float>("chassis_comm.separation", wheel_seperation_,
                                  0.23);
    this->get_parameter_or<float>("chassis_comm.radius", wheel_radius_, 0.105);
    this->get_parameter_or<std::string>("chassis_comm.serial_port",
                                        serial_port_name,
                                        std::string("/dev/ttyS0"));
  }

  void Master_DT_Data_Receive_Prepare(uint8_t data) {
    static uint8_t RxBuffer[256];
    static uint8_t _data_len = 0, _data_cnt = 0;
    static uint8_t state = 0;

    if (state == 0 && data == 0xAA) {
      state = 1;
      RxBuffer[0] = data;
    } else if (state == 1 && data == 0xAA) {
      state = 2;
      RxBuffer[1] = data;
    } else if (state == 2 && data < 0xF1) {
      state = 3;
      RxBuffer[2] = data;
    } else if (state == 3 && data < 100) {
      state = 4;
      RxBuffer[3] = data;
      _data_len = data;
      _data_cnt = 0;
    } else if (state == 4 && _data_len > 0) {
      _data_len--;
      RxBuffer[4 + _data_cnt++] = data;
      if (_data_len == 0) state = 5;
    } else if (state == 5) {
      state = 0;
      RxBuffer[4 + _data_cnt] = data;
      Master_DT_Data_Receive_Anl(RxBuffer, _data_cnt + 5);
    } else
      state = 0;
  }

  void Master_DT_Data_Receive_Anl(uint8_t *data_buf, uint8_t num) {
    uint8_t sum = 0;

    for (uint8_t i = 0; i < num - 1; ++i) {
      sum += *(data_buf + i);
    }
    if (sum != *(data_buf + num - 1)) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("chassis_comm"),
                          "sum check failed !!!");
      return;
    }

    if (*data_buf != 0xAA || *(data_buf + 1) != 0xAA) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("chassis_comm"),
                          "data header check failed !!!");
      return;
    }

    if (*(data_buf + 2) == 0x0A) {
      sensor_data_all_t data;
      std::array<double, 2> wheel;
      memcpy((void *)&data, data_buf + 4, sizeof(data));
      // cout << "whell_circle_data" << data.wheel_circle[0] << " "
      //      << data.wheel_circle[1] << endl;
      // ;

      wheel[0] = data.wheel_circle[0];
      wheel[1] = -data.wheel_circle[1];
      odometry_->OdometryProcess(this->now(), wheel);
      auto imu_data = sensor_msgs::msg::Imu();
      imu_data.orientation.w = 1;
      imu_data.orientation.x = 0;
      imu_data.orientation.y = 0;
      imu_data.orientation.z = 0;
      imu_data.angular_velocity.x = data.Gyro[0] * DEG_TO_RAD;
      imu_data.angular_velocity.y = data.Gyro[1] * DEG_TO_RAD;
      imu_data.angular_velocity.z = data.Gyro[2] * DEG_TO_RAD;
      imu_data.linear_acceleration.x = data.Acc[0] * GRA_ACC;
      imu_data.linear_acceleration.y = data.Acc[1] * GRA_ACC;
      imu_data.linear_acceleration.z = data.Acc[2] * GRA_ACC;

      imu_data.header.stamp = rclcpp::Clock().now();
      imu_data.header.frame_id = "base_link";
      imu_pub->publish(imu_data);
    } else if (*(data_buf + 2) == 0x10) {
      batt_s bat;
      memcpy((void *)&bat, data_buf + 4, sizeof(bat));
      auto battery_data = sensor_msgs::msg::BatteryState();
      battery_data.current = bat.current;
      battery_data.voltage = bat.vbat;
      battery_data.percentage = bat.bat;
      battery_data.capacity = bat.mah;
      battery_data.design_capacity = bat.max_mah;

      battery_pub->publish(battery_data);
    } else if (*(data_buf + 2) == 0x0D) {
      uint16_t distence[8];
      memcpy((void *)&distence, data_buf + 4, 16);
      sensor_msgs::msg::Range data;
      data.field_of_view = 1;
      data.header.frame_id = "ultrasonic_front";
      data.header.stamp = rclcpp::Clock().now();
      data.max_range = 3;
      data.min_range = 0.2;
      data.range = distence[0] * 0.0001;
      data.radiation_type = data.ULTRASOUND;
      ultrasonic_front_pub->publish(data);
      data.header.frame_id = "ultrasonic_front_left_link";
      data.range = distence[1] * 0.0001;
      ultrasonic_front_left_pub->publish(data);
      data.header.frame_id = "ultrasonic_rear_link";
      data.range = distence[5] * 0.0001;
      ultrasonic_rear_pub->publish(data);
      data.header.frame_id = "ultrasonic_rear_right_link";
      data.range = distence[4] * 0.0001;
      ultrasonic_rear_right_pub->publish(data);
      data.header.frame_id = "ultrasonic_rear_left_link";
      data.range = distence[6] * 0.0001;
      ultrasonic_rear_left_pub->publish(data);
      data.header.frame_id = "ultrasonic_front_right_link";
      data.range = distence[7] * 0.0001;
      ultrasonic_front_right_pub->publish(data);
      // cout<<"0:"<<distence[0]<<" 1:"<<distence[1]<<" 4:"<<distence[4]<<"
      // 5:"<<distence[5]<<" 6:"<<distence[6]<<" 7:"<<distence[7]<<endl;
    } else if (*(data_buf + 2) == 0x0F) {
      uint16_t distence[2];
      memcpy((void *)&distence, data_buf + 4, 4);
      sensor_msgs::msg::Range data;
      data.field_of_view = 1;
      data.header.frame_id = "laser_front_left_link";
      data.header.stamp = rclcpp::Clock().now();
      data.max_range = 3;
      data.min_range = 0.2;
      data.range = distence[0] * 0.0001;
      data.radiation_type = data.INFRARED;
      laser_front_left_pub->publish(data);
      data.range = distence[1] * 0.0001;
      data.header.frame_id = "laser_front_right_link";
      laser_front_right_pub->publish(data);
    }
  }

  // @ Send message to serialPort
  // @ Struct_Data        the message
  // @ length             length of message
  // @ function           the header of message
  void Master_DT_Send_Struct(void *Struct_Data, uint8_t length,
                             uint8_t function) {
    uint8_t _cnt = 0;
    uint8_t sum = 0;

    uint8_t data_to_send[256];

    data_to_send[0] = 0xAA;
    data_to_send[1] = 0xAF;
    data_to_send[2] = function;
    data_to_send[3] = length;

    memcpy(&data_to_send[4], Struct_Data, length);

    _cnt = data_to_send[3] + 4;

    for (size_t i = 0; i < _cnt; ++i) sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

    write(fd, (const void *)data_to_send, _cnt);
  }

  void command_velocity_callback(
      const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg) {
    RC_s rc_data;

    goal_linear_velocity_ = cmd_vel_msg->linear.x;
    goal_angular_velocity_ = -cmd_vel_msg->angular.z;

    float wheel_left_vel = goal_linear_velocity_ -
                           (goal_angular_velocity_ * wheel_seperation_ / 2);
    float wheel_right_vel = goal_linear_velocity_ +
                            (goal_angular_velocity_ * wheel_seperation_ / 2);
    rc_data.X = -wheel_left_vel / (wheel_radius_ * 2.0 * M_PI) * 60;
    rc_data.Y = wheel_right_vel / (wheel_radius_ * 2.0 * M_PI) * 60;
    rc_data.Mode = 4;
    rc_data.Type = 2;
    Master_DT_Send_Struct((void *)&rc_data, sizeof(rc_data), 3);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("chassis_comm"),
                       "L:" << rc_data.X << " R:" << rc_data.Y
                            << " wheel_radius:" << wheel_radius_ << endl);
  }
  void timer_callback() {
    auto imu_data = sensor_msgs::msg::Imu();
    int n = read(fd, buf, sizeof(buf));

    for (int i = 0; i < n; i++) {
      // std::cout<<hex<<int(buf[i])<<std::endl;
      Master_DT_Data_Receive_Prepare(buf[i]);
    }

    memset(buf, 0, sizeof(buf));
  }

  int open_serial(void) {
    struct termios options;

    int fd = open(serial_port_name.c_str(), O_RDWR | O_NOCTTY);
    tcgetattr(fd, &options);

    if (fd == -1) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("chassis_comm"),
                          "unable to open serial port");
      exit(0);
    }

    if (fcntl(fd, F_SETFL, 0) < 0)
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("chassis_comm"), "fcntl failed");
    else
      fcntl(fd, F_SETFL, 0);

    if (isatty(STDIN_FILENO) == 0)
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("chassis_comm"),
                          "standard input is not a terminal device");
    else
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("chassis_comm"),
                          "isatty success!");

    memset(&options, 0, sizeof(options));

    options.c_cflag = BAUD | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);

    cfsetispeed(&options, BAUD);
    cfsetospeed(&options, BAUD);
    tcsetattr(fd, TCSANOW, &options);

    return fd;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisComm>());
  rclcpp::shutdown();
}
