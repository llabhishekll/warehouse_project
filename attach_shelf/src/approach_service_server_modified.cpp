#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <cmath>

class ApproachService : public rclcpp::Node {
private:
  // node parameters
  bool use_sim_time;

  // member variables
  double px;
  double py;
  double x;
  double y;
  double yaw;

  // flag variable
  bool is_approachable;

  // callback groups
  rclcpp::CallbackGroup::SharedPtr callback_g1;
  rclcpp::CallbackGroup::SharedPtr callback_g2;

  // ros objects
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_server;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_scan;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel;
  rclcpp::TimerBase::SharedPtr timer_tf_broadcaster;

  // tf objects
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_dynamic_broadcaster;

  // member method
  void
  subscriber_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // find the front distance after reducing noise
    auto first = msg->intensities.begin();
    auto last = msg->intensities.end();

    // lambda function to detect reflector
    auto find_reflector = [](int i) { return i > 7000; };

    // find first leg
    auto it1 = std::find_if(first, last, find_reflector);
    if (it1 == last) {
      this->is_approachable = false;
      return;
    }
    int loc1 = std::distance(first, it1);
    double leg1_angle = (loc1 - 540) * msg->angle_increment;
    double leg1_distance = msg->ranges.at(loc1);

    // find second leg
    auto it2 = std::find_if(first + loc1 + 20, last, find_reflector);
    if (it2 == last) { // bug fix: error `out_of_range` in std::distance
      this->is_approachable = false;
      return;
    }
    int loc2 = std::distance(first, it2);
    double leg2_angle = (loc2 - 540) * msg->angle_increment;
    double leg2_distance = msg->ranges.at(loc2);

    // validate if both legs of cart are detected
    if ((it1 != last) && (it2 != last)) {
      // calculate (x1, y1), (x2, y2)
      double x1 = leg1_distance * std::cos(leg1_angle);
      double y1 = leg1_distance * std::sin(leg1_angle);
      double x2 = leg2_distance * std::cos(leg2_angle);
      double y2 = leg2_distance * std::sin(leg2_angle);

      // calculate mid point between cart legs
      this->px = (x1 + x2) / 2.0;
      this->py = (y1 + y2) / 2.0;
      this->is_approachable = true;
    }
  }

  void subscriber_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // reading current position from /odom topic
    this->x = msg->pose.pose.position.x;
    this->y = msg->pose.pose.position.y;

    // reading current orientation from /odom topic
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;

    // convert quaternion into euler angles
    double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    // fix: radian to degree conversions
    this->yaw = (180 / M_PI) * yaw;
  }

  void publish_cart_frame() {
    // required frames parameters
    std::string f_ref = "map";
    std::string f_tar = "robot_front_laser_link";

    // odom->laser transformation
    tf2::Stamped<tf2::Transform> map2laser;
    geometry_msgs::msg::TransformStamped map2laser_msg;

    try {
      map2laser_msg =
          this->tf_buffer->lookupTransform(f_ref, f_tar, tf2::TimePointZero);
      tf2::fromMsg(map2laser_msg, map2laser);
    } catch (const tf2::TransformException &ex) {
      // node feedback
      RCLCPP_WARN(get_logger(), "Requested transform not found: %s", ex.what());
      return;
    }

    // laser->cart transformation
    tf2::Transform laser2cart;
    laser2cart.setOrigin(tf2::Vector3(this->px, this->py, 0.0));
    laser2cart.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    // odom->cart transformation
    tf2::Transform map2cart = map2laser * laser2cart;

    // publish odom->cart transformation
    geometry_msgs::msg::TransformStamped map2cart_msg;
    map2cart_msg.header.stamp = map2laser_msg.header.stamp;
    map2cart_msg.header.frame_id = "map";
    map2cart_msg.child_frame_id = "cart_frame";
    map2cart_msg.transform.translation.x = map2cart.getOrigin().getX();
    map2cart_msg.transform.translation.y = map2cart.getOrigin().getY();
    map2cart_msg.transform.translation.z = map2cart.getOrigin().getZ();
    map2cart_msg.transform.rotation.x = 0;
    map2cart_msg.transform.rotation.y = 0;
    map2cart_msg.transform.rotation.z = 0;
    map2cart_msg.transform.rotation.w = 1;

    // publish transformation
    this->tf_dynamic_broadcaster->sendTransform(map2cart_msg);
  }

  void service_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // node feedback
    RCLCPP_INFO(this->get_logger(), "/approach_shelf received a new request");

    //
    (void)request;
    bool status;

    //
    if (this->use_sim_time) {
      status = this->sim_controller();
    } else {
      status = this->real_controller();
    }

    // halt robot motion
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.angular.z = 0.0;
    this->publisher_cmd_vel->publish(message);

    // set return status
    response->success = status;
  }

  bool sim_controller() {
    // restart the publish_cart_frame timmer
    this->timer_tf_broadcaster->reset();
    // bad fix: halt the process for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // publishing velocity to the topic /cmd_vel
    auto message = geometry_msgs::msg::Twist();

    // required frames parameters
    std::string f_ref = "robot_base_footprint";
    std::string f_tar = "cart_frame";

    while (rclcpp::ok()) {
      if (tf_buffer->canTransform(f_ref, f_tar, tf2::TimePointZero)) {
        // fetch transformation
        auto t = tf_buffer->lookupTransform(f_ref, f_tar, tf2::TimePointZero);

        auto x = t.transform.translation.x;
        auto y = t.transform.translation.y;

        // calculate error
        auto error_distance = std::sqrt(x * x + y * y);
        auto error_yaw = std::atan2(y, x);

        // local control structure
        if (error_distance > 0.10) {
          message.linear.x = 0.25;
          message.angular.z = error_yaw / 2;
        } else {
          RCLCPP_INFO(this->get_logger(), "error_distance %f", error_distance);
          break;
        }
        // publish velocity
        this->publisher_cmd_vel->publish(message);

        // node feedback
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "(distance : %f, angle : %f)", error_distance,
                             error_yaw);
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Requested transform not found");
      }
    }
    this->timer_tf_broadcaster->cancel();
    return true;
  }

  bool real_controller() {
    // publishing velocity to the topic /cmd_vel
    auto message = geometry_msgs::msg::Twist();

    // required frames parameters
    std::string f_ref = "robot_base_footprint";
    std::string f_tar = "robot_cart_laser";

    double last_error_distance = 0.0;
    bool loop_once_done = false;

    while (rclcpp::ok()) {
      if (tf_buffer->canTransform(f_ref, f_tar, tf2::TimePointZero)) {
        // fetch transformation
        auto t = tf_buffer->lookupTransform(f_ref, f_tar, tf2::TimePointZero);

        auto x = t.transform.translation.x;
        auto y = t.transform.translation.y;

        // calculate error
        auto error_distance = std::sqrt(x * x + y * y);
        auto error_yaw = std::atan2(y, x);

        RCLCPP_INFO_ONCE(this->get_logger(), "Target : %f", error_distance);

        // local control structure
        if ((error_distance > 0.48) || (error_distance < last_error_distance)) {
          message.linear.x = 0.1;
          message.angular.z = error_yaw / 2;
          last_error_distance = error_distance;
        } else {
          RCLCPP_INFO(this->get_logger(), "error_distance %f", error_distance);
          break;
        }
        loop_once_done = true;

        // node feedback
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "(distance : %f, angle : %f)", error_distance,
                             error_yaw);

        // publish velocity
        this->publisher_cmd_vel->publish(message);
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1200,
                             "Requested transform not found");
        if (loop_once_done) {
          break;
        }
      }
    }
    return true;
  }

public:
  // constructor
  ApproachService()
      : Node("approach_shelf_service_node"), is_approachable(false) {

    // node parameters
    this->use_sim_time = this->get_parameter("use_sim_time").as_bool();

    // callback groups objects
    callback_g1 =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    callback_g2 =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // ros node options
    rclcpp::SubscriptionOptions sub_callback_g1;
    sub_callback_g1.callback_group = callback_g1;

    // ros objects
    this->service_server = create_service<std_srvs::srv::Trigger>(
        "/approach_shelf",
        std::bind(&ApproachService::service_callback, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_g2);
    this->subscriber_scan =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ApproachService::subscriber_scan_callback, this,
                      std::placeholders::_1),
            sub_callback_g1);
    this->publisher_cmd_vel =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    this->timer_tf_broadcaster = this->create_wall_timer(
        std::chrono::seconds(1 / 120),
        std::bind(&ApproachService::publish_cart_frame, this), callback_g1);
    this->timer_tf_broadcaster->cancel();

    // tf objects
    this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    this->tf_dynamic_broadcaster =
        std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // node acknowledgement
    RCLCPP_INFO(this->get_logger(),
                "The service /approach_shelf is available for request.");
  }
};

int main(int argc, char *argv[]) {
  // initialize ros, executor and node
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<ApproachService> node = std::make_shared<ApproachService>();

  // add node to executor and spin
  executor.add_node(node);
  executor.spin();

  // shutdown
  rclcpp::shutdown();
  return 0;
}