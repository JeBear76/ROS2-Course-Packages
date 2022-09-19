#include <cmath>

#include "assignment_interfaces/msg/dead_turtle.hpp"
#include "assignment_interfaces/msg/spawn_location.hpp"
#include "assignment_interfaces/msg/turtle_control_ready.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

using assignment_interfaces::msg::DeadTurtle;
using assignment_interfaces::msg::SpawnLocation;
using assignment_interfaces::msg::TurtleControlReady;
using geometry_msgs::msg::Twist;
using std::placeholders::_1;
using turtlesim::msg::Pose;

class TurtleControlNode : public rclcpp::Node {
 public:
  TurtleControlNode(std::string nodeName) : Node(nodeName) {
    _nodeName = nodeName;

    this->declare_parameter<double>("turtle_speed", 1.4);
    _turtle_speed = this->get_parameter("turtle_speed").as_double();

    _turtleSpawnListener = this->create_subscription<SpawnLocation>(
        "turtle_spawn_location", 10,
        std::bind(&TurtleControlNode::newTurtleReceived, this,
                  std::placeholders::_1));

    _turtlePoseListener = this->create_subscription<Pose>(
        "turtle1/pose", 10,
        std::bind(&TurtleControlNode::updatePose, this, _1));

    _deadTurtlePublisher =
        this->create_publisher<DeadTurtle>("dead_turtle", 10);

    _turtleDriverPublisher =
        this->create_publisher<Twist>("turtle1/cmd_vel", 10);

    this->create_publisher<TurtleControlReady>("turtle_control_ready", 10)
        ->publish(TurtleControlReady());

    _decisionTimer =
        this->create_wall_timer(std::chrono::milliseconds(250),
                                std::bind(&TurtleControlNode::hunt, this));

    RCLCPP_INFO(this->get_logger(),
                (std::string("ROS2 ") + nodeName + std::string(" C++ started."))
                    .c_str());
  }

 private:
  /**
   * @brief
   *
   * @param spawnLocation
   */
  void newTurtleReceived(const SpawnLocation spawnLocation) {
    std::lock_guard<std::mutex> lock(threadMutex);
    _turtles.push_back(spawnLocation);
  }

  /**
   * @brief get distance of all turtles from _turtlePose.
   * Find the closest one
   */
  void findNearestTurtle() {
    float shortest = 100.0;
    float distance = 100.0;

    for (auto &&turtle : _turtles) {
      distance = calculateDistance(&turtle);
      RCLCPP_INFO(this->get_logger(),
                  ("distance: " + std::to_string(distance)).c_str());
      if (distance < shortest) {
        shortest = distance;
        _targetTurtle = turtle;
      }
    }
    RCLCPP_INFO(
        this->get_logger(),
        ("_targetTurtle x: " + std::to_string(_targetTurtle.x)).c_str());
    RCLCPP_INFO(
        this->get_logger(),
        ("_targetTurtle y: " + std::to_string(_targetTurtle.y)).c_str());
  }

  float calculateDistance(SpawnLocation *turtle) {
    return hypot((*turtle).x - _turtlePose.x, (*turtle).y - _turtlePose.y);
  }

  /**
   * @brief
   *
   */
  void hunt() {
    try {
      if (_turtles.size() == 0) return;
      findNearestTurtle();
      Twist turtleTwist;
      generateMotion(&turtleTwist);
      moveToTarget(&turtleTwist);
    } catch (const std::exception &ex) {
      RCLCPP_INFO(this->get_logger(), ex.what());
      throw;
    }
  }

  /**
   * @brief
   *
   * @return Twist
   */
  void generateMotion(Twist *turtleTwist) {
    float distance = calculateDistance(&_targetTurtle);

    RCLCPP_INFO(this->get_logger(),
                ("distance: " + std::to_string(distance)).c_str());

    double angleTan = (abs(_targetTurtle.y - _turtlePose.y) /
                       abs(_targetTurtle.x - _turtlePose.x));

    double tanAngle = atan(angleTan);

    RCLCPP_INFO(this->get_logger(),
                ("tanAngle: " + std::to_string(tanAngle)).c_str());

    if (_targetTurtle.x < _turtlePose.x) {
      if (_targetTurtle.y < _turtlePose.y) {
        tanAngle -= M_PI;
      } else {
        tanAngle = M_PI - tanAngle;
      }
    } else {
      if (_targetTurtle.y < _turtlePose.y) {
        tanAngle = -tanAngle;
      }
    }

    RCLCPP_INFO(this->get_logger(),
                ("tanAngle: " + std::to_string(tanAngle)).c_str());

    double rotation = tanAngle - _turtlePose.theta;
    rotation = remainder(rotation, 2.0 * M_PI);

    RCLCPP_INFO(
        this->get_logger(),
        ("_turtlePose.theta: " + std::to_string(_turtlePose.theta)).c_str());

    RCLCPP_INFO(this->get_logger(),
                ("rotation: " + std::to_string(rotation)).c_str());

    (*turtleTwist).angular.z = rotation;
    (*turtleTwist).linear.x =
        distance > _turtle_speed ? _turtle_speed : 2.0 * distance / 3.0;
  }

  /**
   * @brief
   *
   * @param turtleTwist
   */
  void moveToTarget(Twist *turtleTwist) {
    _turtleDriverPublisher->publish(*turtleTwist);
  }

  /**
   * @brief Create deadTurle message.
   * Populate turtle_name in message.
   * Find _targetTurtle in vector.
   * Remove from vector.
   * Publish deadturtle name.
   *
   * @param pose
   */
  void updatePose(Pose pose) {
    _turtlePose = pose;
    if (_targetTurtle.turtle_name.empty()) return;

    if (std::round(calculateDistance(&_targetTurtle)) == 0.0) {
      auto deadTurtle = DeadTurtle();
      deadTurtle.turtle_name = _targetTurtle.turtle_name;

      std::lock_guard<std::mutex> lock(threadMutex);
      auto iter = std::find_if(
          _turtles.begin(), _turtles.end(), [=](SpawnLocation &spawnLocation) {
            return spawnLocation.turtle_name == _targetTurtle.turtle_name;
          });

      if (iter != _turtles.end()) {
        _turtles.erase(iter);
        _targetTurtle = SpawnLocation();
        _deadTurtlePublisher->publish(deadTurtle);
      }
    }
  }

  float _turtle_speed;
  std::mutex threadMutex;
  Pose _turtlePose;
  SpawnLocation _targetTurtle;
  std::vector<SpawnLocation> _turtles;
  std::string _nodeName;
  rclcpp::Subscription<Pose>::SharedPtr _turtlePoseListener;
  rclcpp::Subscription<SpawnLocation>::SharedPtr _turtleSpawnListener;
  rclcpp::Publisher<DeadTurtle>::SharedPtr _deadTurtlePublisher;
  rclcpp::Publisher<Twist>::SharedPtr _turtleDriverPublisher;
  rclcpp::TimerBase::SharedPtr _decisionTimer;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<TurtleControlNode>(std::string("turtle_control"));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}