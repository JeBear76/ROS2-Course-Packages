#include <random>

#include "assignment_interfaces/msg/dead_turtle.hpp"
#include "assignment_interfaces/msg/spawn_location.hpp"
#include "assignment_interfaces/msg/turtle_control_ready.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"

using assignment_interfaces::msg::DeadTurtle;
using assignment_interfaces::msg::SpawnLocation;
using assignment_interfaces::msg::TurtleControlReady;
using turtlesim::srv::Kill;
using turtlesim::srv::Spawn;

class TurtleSpawnNode : public rclcpp::Node {
 public:
  TurtleSpawnNode(std::string nodeName) : Node(nodeName) {
    _nodeName = nodeName;
    _distribution = std::uniform_real_distribution<float>(0, 10);
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    _generator = std::default_random_engine(seed);

    this->declare_parameter<int32_t>("spawn_frequency", 5);
    int spawnFrequency = this->get_parameter("spawn_frequency").as_int();

    this->declare_parameter<bool>("spawn_immediately", false);
    _turtleSpawner = this->create_client<Spawn>("spawn");

    _spawnLocationPublisher =
        this->create_publisher<SpawnLocation>("turtle_spawn_location", 10);

    _spawnTimer =
        this->create_wall_timer(std::chrono::seconds(spawnFrequency),
                                std::bind(&TurtleSpawnNode::spawnTurtle, this));

    _turtleKiller = this->create_client<Kill>("kill");
    _deadTurtleListener = this->create_subscription<DeadTurtle>(
        "dead_turtle", 10,
        std::bind(&TurtleSpawnNode::killTurtle, this, std::placeholders::_1));

    _turtleControlReadyListener = this->create_subscription<TurtleControlReady>(
        "turtle_control_ready", 10, [this](TurtleControlReady) {
          RCLCPP_INFO(this->get_logger(), "Received TurtleControlReady...");
          for (auto &&turtle : _turtles) {
            _spawnLocationPublisher->publish(turtle);
          }
        });

    if (this->get_parameter("spawn_immediately").as_bool()) spawnTurtle();

    RCLCPP_INFO(this->get_logger(),
                (std::string("ROS2 ") + nodeName + std::string(" C++ started."))
                    .c_str());
  }

 private:
  void killTurtle(const DeadTurtle deadTurtle) {
    _turtleKiller->prune_pending_requests();

    std::lock_guard<std::mutex> lock(threadMutex);
    _worker.push_back(std::thread([this, deadTurtle]() {
      while (!_turtleKiller->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for service...");
      }

      auto request = std::make_shared<Kill::Request>();

      request->name = deadTurtle.turtle_name;

      _turtleKiller->async_send_request(
          request, [this, request](rclcpp::Client<Kill>::SharedFuture future) {
            try {
              auto response = future.get();
              RCLCPP_INFO(this->get_logger(),
                          (std::string("killed ") + request->name).c_str());
              auto spawnLocation = SpawnLocation();
              auto iter = std::find_if(
                  _turtles.begin(), _turtles.end(),
                  [&request](SpawnLocation &spawnLocation) {
                    return spawnLocation.turtle_name == request->name;
                  });
              if (iter != _turtles.end()) _turtles.erase(iter);
            } catch (const std::exception &e) {
              RCLCPP_ERROR(this->get_logger(), "Error calling service");
            }
            auto _ = std::async(std::bind(&TurtleSpawnNode::removeThread, this,
                                          std::this_thread::get_id()));
          });
    }));
  }

  void spawnTurtle() {
    _turtleSpawner->prune_pending_requests();

    std::lock_guard<std::mutex> lock(threadMutex);
    _worker.push_back(std::thread([this]() {
      while (!_turtleSpawner->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for service...");
      }

      auto request = std::make_shared<Spawn::Request>();

      request->x = _distribution(_generator);
      request->y = _distribution(_generator);

      auto fnr = _turtleSpawner->async_send_request(
          request, [this, request](rclcpp::Client<Spawn>::SharedFuture future) {
            try {
              auto response = future.get();
              RCLCPP_INFO(this->get_logger(),
                          (std::string("spawned ") + response->name).c_str());
              auto spawnLocation = SpawnLocation();
              spawnLocation.turtle_name = response->name;
              spawnLocation.x = request->x;
              spawnLocation.y = request->y;

              _turtles.push_back(spawnLocation);

              _spawnLocationPublisher->publish(spawnLocation);
            } catch (const std::exception &e) {
              RCLCPP_ERROR(this->get_logger(), "Error calling service");
            }
            auto _ = std::async(std::bind(&TurtleSpawnNode::removeThread, this,
                                          std::this_thread::get_id()));
          });
    }));
  }

  void removeThread(std::thread::id id) {
    std::lock_guard<std::mutex> lock(threadMutex);
    auto iter =
        std::find_if(_worker.begin(), _worker.end(),
                     [=](std::thread &t) { return (t.get_id() == id); });
    if (iter != _worker.end()) {
      iter->detach();
      _worker.erase(iter);
    }
  }

  std::mutex threadMutex;
  std::uniform_real_distribution<float> _distribution;
  std::default_random_engine _generator;
  std::vector<std::thread> _worker;
  std::vector<SpawnLocation> _turtles;
  std::string _nodeName;
  rclcpp::Client<Spawn>::SharedPtr _turtleSpawner;
  rclcpp::TimerBase::SharedPtr _spawnTimer;
  rclcpp::Publisher<SpawnLocation>::SharedPtr _spawnLocationPublisher;
  rclcpp::Client<Kill>::SharedPtr _turtleKiller;
  rclcpp::Subscription<DeadTurtle>::SharedPtr _deadTurtleListener;
  rclcpp::Subscription<TurtleControlReady>::SharedPtr
      _turtleControlReadyListener;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleSpawnNode>(std::string("turtle_spawn"));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}