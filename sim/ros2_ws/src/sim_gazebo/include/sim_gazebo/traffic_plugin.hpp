#ifndef SIM_GAZEBO_TRAFFIC_PLUGIN_HPP_
#define SIM_GAZEBO_TRAFFIC_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <random>
#include <mutex>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sim_msgs/msg/traffic_state.hpp>
#include <sim_msgs/srv/set_traffic_density.hpp>

namespace sim_gazebo
{

struct Vehicle
{
  std::string model_name;
  std::string type;
  ignition::math::Pose3d pose;
  ignition::math::Vector3d velocity;
  ignition::math::Vector3d target;
  double speed;
  bool active;
  int lane;
  int road_segment;
  int waypoint_index;
};

struct Pedestrian
{
  std::string model_name;
  std::string type;
  ignition::math::Pose3d pose;
  ignition::math::Vector3d velocity;
  ignition::math::Vector3d target;
  double speed;
  bool active;
  int sidewalk_segment;
  int waypoint_index;
};

struct RoadSegment
{
  int id;
  std::vector<ignition::math::Vector3d> waypoints;
  std::vector<int> connected_segments;
  int num_lanes;
  double speed_limit;
};

struct SidewalkSegment
{
  int id;
  std::vector<ignition::math::Vector3d> waypoints;
  std::vector<int> connected_segments;
  double width;
};

struct TrafficLight
{
  std::string model_name;
  ignition::math::Pose3d pose;
  std::string state;  // "red", "yellow", "green"
  double timer;
  std::vector<int> controlled_road_segments;
};

class TrafficPlugin : public ignition::gazebo::System,
                      public ignition::gazebo::ISystemConfigure,
                      public ignition::gazebo::ISystemPreUpdate,
                      public ignition::gazebo::ISystemUpdate
{
public:
  /// \brief Constructor
  TrafficPlugin();

  /// \brief Destructor
  ~TrafficPlugin() override = default;

  // Documentation inherited
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) override;

  // Documentation inherited
  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override;

  // Documentation inherited
  void Update(const ignition::gazebo::UpdateInfo &_info,
              ignition::gazebo::EntityComponentManager &_ecm) override;

private:
  /// \brief Callback for the set traffic density service
  void SetTrafficDensityCallback(
      const std::shared_ptr<sim_msgs::srv::SetTrafficDensity::Request> request,
      std::shared_ptr<sim_msgs::srv::SetTrafficDensity::Response> response);

  /// \brief Load road network from file
  void LoadRoadNetwork(const std::string &filename);

  /// \brief Spawn a vehicle
  void SpawnVehicle(ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Spawn a pedestrian
  void SpawnPedestrian(ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Update vehicle positions
  void UpdateVehicles(const ignition::gazebo::UpdateInfo &_info,
                      ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Update pedestrian positions
  void UpdatePedestrians(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Update traffic lights
  void UpdateTrafficLights(const ignition::gazebo::UpdateInfo &_info,
                           ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Check for collisions
  void CheckCollisions(ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Get next waypoint for a vehicle
  ignition::math::Vector3d GetNextVehicleWaypoint(const Vehicle &vehicle);

  /// \brief Get next waypoint for a pedestrian
  ignition::math::Vector3d GetNextPedestrianWaypoint(const Pedestrian &pedestrian);

  /// \brief Check if a road segment is controlled by a red light
  bool IsRoadSegmentBlocked(int road_segment_id);

  /// \brief ROS node
  rclcpp::Node::SharedPtr ros_node_;

  /// \brief Traffic state publisher
  rclcpp::Publisher<sim_msgs::msg::TrafficState>::SharedPtr traffic_pub_;

  /// \brief Set traffic density service
  rclcpp::Service<sim_msgs::srv::SetTrafficDensity>::SharedPtr traffic_srv_;

  /// \brief Ignition transport node
  ignition::transport::Node ignition_node_;

  /// \brief World entity
  ignition::gazebo::Entity world_entity_;

  /// \brief World name
  std::string world_name_;

  /// \brief Update rate
  double update_rate_{10.0};

  /// \brief Last update time
  std::chrono::steady_clock::time_point last_update_time_;

  /// \brief Vehicle density (0-1)
  double vehicle_density_{0.5};

  /// \brief Pedestrian density (0-1)
  double pedestrian_density_{0.3};

  /// \brief Maximum number of vehicles
  int max_vehicles_{50};

  /// \brief Maximum number of pedestrians
  int max_pedestrians_{100};

  /// \brief Road network file
  std::string road_network_file_;

  /// \brief Road segments
  std::vector<RoadSegment> road_segments_;

  /// \brief Sidewalk segments
  std::vector<SidewalkSegment> sidewalk_segments_;

  /// \brief Traffic lights
  std::vector<TrafficLight> traffic_lights_;

  /// \brief Vehicles
  std::vector<Vehicle> vehicles_;

  /// \brief Pedestrians
  std::vector<Pedestrian> pedestrians_;

  /// \brief Vehicle models
  std::vector<std::string> vehicle_models_;

  /// \brief Pedestrian models
  std::vector<std::string> pedestrian_models_;

  /// \brief Random number generator
  std::mt19937 rng_;

  /// \brief Mutex for thread safety
  std::mutex mutex_;
};

}  // namespace sim_gazebo

#endif  // SIM_GAZEBO_TRAFFIC_PLUGIN_HPP_
