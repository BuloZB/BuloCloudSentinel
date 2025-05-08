#include "sim_gazebo/traffic_plugin.hpp"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/common/Profiler.hh>

namespace sim_gazebo
{

//////////////////////////////////////////////////
TrafficPlugin::TrafficPlugin()
: rng_(std::random_device()())
{
  // Initialize vehicle models
  vehicle_models_ = {
    "sedan",
    "suv",
    "hatchback",
    "truck",
    "bus",
    "motorcycle"
  };

  // Initialize pedestrian models
  pedestrian_models_ = {
    "pedestrian_walking",
    "pedestrian_standing",
    "pedestrian_running"
  };
}

//////////////////////////////////////////////////
void TrafficPlugin::Configure(const ignition::gazebo::Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             ignition::gazebo::EntityComponentManager &_ecm,
                             ignition::gazebo::EventManager &_eventMgr)
{
  world_entity_ = _entity;

  // Get the world name
  auto worldName = _ecm.Component<ignition::gazebo::components::Name>(_entity);
  if (worldName)
  {
    world_name_ = worldName->Data();
  }

  // Get parameters from SDF
  if (_sdf->HasElement("update_rate"))
  {
    update_rate_ = _sdf->Get<double>("update_rate");
  }

  if (_sdf->HasElement("vehicle_density"))
  {
    vehicle_density_ = _sdf->Get<double>("vehicle_density");
  }

  if (_sdf->HasElement("pedestrian_density"))
  {
    pedestrian_density_ = _sdf->Get<double>("pedestrian_density");
  }

  if (_sdf->HasElement("max_vehicles"))
  {
    max_vehicles_ = _sdf->Get<int>("max_vehicles");
  }

  if (_sdf->HasElement("max_pedestrians"))
  {
    max_pedestrians_ = _sdf->Get<int>("max_pedestrians");
  }

  if (_sdf->HasElement("road_network_file"))
  {
    road_network_file_ = _sdf->Get<std::string>("road_network_file");
  }

  // Initialize ROS node
  ros_node_ = std::make_shared<rclcpp::Node>("traffic_plugin");

  // Create publishers
  traffic_pub_ = ros_node_->create_publisher<sim_msgs::msg::TrafficState>(
    "/sim/traffic/state", 10);

  // Create services
  traffic_srv_ = ros_node_->create_service<sim_msgs::srv::SetTrafficDensity>(
    "/sim/traffic/set_density",
    std::bind(&TrafficPlugin::SetTrafficDensityCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // Load road network
  if (!road_network_file_.empty())
  {
    LoadRoadNetwork(road_network_file_);
  }
  else
  {
    RCLCPP_WARN(ros_node_->get_logger(), "No road network file specified");
  }

  // Initialize last update time
  last_update_time_ = std::chrono::steady_clock::now();
}

//////////////////////////////////////////////////
void TrafficPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                             ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("TrafficPlugin::PreUpdate");

  // Check if it's time to update
  auto current_time = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
    current_time - last_update_time_).count();

  if (elapsed < 1.0 / update_rate_)
  {
    return;
  }

  last_update_time_ = current_time;

  // Lock mutex for thread safety
  std::lock_guard<std::mutex> lock(mutex_);

  // Spawn new vehicles and pedestrians based on density
  int target_vehicles = static_cast<int>(vehicle_density_ * max_vehicles_);
  int active_vehicles = 0;
  for (const auto &vehicle : vehicles_)
  {
    if (vehicle.active)
    {
      active_vehicles++;
    }
  }

  int vehicles_to_spawn = target_vehicles - active_vehicles;
  for (int i = 0; i < vehicles_to_spawn && vehicles_.size() < max_vehicles_; ++i)
  {
    SpawnVehicle(_ecm);
  }

  int target_pedestrians = static_cast<int>(pedestrian_density_ * max_pedestrians_);
  int active_pedestrians = 0;
  for (const auto &pedestrian : pedestrians_)
  {
    if (pedestrian.active)
    {
      active_pedestrians++;
    }
  }

  int pedestrians_to_spawn = target_pedestrians - active_pedestrians;
  for (int i = 0; i < pedestrians_to_spawn && pedestrians_.size() < max_pedestrians_; ++i)
  {
    SpawnPedestrian(_ecm);
  }
}

//////////////////////////////////////////////////
void TrafficPlugin::Update(const ignition::gazebo::UpdateInfo &_info,
                          ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("TrafficPlugin::Update");

  // Lock mutex for thread safety
  std::lock_guard<std::mutex> lock(mutex_);

  // Update traffic lights
  UpdateTrafficLights(_info, _ecm);

  // Update vehicles
  UpdateVehicles(_info, _ecm);

  // Update pedestrians
  UpdatePedestrians(_info, _ecm);

  // Check for collisions
  CheckCollisions(_ecm);

  // Publish traffic state
  auto msg = std::make_unique<sim_msgs::msg::TrafficState>();
  msg->vehicle_count = 0;
  msg->pedestrian_count = 0;
  
  for (const auto &vehicle : vehicles_)
  {
    if (vehicle.active)
    {
      msg->vehicle_count++;
    }
  }
  
  for (const auto &pedestrian : pedestrians_)
  {
    if (pedestrian.active)
    {
      msg->pedestrian_count++;
    }
  }
  
  msg->vehicle_density = vehicle_density_;
  msg->pedestrian_density = pedestrian_density_;
  
  traffic_pub_->publish(std::move(msg));
}

//////////////////////////////////////////////////
void TrafficPlugin::SetTrafficDensityCallback(
    const std::shared_ptr<sim_msgs::srv::SetTrafficDensity::Request> request,
    std::shared_ptr<sim_msgs::srv::SetTrafficDensity::Response> response)
{
  // Lock mutex for thread safety
  std::lock_guard<std::mutex> lock(mutex_);

  // Update densities
  vehicle_density_ = std::clamp(request->vehicle_density, 0.0, 1.0);
  pedestrian_density_ = std::clamp(request->pedestrian_density, 0.0, 1.0);

  response->success = true;
  response->message = "Traffic density updated successfully";
}

//////////////////////////////////////////////////
void TrafficPlugin::LoadRoadNetwork(const std::string &filename)
{
  RCLCPP_INFO(ros_node_->get_logger(), "Loading road network from %s", filename.c_str());

  // TODO: Implement road network loading from file
  // For now, create a simple grid road network

  // Create a simple grid of roads
  const int grid_size = 5;
  const double road_length = 50.0;
  const double road_width = 8.0;
  const int lanes_per_road = 2;
  const double speed_limit = 13.89;  // 50 km/h in m/s

  // Create road segments
  for (int i = 0; i < grid_size; ++i)
  {
    for (int j = 0; j < grid_size - 1; ++j)
    {
      // Horizontal road segment
      RoadSegment horizontal_segment;
      horizontal_segment.id = road_segments_.size();
      horizontal_segment.num_lanes = lanes_per_road;
      horizontal_segment.speed_limit = speed_limit;

      // Add waypoints
      double start_x = -road_length * (grid_size - 1) / 2.0 + j * road_length;
      double start_y = -road_length * (grid_size - 1) / 2.0 + i * road_length;
      double end_x = start_x + road_length;
      double end_y = start_y;

      horizontal_segment.waypoints.push_back(ignition::math::Vector3d(start_x, start_y, 0.05));
      horizontal_segment.waypoints.push_back(ignition::math::Vector3d(end_x, end_y, 0.05));

      road_segments_.push_back(horizontal_segment);

      // Vertical road segment
      RoadSegment vertical_segment;
      vertical_segment.id = road_segments_.size();
      vertical_segment.num_lanes = lanes_per_road;
      vertical_segment.speed_limit = speed_limit;

      // Add waypoints
      start_x = -road_length * (grid_size - 1) / 2.0 + i * road_length;
      start_y = -road_length * (grid_size - 1) / 2.0 + j * road_length;
      end_x = start_x;
      end_y = start_y + road_length;

      vertical_segment.waypoints.push_back(ignition::math::Vector3d(start_x, start_y, 0.05));
      vertical_segment.waypoints.push_back(ignition::math::Vector3d(end_x, end_y, 0.05));

      road_segments_.push_back(vertical_segment);
    }
  }

  // Connect road segments
  for (auto &segment : road_segments_)
  {
    // Find connected segments
    for (auto &other_segment : road_segments_)
    {
      if (segment.id == other_segment.id)
      {
        continue;
      }

      // Check if the end of this segment connects to the start of the other segment
      if ((segment.waypoints.back() - other_segment.waypoints.front()).Length() < 1.0)
      {
        segment.connected_segments.push_back(other_segment.id);
      }
    }
  }

  // Create sidewalk segments
  for (const auto &road : road_segments_)
  {
    // Create sidewalks on both sides of the road
    for (int side = 0; side < 2; ++side)
    {
      SidewalkSegment sidewalk;
      sidewalk.id = sidewalk_segments_.size();
      sidewalk.width = 2.0;

      // Offset from the road
      double offset = (road.num_lanes * road_width / 2.0 + sidewalk.width / 2.0) * (side == 0 ? 1.0 : -1.0);

      // Calculate perpendicular direction
      ignition::math::Vector3d road_dir = road.waypoints[1] - road.waypoints[0];
      road_dir.Normalize();
      ignition::math::Vector3d perp_dir(-road_dir.Y(), road_dir.X(), 0);

      // Add waypoints
      for (const auto &road_point : road.waypoints)
      {
        sidewalk.waypoints.push_back(road_point + perp_dir * offset);
      }

      sidewalk_segments_.push_back(sidewalk);
    }
  }

  // Connect sidewalk segments
  for (auto &segment : sidewalk_segments_)
  {
    // Find connected segments
    for (auto &other_segment : sidewalk_segments_)
    {
      if (segment.id == other_segment.id)
      {
        continue;
      }

      // Check if the end of this segment connects to the start of the other segment
      if ((segment.waypoints.back() - other_segment.waypoints.front()).Length() < 1.0)
      {
        segment.connected_segments.push_back(other_segment.id);
      }
    }
  }

  // Create traffic lights at intersections
  for (int i = 0; i < grid_size; ++i)
  {
    for (int j = 0; j < grid_size; ++j)
    {
      double x = -road_length * (grid_size - 1) / 2.0 + i * road_length;
      double y = -road_length * (grid_size - 1) / 2.0 + j * road_length;

      TrafficLight light;
      light.model_name = "traffic_light_" + std::to_string(traffic_lights_.size());
      light.pose = ignition::math::Pose3d(x, y, 5.0, 0, 0, 0);
      light.state = "green";
      light.timer = 0.0;

      // Find road segments controlled by this light
      for (const auto &segment : road_segments_)
      {
        // Check if the start of the segment is near this intersection
        if ((segment.waypoints.front() - ignition::math::Vector3d(x, y, 0)).Length() < 1.0)
        {
          light.controlled_road_segments.push_back(segment.id);
        }
      }

      traffic_lights_.push_back(light);
    }
  }

  RCLCPP_INFO(ros_node_->get_logger(), "Created %zu road segments, %zu sidewalk segments, and %zu traffic lights",
              road_segments_.size(), sidewalk_segments_.size(), traffic_lights_.size());
}

//////////////////////////////////////////////////
void TrafficPlugin::SpawnVehicle(ignition::gazebo::EntityComponentManager &_ecm)
{
  if (road_segments_.empty())
  {
    return;
  }

  // Select a random road segment
  std::uniform_int_distribution<int> road_dist(0, road_segments_.size() - 1);
  int road_index = road_dist(rng_);
  const auto &road = road_segments_[road_index];

  // Select a random lane
  std::uniform_int_distribution<int> lane_dist(0, road.num_lanes - 1);
  int lane = lane_dist(rng_);

  // Select a random position along the road
  std::uniform_real_distribution<double> pos_dist(0.0, 1.0);
  double pos = pos_dist(rng_);

  // Calculate the position
  ignition::math::Vector3d start = road.waypoints.front();
  ignition::math::Vector3d end = road.waypoints.back();
  ignition::math::Vector3d dir = end - start;
  dir.Normalize();

  // Calculate lane offset
  double lane_width = 4.0;  // meters
  double lane_offset = (lane - (road.num_lanes - 1) / 2.0) * lane_width;
  ignition::math::Vector3d lane_dir(-dir.Y(), dir.X(), 0);

  ignition::math::Vector3d position = start + dir * pos * (end - start).Length() + lane_dir * lane_offset;

  // Calculate orientation
  double yaw = std::atan2(dir.Y(), dir.X());

  // Select a random vehicle model
  std::uniform_int_distribution<int> model_dist(0, vehicle_models_.size() - 1);
  std::string model_type = vehicle_models_[model_dist(rng_)];

  // Create a new vehicle
  Vehicle vehicle;
  vehicle.model_name = model_type + "_" + std::to_string(vehicles_.size());
  vehicle.type = model_type;
  vehicle.pose = ignition::math::Pose3d(position.X(), position.Y(), position.Z(), 0, 0, yaw);
  vehicle.velocity = ignition::math::Vector3d(0, 0, 0);
  vehicle.speed = std::uniform_real_distribution<double>(5.0, road.speed_limit)(rng_);
  vehicle.active = true;
  vehicle.lane = lane;
  vehicle.road_segment = road_index;
  vehicle.waypoint_index = 0;
  vehicle.target = GetNextVehicleWaypoint(vehicle);

  vehicles_.push_back(vehicle);

  // TODO: Actually spawn the vehicle in Gazebo
  // For now, we'll just simulate the vehicles without visual representation
  RCLCPP_DEBUG(ros_node_->get_logger(), "Spawned vehicle %s at (%f, %f, %f)",
               vehicle.model_name.c_str(), position.X(), position.Y(), position.Z());
}

//////////////////////////////////////////////////
void TrafficPlugin::SpawnPedestrian(ignition::gazebo::EntityComponentManager &_ecm)
{
  if (sidewalk_segments_.empty())
  {
    return;
  }

  // Select a random sidewalk segment
  std::uniform_int_distribution<int> sidewalk_dist(0, sidewalk_segments_.size() - 1);
  int sidewalk_index = sidewalk_dist(rng_);
  const auto &sidewalk = sidewalk_segments_[sidewalk_index];

  // Select a random position along the sidewalk
  std::uniform_real_distribution<double> pos_dist(0.0, 1.0);
  double pos = pos_dist(rng_);

  // Calculate the position
  ignition::math::Vector3d start = sidewalk.waypoints.front();
  ignition::math::Vector3d end = sidewalk.waypoints.back();
  ignition::math::Vector3d dir = end - start;
  dir.Normalize();

  ignition::math::Vector3d position = start + dir * pos * (end - start).Length();

  // Calculate orientation
  double yaw = std::atan2(dir.Y(), dir.X());

  // Select a random pedestrian model
  std::uniform_int_distribution<int> model_dist(0, pedestrian_models_.size() - 1);
  std::string model_type = pedestrian_models_[model_dist(rng_)];

  // Create a new pedestrian
  Pedestrian pedestrian;
  pedestrian.model_name = model_type + "_" + std::to_string(pedestrians_.size());
  pedestrian.type = model_type;
  pedestrian.pose = ignition::math::Pose3d(position.X(), position.Y(), position.Z(), 0, 0, yaw);
  pedestrian.velocity = ignition::math::Vector3d(0, 0, 0);
  pedestrian.speed = std::uniform_real_distribution<double>(0.5, 2.0)(rng_);
  pedestrian.active = true;
  pedestrian.sidewalk_segment = sidewalk_index;
  pedestrian.waypoint_index = 0;
  pedestrian.target = GetNextPedestrianWaypoint(pedestrian);

  pedestrians_.push_back(pedestrian);

  // TODO: Actually spawn the pedestrian in Gazebo
  // For now, we'll just simulate the pedestrians without visual representation
  RCLCPP_DEBUG(ros_node_->get_logger(), "Spawned pedestrian %s at (%f, %f, %f)",
               pedestrian.model_name.c_str(), position.X(), position.Y(), position.Z());
}

//////////////////////////////////////////////////
void TrafficPlugin::UpdateVehicles(const ignition::gazebo::UpdateInfo &_info,
                                  ignition::gazebo::EntityComponentManager &_ecm)
{
  double dt = _info.dt.count();

  for (auto &vehicle : vehicles_)
  {
    if (!vehicle.active)
    {
      continue;
    }

    // Check if we've reached the target
    ignition::math::Vector3d pos(vehicle.pose.Pos());
    ignition::math::Vector3d to_target = vehicle.target - pos;
    to_target.Z() = 0;  // Ignore Z component

    if (to_target.Length() < 1.0)
    {
      // Get next waypoint
      vehicle.target = GetNextVehicleWaypoint(vehicle);
      
      // If no valid target, deactivate the vehicle
      if (vehicle.target == ignition::math::Vector3d::Zero)
      {
        vehicle.active = false;
        continue;
      }
      
      // Recalculate direction to target
      to_target = vehicle.target - pos;
      to_target.Z() = 0;
    }

    // Check if the road segment is blocked by a red light
    if (IsRoadSegmentBlocked(vehicle.road_segment))
    {
      // Slow down and stop
      vehicle.speed = std::max(0.0, vehicle.speed - 5.0 * dt);
    }
    else
    {
      // Accelerate to target speed
      double target_speed = road_segments_[vehicle.road_segment].speed_limit;
      vehicle.speed = std::min(target_speed, vehicle.speed + 2.0 * dt);
    }

    // Calculate direction and update position
    to_target.Normalize();
    vehicle.velocity = to_target * vehicle.speed;
    
    // Update position
    pos += vehicle.velocity * dt;
    
    // Update orientation
    double yaw = std::atan2(vehicle.velocity.Y(), vehicle.velocity.X());
    
    // Update pose
    vehicle.pose = ignition::math::Pose3d(pos.X(), pos.Y(), pos.Z(), 0, 0, yaw);
    
    // TODO: Update the actual Gazebo entity
  }
}

//////////////////////////////////////////////////
void TrafficPlugin::UpdatePedestrians(const ignition::gazebo::UpdateInfo &_info,
                                     ignition::gazebo::EntityComponentManager &_ecm)
{
  double dt = _info.dt.count();

  for (auto &pedestrian : pedestrians_)
  {
    if (!pedestrian.active)
    {
      continue;
    }

    // Check if we've reached the target
    ignition::math::Vector3d pos(pedestrian.pose.Pos());
    ignition::math::Vector3d to_target = pedestrian.target - pos;
    to_target.Z() = 0;  // Ignore Z component

    if (to_target.Length() < 0.5)
    {
      // Get next waypoint
      pedestrian.target = GetNextPedestrianWaypoint(pedestrian);
      
      // If no valid target, deactivate the pedestrian
      if (pedestrian.target == ignition::math::Vector3d::Zero)
      {
        pedestrian.active = false;
        continue;
      }
      
      // Recalculate direction to target
      to_target = pedestrian.target - pos;
      to_target.Z() = 0;
    }

    // Calculate direction and update position
    to_target.Normalize();
    pedestrian.velocity = to_target * pedestrian.speed;
    
    // Update position
    pos += pedestrian.velocity * dt;
    
    // Update orientation
    double yaw = std::atan2(pedestrian.velocity.Y(), pedestrian.velocity.X());
    
    // Update pose
    pedestrian.pose = ignition::math::Pose3d(pos.X(), pos.Y(), pos.Z(), 0, 0, yaw);
    
    // TODO: Update the actual Gazebo entity
  }
}

//////////////////////////////////////////////////
void TrafficPlugin::UpdateTrafficLights(const ignition::gazebo::UpdateInfo &_info,
                                       ignition::gazebo::EntityComponentManager &_ecm)
{
  double dt = _info.dt.count();

  for (auto &light : traffic_lights_)
  {
    // Update timer
    light.timer += dt;

    // State transitions
    if (light.state == "green" && light.timer > 30.0)
    {
      light.state = "yellow";
      light.timer = 0.0;
    }
    else if (light.state == "yellow" && light.timer > 5.0)
    {
      light.state = "red";
      light.timer = 0.0;
    }
    else if (light.state == "red" && light.timer > 30.0)
    {
      light.state = "green";
      light.timer = 0.0;
    }

    // TODO: Update the actual Gazebo entity
  }
}

//////////////////////////////////////////////////
void TrafficPlugin::CheckCollisions(ignition::gazebo::EntityComponentManager &_ecm)
{
  // Check vehicle-vehicle collisions
  for (size_t i = 0; i < vehicles_.size(); ++i)
  {
    if (!vehicles_[i].active)
    {
      continue;
    }

    for (size_t j = i + 1; j < vehicles_.size(); ++j)
    {
      if (!vehicles_[j].active)
      {
        continue;
      }

      // Simple collision check based on distance
      double distance = (vehicles_[i].pose.Pos() - vehicles_[j].pose.Pos()).Length();
      if (distance < 2.0)  // Assuming vehicle width is about 2 meters
      {
        RCLCPP_INFO(ros_node_->get_logger(), "Collision detected between vehicles %s and %s",
                   vehicles_[i].model_name.c_str(), vehicles_[j].model_name.c_str());
        
        // Deactivate both vehicles
        vehicles_[i].active = false;
        vehicles_[j].active = false;
      }
    }
  }

  // Check vehicle-pedestrian collisions
  for (auto &vehicle : vehicles_)
  {
    if (!vehicle.active)
    {
      continue;
    }

    for (auto &pedestrian : pedestrians_)
    {
      if (!pedestrian.active)
      {
        continue;
      }

      // Simple collision check based on distance
      double distance = (vehicle.pose.Pos() - pedestrian.pose.Pos()).Length();
      if (distance < 1.0)  // Assuming vehicle width is about 2 meters and pedestrian is about 0.5 meters
      {
        RCLCPP_INFO(ros_node_->get_logger(), "Collision detected between vehicle %s and pedestrian %s",
                   vehicle.model_name.c_str(), pedestrian.model_name.c_str());
        
        // Deactivate the pedestrian
        pedestrian.active = false;
      }
    }
  }
}

//////////////////////////////////////////////////
ignition::math::Vector3d TrafficPlugin::GetNextVehicleWaypoint(const Vehicle &vehicle)
{
  if (vehicle.road_segment >= road_segments_.size())
  {
    return ignition::math::Vector3d::Zero;
  }

  const auto &road = road_segments_[vehicle.road_segment];
  
  // If we're at the last waypoint, find a connected road segment
  if (vehicle.waypoint_index >= road.waypoints.size() - 1)
  {
    if (road.connected_segments.empty())
    {
      return ignition::math::Vector3d::Zero;  // No more waypoints
    }
    
    // Choose a random connected segment
    std::uniform_int_distribution<int> dist(0, road.connected_segments.size() - 1);
    int next_segment = road.connected_segments[dist(rng_)];
    
    // Update vehicle's road segment and reset waypoint index
    const_cast<Vehicle&>(vehicle).road_segment = next_segment;
    const_cast<Vehicle&>(vehicle).waypoint_index = 0;
    
    // Return the first waypoint of the new segment
    return road_segments_[next_segment].waypoints[0];
  }
  
  // Otherwise, return the next waypoint on the current road
  const_cast<Vehicle&>(vehicle).waypoint_index++;
  return road.waypoints[vehicle.waypoint_index];
}

//////////////////////////////////////////////////
ignition::math::Vector3d TrafficPlugin::GetNextPedestrianWaypoint(const Pedestrian &pedestrian)
{
  if (pedestrian.sidewalk_segment >= sidewalk_segments_.size())
  {
    return ignition::math::Vector3d::Zero;
  }

  const auto &sidewalk = sidewalk_segments_[pedestrian.sidewalk_segment];
  
  // If we're at the last waypoint, find a connected sidewalk segment
  if (pedestrian.waypoint_index >= sidewalk.waypoints.size() - 1)
  {
    if (sidewalk.connected_segments.empty())
    {
      return ignition::math::Vector3d::Zero;  // No more waypoints
    }
    
    // Choose a random connected segment
    std::uniform_int_distribution<int> dist(0, sidewalk.connected_segments.size() - 1);
    int next_segment = sidewalk.connected_segments[dist(rng_)];
    
    // Update pedestrian's sidewalk segment and reset waypoint index
    const_cast<Pedestrian&>(pedestrian).sidewalk_segment = next_segment;
    const_cast<Pedestrian&>(pedestrian).waypoint_index = 0;
    
    // Return the first waypoint of the new segment
    return sidewalk_segments_[next_segment].waypoints[0];
  }
  
  // Otherwise, return the next waypoint on the current sidewalk
  const_cast<Pedestrian&>(pedestrian).waypoint_index++;
  return sidewalk.waypoints[pedestrian.waypoint_index];
}

//////////////////////////////////////////////////
bool TrafficPlugin::IsRoadSegmentBlocked(int road_segment_id)
{
  for (const auto &light : traffic_lights_)
  {
    if (light.state == "red")
    {
      for (int controlled_segment : light.controlled_road_segments)
      {
        if (controlled_segment == road_segment_id)
        {
          return true;
        }
      }
    }
  }
  
  return false;
}

// Register the plugin
IGNITION_ADD_PLUGIN(
  sim_gazebo::TrafficPlugin,
  ignition::gazebo::System,
  sim_gazebo::TrafficPlugin::ISystemConfigure,
  sim_gazebo::TrafficPlugin::ISystemPreUpdate,
  sim_gazebo::TrafficPlugin::ISystemUpdate)

}  // namespace sim_gazebo
