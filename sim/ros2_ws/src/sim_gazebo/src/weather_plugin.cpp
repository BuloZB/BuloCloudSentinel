#include "sim_gazebo/weather_plugin.hpp"

#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/msgs/visual.pb.h>
#include <ignition/msgs/light.pb.h>

namespace sim_gazebo
{

//////////////////////////////////////////////////
WeatherPlugin::WeatherPlugin()
: rng_(std::random_device()()),
  normal_dist_(0.0, 1.0)
{
}

//////////////////////////////////////////////////
void WeatherPlugin::Configure(const ignition::gazebo::Entity &_entity,
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

  if (_sdf->HasElement("weather_type"))
  {
    weather_type_ = _sdf->Get<std::string>("weather_type");
  }

  if (_sdf->HasElement("wind_speed"))
  {
    wind_speed_ = _sdf->Get<double>("wind_speed");
  }

  if (_sdf->HasElement("wind_direction"))
  {
    wind_direction_ = _sdf->Get<double>("wind_direction");
  }

  if (_sdf->HasElement("wind_gust_factor"))
  {
    wind_gust_factor_ = _sdf->Get<double>("wind_gust_factor");
  }

  if (_sdf->HasElement("wind_turbulence_intensity"))
  {
    wind_turbulence_intensity_ = _sdf->Get<double>("wind_turbulence_intensity");
  }

  if (_sdf->HasElement("rain_intensity"))
  {
    rain_intensity_ = _sdf->Get<double>("rain_intensity");
  }

  if (_sdf->HasElement("fog_density"))
  {
    fog_density_ = _sdf->Get<double>("fog_density");
  }

  if (_sdf->HasElement("time_of_day"))
  {
    time_of_day_ = _sdf->Get<double>("time_of_day");
  }

  if (_sdf->HasElement("gps_multipath_factor"))
  {
    gps_multipath_factor_ = _sdf->Get<double>("gps_multipath_factor");
  }

  // Initialize ROS node
  ros_node_ = std::make_shared<rclcpp::Node>("weather_plugin");

  // Create publishers
  weather_pub_ = ros_node_->create_publisher<sim_msgs::msg::WeatherState>(
    "/sim/weather/state", 10);

  // Create services
  weather_srv_ = ros_node_->create_service<sim_msgs::srv::SetWeather>(
    "/sim/weather/set",
    std::bind(&WeatherPlugin::SetWeatherCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // Initialize Ignition publishers
  wind_pub_ = ignition_node_.Advertise<ignition::msgs::Wind>(
    "/world/" + world_name_ + "/wind");

  visual_pub_ = ignition_node_.Advertise<ignition::msgs::Visual>(
    "/world/" + world_name_ + "/visual");

  light_pub_ = ignition_node_.Advertise<ignition::msgs::Light>(
    "/world/" + world_name_ + "/light");

  // Initialize last update time
  last_update_time_ = std::chrono::steady_clock::now();
}

//////////////////////////////////////////////////
void WeatherPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                             ignition::gazebo::EntityComponentManager &_ecm)
{
  // Check if it's time to update
  auto current_time = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
    current_time - last_update_time_).count();

  if (elapsed < 1.0 / update_rate_)
  {
    return;
  }

  last_update_time_ = current_time;

  // Update weather effects
  UpdateWind(_info, _ecm);
  UpdateRain(_info, _ecm);
  UpdateFog(_info, _ecm);
  UpdateTimeOfDay(_info, _ecm);
  UpdateGPSMultipath(_info, _ecm);

  // Publish weather state
  auto msg = std::make_unique<sim_msgs::msg::WeatherState>();
  msg->weather_type = weather_type_;
  msg->wind_speed = wind_speed_;
  msg->wind_direction = wind_direction_;
  msg->wind_gust_factor = wind_gust_factor_;
  msg->wind_turbulence_intensity = wind_turbulence_intensity_;
  msg->rain_intensity = rain_intensity_;
  msg->fog_density = fog_density_;
  msg->time_of_day = time_of_day_;
  msg->gps_multipath_factor = gps_multipath_factor_;
  weather_pub_->publish(std::move(msg));
}

//////////////////////////////////////////////////
void WeatherPlugin::Update(const ignition::gazebo::UpdateInfo &_info,
                          ignition::gazebo::EntityComponentManager &_ecm)
{
  // Nothing to do here, all updates are done in PreUpdate
}

//////////////////////////////////////////////////
void WeatherPlugin::SetWeatherCallback(
    const std::shared_ptr<sim_msgs::srv::SetWeather::Request> request,
    std::shared_ptr<sim_msgs::srv::SetWeather::Response> response)
{
  weather_type_ = request->weather_type;
  wind_speed_ = request->wind_speed;
  wind_direction_ = request->wind_direction;
  wind_gust_factor_ = request->wind_gust_factor;
  wind_turbulence_intensity_ = request->wind_turbulence_intensity;
  rain_intensity_ = request->rain_intensity;
  fog_density_ = request->fog_density;
  time_of_day_ = request->time_of_day;
  gps_multipath_factor_ = request->gps_multipath_factor;

  response->success = true;
  response->message = "Weather parameters updated successfully";
}

//////////////////////////////////////////////////
void WeatherPlugin::UpdateWind(const ignition::gazebo::UpdateInfo &_info,
                              ignition::gazebo::EntityComponentManager &_ecm)
{
  // Convert wind direction from degrees to radians
  double direction_rad = wind_direction_ * M_PI / 180.0;

  // Calculate wind vector components
  double wind_x = wind_speed_ * std::cos(direction_rad);
  double wind_y = wind_speed_ * std::sin(direction_rad);

  // Add turbulence if enabled
  ignition::math::Vector3d turbulence(0, 0, 0);
  if (wind_turbulence_intensity_ > 0)
  {
    turbulence = GenerateTurbulence();
  }

  // Add gusts if enabled
  double gust_factor = 1.0;
  if (wind_gust_factor_ > 0)
  {
    // Random gust based on gust factor
    double gust_probability = 0.05;  // 5% chance of gust per update
    if (std::uniform_real_distribution<double>(0.0, 1.0)(rng_) < gust_probability)
    {
      gust_factor = 1.0 + std::uniform_real_distribution<double>(
        0.0, wind_gust_factor_)(rng_);
    }
  }

  // Final wind vector
  ignition::math::Vector3d wind_vec(
    wind_x * gust_factor + turbulence.X(),
    wind_y * gust_factor + turbulence.Y(),
    turbulence.Z());

  // Publish wind message
  ignition::msgs::Wind wind_msg;
  ignition::msgs::Set(wind_msg.mutable_linear_velocity(), wind_vec);
  wind_pub_.Publish(wind_msg);
}

//////////////////////////////////////////////////
void WeatherPlugin::UpdateRain(const ignition::gazebo::UpdateInfo &_info,
                              ignition::gazebo::EntityComponentManager &_ecm)
{
  if (rain_intensity_ <= 0)
  {
    return;
  }

  // TODO: Implement rain particle effects
  // This would typically involve creating particle emitters in the scene
  // For now, we'll just adjust the scene's ambient light to simulate darker conditions

  // Darken the scene based on rain intensity
  double ambient_factor = 1.0 - (rain_intensity_ * 0.3);  // Reduce ambient light up to 30%

  // Find all lights in the scene
  auto lights = _ecm.EntitiesByComponents(
    ignition::gazebo::components::Light());

  for (const auto &light : lights)
  {
    // Get the light component
    auto lightComp = _ecm.Component<ignition::gazebo::components::Light>(light);
    if (!lightComp)
    {
      continue;
    }

    // Create a light message
    ignition::msgs::Light light_msg;
    light_msg.set_name(lightComp->Data().Name());
    light_msg.set_type(static_cast<ignition::msgs::Light::LightType>(
      lightComp->Data().Type()));

    // Adjust ambient light
    ignition::msgs::Color *ambient = light_msg.mutable_ambient();
    ambient->set_r(0.5 * ambient_factor);
    ambient->set_g(0.5 * ambient_factor);
    ambient->set_b(0.5 * ambient_factor);
    ambient->set_a(1.0);

    // Publish light message
    light_pub_.Publish(light_msg);
  }
}

//////////////////////////////////////////////////
void WeatherPlugin::UpdateFog(const ignition::gazebo::UpdateInfo &_info,
                             ignition::gazebo::EntityComponentManager &_ecm)
{
  if (fog_density_ <= 0)
  {
    return;
  }

  // TODO: Implement fog effects
  // This would typically involve setting fog parameters in the scene
  // For now, we'll just log the fog density
  RCLCPP_DEBUG(ros_node_->get_logger(), "Fog density: %f", fog_density_);
}

//////////////////////////////////////////////////
void WeatherPlugin::UpdateTimeOfDay(const ignition::gazebo::UpdateInfo &_info,
                                   ignition::gazebo::EntityComponentManager &_ecm)
{
  // Find the sun light
  auto lights = _ecm.EntitiesByComponents(
    ignition::gazebo::components::Light());

  for (const auto &light : lights)
  {
    // Get the light component
    auto lightComp = _ecm.Component<ignition::gazebo::components::Light>(light);
    if (!lightComp || lightComp->Data().Name() != "sun")
    {
      continue;
    }

    // Calculate sun position based on time of day
    // This is a simplified model - a real implementation would use astronomical calculations
    double sun_angle = (time_of_day_ - 12.0) * M_PI / 12.0;  // -π at midnight, 0 at noon, π at midnight
    
    // Calculate sun direction
    double x = -std::sin(sun_angle);
    double y = 0.1;  // slight offset from directly overhead
    double z = -std::cos(sun_angle);
    
    // Create a light message
    ignition::msgs::Light light_msg;
    light_msg.set_name(lightComp->Data().Name());
    light_msg.set_type(static_cast<ignition::msgs::Light::LightType>(
      lightComp->Data().Type()));
    
    // Set direction
    ignition::msgs::Vector3d *dir = light_msg.mutable_direction();
    dir->set_x(x);
    dir->set_y(y);
    dir->set_z(z);
    
    // Adjust light intensity based on time of day
    double time_factor = std::max(0.0, std::cos(sun_angle));
    
    // Set diffuse color (brighter during day)
    ignition::msgs::Color *diffuse = light_msg.mutable_diffuse();
    diffuse->set_r(0.8 * time_factor);
    diffuse->set_g(0.8 * time_factor);
    diffuse->set_b(0.8 * time_factor);
    diffuse->set_a(1.0);
    
    // Set specular color
    ignition::msgs::Color *specular = light_msg.mutable_specular();
    specular->set_r(0.2 * time_factor);
    specular->set_g(0.2 * time_factor);
    specular->set_b(0.2 * time_factor);
    specular->set_a(1.0);
    
    // Publish light message
    light_pub_.Publish(light_msg);
    
    // Also update the ambient light of the scene
    // TODO: Implement scene ambient light update
    break;
  }
}

//////////////////////////////////////////////////
void WeatherPlugin::UpdateGPSMultipath(const ignition::gazebo::UpdateInfo &_info,
                                      ignition::gazebo::EntityComponentManager &_ecm)
{
  if (gps_multipath_factor_ <= 0)
  {
    return;
  }

  // TODO: Implement GPS multipath effects
  // This would typically involve modifying GPS sensor noise parameters
  // For now, we'll just log the multipath factor
  RCLCPP_DEBUG(ros_node_->get_logger(), "GPS multipath factor: %f", gps_multipath_factor_);
}

//////////////////////////////////////////////////
ignition::math::Vector3d WeatherPlugin::GenerateTurbulence()
{
  // Generate random turbulence based on turbulence intensity
  double scale = wind_speed_ * wind_turbulence_intensity_;
  
  return ignition::math::Vector3d(
    normal_dist_(rng_) * scale,
    normal_dist_(rng_) * scale,
    normal_dist_(rng_) * scale * 0.5);  // Less turbulence in vertical direction
}

// Register the plugin
IGNITION_ADD_PLUGIN(
  sim_gazebo::WeatherPlugin,
  ignition::gazebo::System,
  sim_gazebo::WeatherPlugin::ISystemConfigure,
  sim_gazebo::WeatherPlugin::ISystemPreUpdate,
  sim_gazebo::WeatherPlugin::ISystemUpdate)

}  // namespace sim_gazebo
