#ifndef SIM_GAZEBO_WEATHER_PLUGIN_HPP_
#define SIM_GAZEBO_WEATHER_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>
#include <random>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Wind.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/Light.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/wind.pb.h>
#include <ignition/msgs/param.pb.h>
#include <ignition/msgs/param_v.pb.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sim_msgs/msg/weather_state.hpp>
#include <sim_msgs/srv/set_weather.hpp>

namespace sim_gazebo
{

class WeatherPlugin : public ignition::gazebo::System,
                      public ignition::gazebo::ISystemConfigure,
                      public ignition::gazebo::ISystemPreUpdate,
                      public ignition::gazebo::ISystemUpdate
{
public:
  /// \brief Constructor
  WeatherPlugin();

  /// \brief Destructor
  ~WeatherPlugin() override = default;

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
  /// \brief Callback for the set weather service
  void SetWeatherCallback(
      const std::shared_ptr<sim_msgs::srv::SetWeather::Request> request,
      std::shared_ptr<sim_msgs::srv::SetWeather::Response> response);

  /// \brief Update wind effects
  void UpdateWind(const ignition::gazebo::UpdateInfo &_info,
                  ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Update rain effects
  void UpdateRain(const ignition::gazebo::UpdateInfo &_info,
                  ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Update fog effects
  void UpdateFog(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Update time of day effects
  void UpdateTimeOfDay(const ignition::gazebo::UpdateInfo &_info,
                       ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Update GPS multipath effects
  void UpdateGPSMultipath(const ignition::gazebo::UpdateInfo &_info,
                          ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Generate turbulence for wind
  ignition::math::Vector3d GenerateTurbulence();

  /// \brief ROS node
  rclcpp::Node::SharedPtr ros_node_;

  /// \brief Weather state publisher
  rclcpp::Publisher<sim_msgs::msg::WeatherState>::SharedPtr weather_pub_;

  /// \brief Set weather service
  rclcpp::Service<sim_msgs::srv::SetWeather>::SharedPtr weather_srv_;

  /// \brief Ignition transport node
  ignition::transport::Node ignition_node_;

  /// \brief Wind publisher
  ignition::transport::Node::Publisher wind_pub_;

  /// \brief Visual effects publisher
  ignition::transport::Node::Publisher visual_pub_;

  /// \brief Light effects publisher
  ignition::transport::Node::Publisher light_pub_;

  /// \brief World entity
  ignition::gazebo::Entity world_entity_;

  /// \brief World name
  std::string world_name_;

  /// \brief Update rate
  double update_rate_{10.0};

  /// \brief Last update time
  std::chrono::steady_clock::time_point last_update_time_;

  /// \brief Weather type
  std::string weather_type_{"clear"};

  /// \brief Wind speed (m/s)
  double wind_speed_{0.0};

  /// \brief Wind direction (degrees)
  double wind_direction_{0.0};

  /// \brief Wind gust factor
  double wind_gust_factor_{0.0};

  /// \brief Wind turbulence intensity
  double wind_turbulence_intensity_{0.0};

  /// \brief Rain intensity (0-1)
  double rain_intensity_{0.0};

  /// \brief Fog density (0-1)
  double fog_density_{0.0};

  /// \brief Time of day (0-24)
  double time_of_day_{12.0};

  /// \brief GPS multipath factor (0-1)
  double gps_multipath_factor_{0.0};

  /// \brief Random number generator
  std::mt19937 rng_;

  /// \brief Normal distribution for turbulence
  std::normal_distribution<double> normal_dist_;
};

}  // namespace sim_gazebo

#endif  // SIM_GAZEBO_WEATHER_PLUGIN_HPP_
