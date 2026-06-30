#ifndef BICYCLE_TO_ACKERMANN_TRACTION_ADAPTER_HPP_
#define BICYCLE_TO_ACKERMANN_TRACTION_ADAPTER_HPP_

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "caddy_ai2_ros2_bicycle_to_ackermann_traction_adapter/bicycle_to_ackermann_traction_adapter_parameters.hpp"

namespace bicycle_to_ackermann_traction_adapter
{
class BicycleToAckermannTractionAdapter : public controller_interface::ChainableControllerInterface
{
public:
  BicycleToAckermannTractionAdapter();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;
  
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // name constants for output steering interfaces
  size_t nr_output_steer_itfs_ = 2;
  // name constants for input steering interfaces
  size_t nr_input_steer_itfs_ = 1;

  // Parameters
  std::shared_ptr<bicycle_to_ackermann_traction_adapter::ParamListener> bicycle_to_ackermann_traction_adapter_param_listener_;
  bicycle_to_ackermann_traction_adapter::Params bicycle_to_ackermann_traction_adapter_params_;

  // Operation mode
  bool by_reference_or_by_state_ = false;

  // Bicycle model parameters
  double wheelbase_;
  double track_width_;
  double wheel_radius_;

  // Steering angle
  double center_steering_angle_;

  // Traction velocities
  double right_traction_vel_;
  double left_traction_vel_;
  double center_traction_vel_;
  double input_linear_vel_;
  double transmission_factor_;
};

} // namespace bicycle_to_ackermann_traction_adapter



#endif  // BICYCLE_TO_ACKERMANN_TRACTION_ADAPTER_HPP_