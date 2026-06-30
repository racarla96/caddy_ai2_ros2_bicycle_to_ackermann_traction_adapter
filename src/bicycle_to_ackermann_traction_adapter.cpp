#include "bicycle_to_ackermann_traction_adapter/bicycle_to_ackermann_traction_adapter.hpp"

namespace bicycle_to_ackermann_traction_adapter
{

BicycleToAckermannTractionAdapter::BicycleToAckermannTractionAdapter() : controller_interface::ChainableControllerInterface()
{
}

controller_interface::CallbackReturn BicycleToAckermannTractionAdapter::on_init()
{
  try
  {
    bicycle_to_ackermann_traction_adapter_param_listener_ =
        std::make_shared<bicycle_to_ackermann_traction_adapter::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BicycleToAckermannTractionAdapter::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  try
  {
    bicycle_to_ackermann_traction_adapter_params_ = bicycle_to_ackermann_traction_adapter_param_listener_->get_params();

    wheelbase_ = bicycle_to_ackermann_traction_adapter_params_.wheelbase;
    track_width_ = bicycle_to_ackermann_traction_adapter_params_.track_width;
    wheel_radius_ = bicycle_to_ackermann_traction_adapter_params_.wheel_radius;
    by_reference_or_by_state_ = bicycle_to_ackermann_traction_adapter_params_.by_reference_or_by_state;
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during configure stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration BicycleToAckermannTractionAdapter::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.reserve(nr_output_steer_itfs_);
  for (size_t i = 0; i < nr_output_steer_itfs_; i++)
  {
    command_interfaces_config.names.push_back(
      bicycle_to_ackermann_traction_adapter_params_.output_traction_names[i]
      + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration BicycleToAckermannTractionAdapter::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (by_reference_or_by_state_)
  {
    state_interfaces_config.names.push_back(
      bicycle_to_ackermann_traction_adapter_params_.input_traction_name
      + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(
      bicycle_to_ackermann_traction_adapter_params_.input_steering_name
      + "/" + hardware_interface::HW_IF_POSITION);
  }
  else
  {
    state_interfaces_config.names.reserve(nr_output_steer_itfs_ + 1);
    for (size_t i = 0; i < nr_output_steer_itfs_; i++)
    {
      state_interfaces_config.names.push_back(
        bicycle_to_ackermann_traction_adapter_params_.output_traction_names[i]
        + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    state_interfaces_config.names.push_back(
      bicycle_to_ackermann_traction_adapter_params_.input_steering_name
      + "/" + hardware_interface::HW_IF_POSITION);
  }

  return state_interfaces_config;
}

std::vector<hardware_interface::StateInterface>
BicycleToAckermannTractionAdapter::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces_values_.reserve(nr_input_steer_itfs_);

    state_interfaces.emplace_back(
        get_name()
        + "/" + bicycle_to_ackermann_traction_adapter_params_.input_traction_name,
        hardware_interface::HW_IF_VELOCITY,
        &state_interfaces_values_[0]
    );

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
BicycleToAckermannTractionAdapter::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.reserve(nr_input_steer_itfs_);
  reference_interfaces_.resize(nr_input_steer_itfs_);

    command_interfaces.emplace_back(
        get_name()
        + "/" + bicycle_to_ackermann_traction_adapter_params_.input_traction_name,
        hardware_interface::HW_IF_VELOCITY,
        &reference_interfaces_[0]
    );

  return command_interfaces;
}

controller_interface::CallbackReturn BicycleToAckermannTractionAdapter::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BicycleToAckermannTractionAdapter::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type BicycleToAckermannTractionAdapter::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  return controller_interface::return_type::OK;
}

controller_interface::return_type BicycleToAckermannTractionAdapter::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;

  auto logger = get_node()->get_logger();

  right_traction_vel_ = 0.0;
  left_traction_vel_ = 0.0;

  if (by_reference_or_by_state_)
  {
    // by_state: input = hardware sensors (center traction velocity + center steering angle) → apply kinematics → output individual wheel velocities
    auto center_traction_vel_op = state_interfaces_[0].get_optional();
    auto center_steering_angle_op = state_interfaces_[1].get_optional();

    if (!center_traction_vel_op.has_value())
    {
      RCLCPP_ERROR(logger, "Unable to retrieve velocity data for center traction sensor");
      return controller_interface::return_type::ERROR;
    }
    if (!center_steering_angle_op.has_value())
    {
      RCLCPP_ERROR(logger, "Unable to retrieve position data for center steering angle sensor");
      return controller_interface::return_type::ERROR;
    }

    center_traction_vel_ = center_traction_vel_op.value();
    center_steering_angle_ = center_steering_angle_op.value();

    state_interfaces_values_[0] = center_traction_vel_ * wheel_radius_;

    input_linear_vel_ = center_traction_vel_ * wheel_radius_;
    transmission_factor_ = (track_width_ * tan(center_steering_angle_)) / (2.0 * wheelbase_);
    left_traction_vel_ = (input_linear_vel_ * (1.0 - transmission_factor_)) / wheel_radius_;
    right_traction_vel_ = (input_linear_vel_ * (1.0 + transmission_factor_)) / wheel_radius_;
  }
  else
  {
    // by_reference: read individual wheel states → compute center velocity feedback → apply kinematics from reference
    auto right_traction_vel_op = state_interfaces_[0].get_optional();
    auto left_traction_vel_op = state_interfaces_[1].get_optional();
    auto center_steering_angle_op = state_interfaces_[2].get_optional();

    if (!right_traction_vel_op.has_value())
    {
      RCLCPP_ERROR(logger, "Unable to retrieve velocity feedback data for right traction");
      return controller_interface::return_type::ERROR;
    }
    if (!left_traction_vel_op.has_value())
    {
      RCLCPP_ERROR(logger, "Unable to retrieve velocity feedback data for left traction");
      return controller_interface::return_type::ERROR;
    }
    if (!center_steering_angle_op.has_value())
    {
      RCLCPP_ERROR(logger, "Unable to retrieve position feedback data for center steering angle");
      return controller_interface::return_type::ERROR;
    }

    right_traction_vel_ = right_traction_vel_op.value();
    left_traction_vel_ = left_traction_vel_op.value();
    state_interfaces_values_[0] = ((right_traction_vel_ + left_traction_vel_) / 2) * wheel_radius_;

    center_steering_angle_ = center_steering_angle_op.value();

    right_traction_vel_ = 0.0;
    left_traction_vel_ = 0.0;

    if (!std::isnan(reference_interfaces_[0])) {
      center_traction_vel_ = reference_interfaces_[0];
      input_linear_vel_ = center_traction_vel_ * wheel_radius_;
      transmission_factor_ = (track_width_ * tan(center_steering_angle_)) / (2.0 * wheelbase_);
      left_traction_vel_ = (input_linear_vel_ * (1.0 - transmission_factor_)) / wheel_radius_;
      right_traction_vel_ = (input_linear_vel_ * (1.0 + transmission_factor_)) / wheel_radius_;
    }

    reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
  }

  if (!command_interfaces_[0].set_value(right_traction_vel_)) {
    RCLCPP_WARN(logger, "Failed to set right traction");
  }
  if (!command_interfaces_[1].set_value(left_traction_vel_)) {
    RCLCPP_WARN(logger, "Failed to set left traction");
  }

  return controller_interface::return_type::OK;
}


}; // namespace bicycle_to_ackermann_traction_adapter

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  bicycle_to_ackermann_traction_adapter::BicycleToAckermannTractionAdapter,
  controller_interface::ChainableControllerInterface)
