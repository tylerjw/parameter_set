## Sets of Parameters

Nodes contain sets of parameters.

Callbacks are registered for changes within the sets of parameters.  This allows for updates to parameters within a set to notify the correct subsystem (class) within the Node.  An exmple of this would be the parameters for the planning scene monitor for moveit.

An example config:

moveit:
  ros__parameters:
    robot:
      robot_description: "robot_description"
      joint_state_topic: "/joint_states"
    planning:
      pipeline_names: ["ompl"]
      planning_attempts: 10
      max_velocity_scaling_factor: 1.0
      max_acceleration_scaling_factor: 1.0
    planning_scene_monitor:
      name: "planning_scene_monitor"
      attached_collision_object_topic: "planning_scene_monitor"
      publish_planning_scene_topic: "publish_planning_scene"
      monitored_planning_scene_topic: "monitored_planning_scene"

This means that each of the subsystems within moveit that require these configs can register for a callback when their Set of parameters has a change.  They can then set a flag to get a new set of parameters when they are ready to update their own internal state.  It is important that the method to get new parameters is not called within the callback notifying the subsystem of new parameters.

## Constraints

When parameters are declared they can have constraints that are checked when they are set (both when set from yaml durring initialization and at any point through the parameter service interface).

The ParameterDescription ros message is used by ROS to advertise to other nodes and users information about the parameter: https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterDescriptor.msg

### additional_constraints

Constraints that are not represented by values in ParameterDescriptor.msg can be represented by validation callbacks and a string in the additional_constraints string to notify external users about the additional constraints.  An exmple of this would the a normalized quaternion constraint that only lets the user set it with a double array of size 4 that is normalized.
To specify these the NodeParameters class should have an interface for adding these additional_constraint callbacks.

## C++ ParameterSet Pattern
```c++
class RobotParameters : public ParameterSet {
 public:
  using ParameterSet::ParameterSet;

  // parameters with default values
  string robot_description = "robot_description";
  string joint_state_topic = "/joint_states";

  bool declare(node_parameters::NodeParameters* node_parameters,
               std::shared_ptr<rclcpp::Node> node) override;
  bool get(std::shared_ptr<rclcpp::Node> node) override;
};

bool RobotParameters::declare(node_parameters::NodeParameters* node_parameters,
                              std::shared_ptr<rclcpp::Node> node) {
  auto ns = getNamespace();
  node->declare_parameter(
      ns + ".robot_description", robot_description,
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_STRING)
          .description("Parameter containing XML robotic description"));

  node->declare_parameter(
      ns + ".joint_state_topic", joint_state_topic,
      ParameterDescriptorBuilder()
          .type(ParameterType::PARAMETER_STRING)
          .description("Topic to subscribe to for joint states")
          .additional_constraints("Must be a valid topic name"));
  node_parameters->registerValidateFunction(
      ns + ".joint_state_topic", &node_parameters::validate::topic_name);

  return true;
}

bool RobotParameters::get(std::shared_ptr<rclcpp::Node> node) {
  auto ns = getNamespace();
  bool success = true;
  success &= node->get_parameter(ns + ".robot_description", robot_description);
  success &= node->get_parameter(ns + ".joint_state_topic", joint_state_topic);

  return success;
}
```
