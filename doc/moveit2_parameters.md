# moviet2 run_moveit_cpp example

## panda robot_description
```python
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("moveit_resources_panda_moveit_config"),
            "config",
            "panda.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "moveit_resources_panda_moveit_config", "config/panda.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    moveit_simple_controllers_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/panda_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)
```

## moveit_cpp.yaml

```python
    moveit_cpp_yaml_file_name = (
        get_package_share_directory("run_moveit_cpp") + "/config/moveit_cpp.yaml"
    )
```

```yaml
run_moveit_cpp:
  ros__parameters:
    # namespace (structure?)
    planning_scene_monitor_options:

      # string, what is this used for, it sets a string inside psm that can be accessed with psm->getNmae() but otherwise seems unused
      name: "planning_scene_monitor"
      # string, parameter that contains the xml robot descroption
      robot_description: "robot_description"
      # string, topic containing joint states (must start with /???)
      joint_state_topic: "/joint_states"
      # used by psm, topics to use for interacting with psm
      attached_collision_object_topic: "/moveit_cpp/planning_scene_monitor"
      publish_planning_scene_topic: "/moveit_cpp/publish_planning_scene"
      monitored_planning_scene_topic: "/moveit_cpp/monitored_planning_scene"
      // double, seconds, unused for now, must be >= 0.0
      wait_for_initial_state_timeout: 10.0

    planning_pipelines:
      #namespace: "moveit_cpp"  # optional, default is ~
      pipeline_names: ["ompl"]

    plan_request_params:
      planning_attempts: 10
      planning_pipeline: ompl
      max_velocity_scaling_factor: 1.0
      max_acceleration_scaling_factor: 1.0
```

```python
    run_moveit_cpp_node = Node(
        name="run_moveit_cpp",
        package="run_moveit_cpp",
        # TODO(henningkayser): add debug argument
        # prefix='xterm -e gdb --args',
        executable="run_moveit_cpp",
        output="screen",
        parameters=[
            moveit_cpp_yaml_file_name,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            fake_controller,
        ],
    )
```
