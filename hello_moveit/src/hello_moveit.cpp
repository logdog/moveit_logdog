#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ur_msgs/srv/set_io.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <future>

// Structure to hold I/O commands with timing
struct IOCommand {
  double time_from_start;  // seconds
  int pin;                // I/O pin number
  bool state;             // true = HIGH, false = LOW
};

// Function to set digital I/O using UR service
void set_digital_io(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client, int pin, bool state) {
  auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
  request->fun = 1;  // Digital output function
  request->pin = pin;
  request->state = state ? 1.0 : 0.0;
  
  auto future = client->async_send_request(request);
  
  if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(rclcpp::get_logger("hello_moveit"), 
                  "I/O Pin %d set to %s", pin, state ? "HIGH" : "LOW");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("hello_moveit"), "Failed to set I/O");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("hello_moveit"), "I/O service call timeout");
  }
}

// Function to execute I/O commands based on timer
void execute_io_sequence(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr io_client,
                        const std::vector<IOCommand>& io_commands) {
  auto start_time = std::chrono::steady_clock::now();
  
  for (const auto& cmd : io_commands) {
    // Calculate when to execute this command
    auto target_time = start_time + std::chrono::duration<double>(cmd.time_from_start);
    
    // Sleep until it's time to execute
    std::this_thread::sleep_until(target_time);
    
    // Execute the I/O command
    set_digital_io(io_client, cmd.pin, cmd.state);
  }
}

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create I/O service client
  auto io_client = node->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");
  RCLCPP_INFO(logger, "Waiting for I/O service...");
  if (!io_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN(logger, "I/O service not available after 5 seconds");
  } else {
    RCLCPP_INFO(logger, "I/O service is ready");
  }

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  const std::string manipulator_group = "ur_manipulator";
  // auto move_group_interface = MoveGroupInterface(node, "manipulator");
  auto move_group_interface = MoveGroupInterface(node, manipulator_group);

  // Debug: Print available planning groups
  auto robot_model = move_group_interface.getRobotModel();
  auto group_names = robot_model->getJointModelGroupNames();
  RCLCPP_INFO(logger, "Available planning groups:");
  for (const auto& group_name : group_names) {
    RCLCPP_INFO(logger, "  - %s", group_name.c_str());
  }

  // Debug: Print links in the current group
  auto jmg = robot_model->getJointModelGroup(manipulator_group);
  auto link_names = jmg->getLinkModelNames();
  RCLCPP_INFO(logger, "Links in '%s' group:", manipulator_group.c_str());
  for (const auto& link_name : link_names) {
    RCLCPP_INFO(logger, "  - %s", link_name.c_str());
  }

  // Check the planning frame - this is what MoveIt uses as reference
  std::string planning_frame = move_group_interface.getPlanningFrame();
  RCLCPP_INFO(logger, "Planning frame: %s", planning_frame.c_str());
  
  // Check the end effector link
  std::string ee_link_name = move_group_interface.getEndEffectorLink();
  RCLCPP_INFO(logger, "End effector link: %s", ee_link_name.c_str());

  // Get current pose of the end effector - this will be relative to the planning frame
  auto current_pose = move_group_interface.getCurrentPose();  // Uses default end effector link
  
  RCLCPP_INFO(logger, "Current end effector pose:");
  RCLCPP_INFO(logger, "Frame: %s (this should match planning frame)", current_pose.header.frame_id.c_str());
  RCLCPP_INFO(logger, "Position - x: %f, y: %f, z: %f", 
              current_pose.pose.position.x, 
              current_pose.pose.position.y, 
              current_pose.pose.position.z);
  RCLCPP_INFO(logger, "Orientation - x: %f, y: %f, z: %f, w: %f",
              current_pose.pose.orientation.x,
              current_pose.pose.orientation.y,
              current_pose.pose.orientation.z,
              current_pose.pose.orientation.w);

  // Construct and initialize MoveItVisualTools - use the same frame as planning
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, planning_frame, rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                              move_group_interface.getRobotModel() };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closure for updating the text in rviz
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;  // Place text 1m above the planning frame
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) { moveit_visual_tools.prompt(text); };
  
  const moveit::core::LinkModel *ee_link = 
            move_group_interface.getRobotModel()->getLinkModel("tool0");
  
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools, ee_link, jmg](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, ee_link, jmg, rviz_visual_tools::GREEN);
      };

  // Generate a series of waypoints for the robot to follow
  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  // Start with current pose
  auto start_pose = current_pose.pose;
  waypoints.push_back(start_pose);
  
  // Waypoint 1: Move 5cm in +X direction
  auto waypoint1 = start_pose;
  waypoint1.position.x += 0.10;
  waypoints.push_back(waypoint1);
  
  // Waypoint 2: Move 3cm in +Y direction from waypoint1
  auto waypoint2 = waypoint1;
  waypoint2.position.y += 0.10;
  waypoints.push_back(waypoint2);
  
  // Waypoint 3: Move 2cm in -Z direction from waypoint2
  auto waypoint3 = waypoint2;
  waypoint3.position.z -= 0.10;
  waypoints.push_back(waypoint3);
  
  // Waypoint 4: Return close to start but offset
  auto waypoint4 = start_pose;
  waypoints.push_back(waypoint4);

  RCLCPP_INFO(logger, "Generated %zu waypoints", waypoints.size());
  
  // Set velocity and acceleration scaling for faster execution BEFORE computing the path
  move_group_interface.setMaxVelocityScalingFactor(1.0);  // 100% of max velocity (default is often 0.1)
  move_group_interface.setMaxAccelerationScalingFactor(1.0);  // 100% of max acceleration (default is often 0.1)
  
  RCLCPP_INFO(logger, "Set velocity scaling to 100%% and acceleration scaling to 100%%");
  
  // Compute Cartesian path through waypoints
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.0001; // 1mm steps
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, trajectory);
  
  RCLCPP_INFO(logger, "Cartesian path %.2f%% achieved", fraction * 100.0);
  
  if (fraction < 0.9) {
    RCLCPP_WARN(logger, "Cartesian path planning failed or incomplete");
    return -1;
  }
  
  // Calculate trajectory duration
  double trajectory_duration = 0.0;
  if (!trajectory.joint_trajectory.points.empty()) {
    auto& last_point = trajectory.joint_trajectory.points.back();
    trajectory_duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;
  }
  
  RCLCPP_INFO(logger, "Trajectory duration: %.2f seconds", trajectory_duration);
  
  // Define I/O sequence with precise timing
  std::vector<IOCommand> io_sequence = {
    {0.0, 16, false},
    {trajectory_duration * 0.25, 16, true},
    {trajectory_duration * 0.5, 16, false},
    {trajectory_duration * 0.75, 16, true},
    {trajectory_duration * 0.9, 16, false},
  };
  
  // Log the I/O sequence
  RCLCPP_INFO(logger, "I/O Sequence:");
  for (const auto& cmd : io_sequence) {
    RCLCPP_INFO(logger, "  t=%.2fs: Pin %d -> %s", 
                cmd.time_from_start, cmd.pin, cmd.state ? "HIGH" : "LOW");
  }

  // Create a plan using the Cartesian trajectory
  prompt("Press 'next' in the RvizVisualToolsGui window to plan Cartesian path");
  draw_title("Planning Cartesian Path");
  moveit_visual_tools.trigger();
  
  // Create a plan from the computed trajectory
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory = trajectory;
  
  bool success = (fraction > 0.9); // Consider successful if >90% of path achieved

  // Execute the plan with synchronized I/O
  if (success)
  {
    draw_trajectory_tool_path(plan.trajectory);
    moveit_visual_tools.trigger();
    prompt("Press 'next' in the RvizVisualToolsGui window to execute with I/O");
    draw_title("Executing with I/O Control");
    moveit_visual_tools.trigger();
    
    // Start I/O sequence in a separate thread
    std::thread io_thread([io_client, io_sequence]() {
      execute_io_sequence(io_client, io_sequence);
    });
    
    // Execute the trajectory
    RCLCPP_INFO(logger, "Starting trajectory execution...");
    auto execute_result = move_group_interface.execute(plan);
    
    // Wait for I/O thread to complete
    io_thread.join();
    
    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "Trajectory execution completed successfully");
      draw_title("Execution Complete");
    } else {
      RCLCPP_ERROR(logger, "Trajectory execution failed");
      draw_title("Execution Failed");
    }
    moveit_visual_tools.trigger();
  }
  else
  {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}