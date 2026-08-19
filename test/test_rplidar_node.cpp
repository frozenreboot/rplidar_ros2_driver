#include <atomic>
#include <future>
#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>

#include "rplidar_node.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class RplidarNodeTest : public ::testing::Test {
protected:
  // Initialize ROS 2 runtime once for the entire test suite
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  // Shutdown ROS 2 runtime after tests complete
  static void TearDownTestCase() { rclcpp::shutdown(); }
};

/**
 * @brief [Lifecycle Verification]
 * Tests the complete lifecycle state transitions (Configure -> Activate ->
 * Deactivate -> Cleanup). Uses 'dummy_mode' to isolate logic from physical
 * hardware dependencies.
 */
TEST_F(RplidarNodeTest, FullLifecycleTest) {
  // ==========================================================================
  // [Arrange] Setup the node with dummy mode enabled
  // ==========================================================================
  rclcpp::NodeOptions options;
  // Enable dummy driver to simulate hardware behavior
  options.append_parameter_override("dummy_mode", true);
  // Set a dummy serial port (not used in dummy mode, but required for init)
  options.append_parameter_override("serial_port", "/dev/null");

  auto node = std::make_shared<rplidar_driver::RPlidarNode>(options);

  // Verify initial state
  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);

  // ==========================================================================
  // [Act & Assert] 1. Unconfigured -> Inactive (Configure)
  // ==========================================================================
  // This triggers init_parameters() and creates the driver instance.
  auto result_conf = node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE));
  EXPECT_EQ(result_conf.id(), State::PRIMARY_STATE_INACTIVE)
      << "Failed to transition from UNCONFIGURED to INACTIVE.";

  // ==========================================================================
  // [Act & Assert] 2. Inactive -> Active (Activate)
  // ==========================================================================
  // This starts the scan thread and enables publishers.
  auto result_act = node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE));
  EXPECT_EQ(result_act.id(), State::PRIMARY_STATE_ACTIVE)
      << "Failed to transition from INACTIVE to ACTIVE.";

  // ==========================================================================
  // [Act & Assert] 3. Active -> Inactive (Deactivate)
  // ==========================================================================
  // This stops the scan thread and disables publishers.
  auto result_deact = node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE));
  EXPECT_EQ(result_deact.id(), State::PRIMARY_STATE_INACTIVE)
      << "Failed to transition from ACTIVE to INACTIVE.";

  // ==========================================================================
  // [Act & Assert] 4. Inactive -> Unconfigured (Cleanup)
  // ==========================================================================
  // This destroys the driver instance and releases resources.
  auto result_clean = node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CLEANUP));
  EXPECT_EQ(result_clean.id(), State::PRIMARY_STATE_UNCONFIGURED)
      << "Failed to transition from INACTIVE to UNCONFIGURED.";
}

/**
 * @brief [Integration Test] Scan Topic Publication
 * Verifies that the node actually publishes data to the '/scan' topic
 * when in the ACTIVE state.
 */
TEST_F(RplidarNodeTest, ScanPublicationCheck) {
  // ==========================================================================
  // [Arrange] Setup Node, QoS, and Promise/Future for async validation
  // ==========================================================================
  rclcpp::NodeOptions options;
  options.append_parameter_override("dummy_mode", true);
  options.append_parameter_override("frame_id", "test_laser_link");

  auto node = std::make_shared<rplidar_driver::RPlidarNode>(options);

  // Quickly transition to ACTIVE state
  node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE));
  node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE));

  // Prepare a promise to signal when a message is received
  auto promise = std::make_shared<std::promise<void>>();
  auto future = promise->get_future();
  bool message_received = false;

  // Create a subscriber to listen for the published scan
  auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // ----------------------------------------------------------------------
        // [Assert] Callback Validation (Inside the Loop)
        // ----------------------------------------------------------------------
        message_received = true;

        // Check 1: Verify frame_id matches configuration
        EXPECT_EQ(msg->header.frame_id, "test_laser_link");

        // Check 2: Ensure data payload is present
        // Since we use the dummy driver, it should generate valid points.
        EXPECT_GT(msg->ranges.size(), 0) << "Received scan message is empty.";

        // Signal completion
        promise->set_value();
      });

  // ==========================================================================
  // [Act] Spin the node to process callbacks
  // ==========================================================================
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  auto start_time = std::chrono::steady_clock::now();
  // Wait up to 3 seconds for a message
  while (rclcpp::ok() && !message_received) {
    executor.spin_some();

    if (std::chrono::steady_clock::now() - start_time >
        std::chrono::seconds(3)) {
      break; // Timeout
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // ==========================================================================
  // [Assert] Final Validation
  // ==========================================================================
  EXPECT_TRUE(message_received)
      << "Timeout: Failed to receive /scan topic within 3 seconds.";

  // Clean shutdown
  node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE));
}

/**
 * @brief [Integration Test] Standby Mode
 * Verifies that the 'stop_motor' service suspends scan publication without
 * leaving the ACTIVE lifecycle state, and that 'start_motor' resumes it.
 */
TEST_F(RplidarNodeTest, StandbyServiceTogglesScanning) {
  // ==========================================================================
  // [Arrange] Active node in dummy mode + a client node for the services
  // ==========================================================================
  rclcpp::NodeOptions options;
  options.append_parameter_override("dummy_mode", true);

  auto node = std::make_shared<rplidar_driver::RPlidarNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("standby_test_client");

  node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE));
  node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE));

  std::atomic<int> scan_count{0};
  auto sub = client_node->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [&scan_count](sensor_msgs::msg::LaserScan::SharedPtr) { scan_count++; });

  auto stop_client =
      client_node->create_client<std_srvs::srv::Empty>("stop_motor");
  auto start_client =
      client_node->create_client<std_srvs::srv::Empty>("start_motor");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(client_node);

  // Spin for a fixed wall-clock duration, draining all pending callbacks.
  auto spin_for = [&executor](std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  };

  // Call a service and pump the executor until the response arrives.
  auto call_service =
      [&executor](rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client) {
        EXPECT_TRUE(client->wait_for_service(std::chrono::seconds(3)))
            << "Service '" << client->get_service_name() << "' never appeared.";

        auto future = client->async_send_request(
            std::make_shared<std_srvs::srv::Empty::Request>());
        return executor.spin_until_future_complete(future,
                                                   std::chrono::seconds(3)) ==
               rclcpp::FutureReturnCode::SUCCESS;
      };

  // ==========================================================================
  // [Act & Assert] 1. Scanning is running before the standby request
  // ==========================================================================
  spin_for(std::chrono::milliseconds(500));
  EXPECT_GT(scan_count.load(), 0) << "No scans published before standby.";

  // ==========================================================================
  // [Act & Assert] 2. stop_motor -> publication stops, node stays ACTIVE
  // ==========================================================================
  EXPECT_TRUE(call_service(stop_client)) << "'stop_motor' call timed out.";

  // Let the scan thread observe the request and drain any in-flight message.
  spin_for(std::chrono::milliseconds(300));

  scan_count = 0;
  spin_for(std::chrono::milliseconds(500));
  EXPECT_EQ(scan_count.load(), 0) << "Scans still published while in standby.";

  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_ACTIVE)
      << "Standby must not leave the ACTIVE lifecycle state.";

  // ==========================================================================
  // [Act & Assert] 3. start_motor -> publication resumes
  // ==========================================================================
  EXPECT_TRUE(call_service(start_client)) << "'start_motor' call timed out.";

  scan_count = 0;
  spin_for(std::chrono::seconds(2));
  EXPECT_GT(scan_count.load(), 0) << "Scans did not resume after start_motor.";

  // Clean shutdown
  node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE));
}
