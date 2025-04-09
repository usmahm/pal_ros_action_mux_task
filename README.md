# PAL Action Mux - Task: Delay Action & Generic Subscriber

This repository demonstrates two ROS 2 tasks as part of PAL Robotics GSoC qualification task:

1. **Task A: Delay Action with Subscription-Driven Requests**

   Implements an action-like behavior using a service, with a 5-second delay and dynamic cancellation of previous goals when new requests arrive.

2. **Task B: Generic Subscriber for Any Message Type**

   Dynamically subscribes to any ROS 2 topic, identifies the message type, and logs the received messages.

## Table of Contents
- [Overview](#overview)
- [Installation & Building](#installation--building)
- [Task A: Delay Action with Subscription-Driven Requests](#task-a-delay-action-with-subscription-driven-requests)
  - [How It Works](#how-it-works-for-task-a)
  - [Running the Python Version](#running-the-python-version-for-task-a)
  - [Running the C++ Version](#running-the-c-version-for-task-a)
- [Task B: Generic Subscriber for Any Message Type](#task-b-generic-subscriber-for-any-message-type)
  - [How It Works](#how-it-works-for-task-b)
  - [Running the Python Version](#running-the-python-version-for-task-b)
  - [Running the C++ Version](#running-the-c-version-for-task-b)

---

## Overview

### Task A: Delay Action
- Simulates an action server using a service with a 5-second delay.
- Cancels the current goal if a new request arrives.

### Task B: Generic Subscriber
- Dynamically discovers message types on a topic and subscribes to them.
- Logs the message type and content in real-time.

Both tasks are implemented in **C++** (using `rclcpp`) and **Python** (using `rclpy`).

---

## Installation & Building

1. Clone the repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/<username>/pal_ros_action_mux_task.git
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

---

## Task A: Delay Action with Subscription-Driven Requests

### How It Works (for Task A)
- **Action Server**:
  - Implements a service named `action` (defined in `action_interfaces/srv/String.srv`).
  - Waits for requests and starts a 5-second timer to simulate action execution.
  - Cancels the current goal if a new request arrives.

- **Action Client**:
  - Subscribes to the topic `new_action`.
  - Sends requests to the `action` service whenever a new message is received.
  - Logs the server's response after the 5-second delay.

### Running the Python Version (for Task A)
1. In one terminal, start the action server:
   ```bash
   ros2 run delay_action_py action_server_py
   ```
2. In another terminal, start the action client:
   ```bash
   ros2 run delay_action_py action_client_py
   ```
3. In a third terminal, publish messages to the `new_action` topic:
   ```bash
   ros2 topic pub /new_action std_msgs/msg/String "{data: 'actuate'}" -r 2
   ```

### Running the C++ Version (for Task A)
1. In one terminal, start the action server:
   ```bash
   ros2 run delay_action_cpp action_server
   ```
2. In another terminal, start the action client:
   ```bash
   ros2 run delay_action_cpp action_client
   ```
3. In a third terminal, publish messages to the `new_action` topic:
   ```bash
   ros2 topic pub /new_action std_msgs/msg/String "{data: 'actuate'}" -r 2
   ```

---

## Task B: Generic Subscriber for Any Message Type

### How It Works (for Task B)
- **Generic Subscriber**:
  - Periodically checks `get_topic_names_and_types()` to discover message types on the `/buffer` topic.
  - Subscribes to the discovered type and logs the message type and content.

- **Test Publishers**:
  - `generic_tester_int_pub.cpp`: Publishes `std_msgs::msg::Int64` to `/buffer`.
  - `generic_tester_string_pub.cpp`: Publishes `std_msgs::msg::String` to `/buffer`.

### Running the Python Version (for Task B)
1. In one terminal, start the generic subscriber:
   ```bash
   ros2 run generic_py generic_subscriber_py
   ```
2. In another terminal, start the integer publisher:
   ```bash
   ros2 run generic_py generic_int_pub_py
   ```
   If you want to test with the string publisher, stop the subscriber and restart it:
   ```bash
   ros2 run generic_py generic_subscriber_py
   ```
   Then, in another terminal, start the string publisher:
   ```bash
   ros2 run generic_py generic_string_pub_py
   ```

### Running the C++ Version (for Task B)
1. In one terminal, start the generic subscriber:
   ```bash
   ros2 run generic_cpp generic_subscriber
   ```
2. In another terminal, start the integer publisher:
   ```bash
   ros2 run generic_cpp generic_tester_int_pub
   ```
   If you want to test with the string publisher, stop the subscriber and restart it:
   ```bash
   ros2 run generic_cpp generic_subscriber
   ```
   Then, in another terminal, start the string publisher:
   ```bash
   ros2 run generic_cpp generic_tester_string_pub
   ```
