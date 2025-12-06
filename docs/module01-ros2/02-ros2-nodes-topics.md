---
title: ROS 2 Nodes and Topics for Communication
slug: ros2-nodes-topics
id: ros2-nodes-topics
---
# ROS 2 Nodes and Topics for Communication

Welcome back to Module 1 of "Physical AI & Humanoid Robotics"! In our previous chapter, we introduced the exciting world of ROS 2 and set up our development environment. We learned that ROS 2 is a flexible framework for writing robot software. Now, it's time to dive into the fundamental building blocks of ROS 2 communication: **nodes**, **topics**, and **messages**.

Understanding these core concepts is crucial for building any complex robotic system, as they allow different parts of your robot's software to communicate and collaborate seamlessly.

## What is a ROS 2 Node?

Imagine you're building a robot, and this robot needs to do many things: see its environment, move its joints, process sensor data, and make decisions. If you tried to write all of this functionality in one giant program, it would quickly become unmanageable and difficult to debug.

This is where **ROS 2 nodes** come in.

**A ROS 2 node is essentially an executable process that performs a specific computation.** Think of it as a small, focused program or "worker" that handles a particular task within your robot's overall system.

**Mental Model + Analogy:**

*   **Analogy:** Consider a human team working on a project. Each team member (a **node**) has a specific role:
    *   One person is responsible for "gathering information" (e.g., a camera driver node).
    *   Another person "analyzes data" (e.g., an image processing node).
    *   A third person "executes actions" (e.g., a motor control node).
    *   Each person works independently but contributes to the larger goal.

*   **Mental Model:** In a ROS 2 system, your robot's brain is not one single program but a collection of these independent, specialized nodes. Each node runs as its own process, can be started or stopped independently, and communicates with other nodes to achieve the robot's tasks.

For example, a robot might have nodes like:
*   `camera_driver_node`: Reads images from a camera.
*   `lidar_sensor_node`: Collects laser scan data.
*   `motor_controller_node`: Sends commands to the robot's motors.
*   `path_planner_node`: Calculates a safe path for the robot.
*   `user_interface_node`: Displays information to a human operator.

This modularity makes your robot software robust, easier to develop, and simpler to debug.

## Topics and Messages: Publish/Subscribe Explained

Now that we have these independent nodes, how do they talk to each other? They do it using **topics** and **messages** through a communication pattern called **publish/subscribe**.

### Messages

At the heart of ROS 2 communication are **messages**. A message is simply a data structure that nodes use to exchange information. It's like a small packet of data with a predefined structure.

For example, a message might contain:
*   An integer (`int32`)
*   A string (`string`)
*   A floating-point number (`float64`)
*   A more complex structure like an image, a sensor reading with multiple values, or a robot's pose (position and orientation).

ROS 2 provides many standard message types (e.g., `std_msgs` for common data types, `sensor_msgs` for sensor data). You can also define your own custom message types when needed.

### Topics

**Topics are named channels over which nodes exchange messages.** Think of a topic as a public broadcasting channel or a chat room where specific types of information are shared.

**Analogy:**
Imagine a radio station (a **topic**) broadcasting weather updates.
*   Any weather reporter (a **publisher node**) can broadcast weather messages on this "Weather Updates" radio station.
*   Anyone interested in the weather (a **subscriber node**) can tune into the "Weather Updates" station to receive those messages.

Key characteristics of topics:
*   **Named:** Each topic has a unique name (e.g., `/robot/camera/image`, `/cmd_vel`, `/odom`).
*   **Untyped:** Actually, topics are *strongly typed* by the message type they carry (e.g., a topic `/chatter` might carry `std_msgs/String` messages). All messages published on a topic must be of the same type.
*   **Anonymous:** Publishers don't know who their subscribers are, and subscribers don't know who their publishers are. This creates **loose coupling**, which is a powerful design principle. Nodes can be added or removed from the system without affecting others, as long as they adhere to the topic's message type.

### Publish/Subscribe

This is the communication mechanism:

1.  **Publisher Node:** A node that creates and sends messages on a specific topic. It "publishes" messages.
2.  **Subscriber Node:** A node that listens for and receives messages from a specific topic. It "subscribes" to a topic. When a new message is published on that topic, the subscriber's callback function is invoked to process the message.

This asynchronous, many-to-many communication model is highly flexible and scalable, forming the backbone of ROS 2 applications.

## A Minimal Python Example with `rclpy`

Let's put these concepts into practice with a simple Python example using `rclpy`, the ROS 2 client library for Python. We'll create two nodes: one that publishes a "hello world" message and another that subscribes to it and prints the message.

### 1. The Publisher Node (`simple_publisher.py`)

This node will create a publisher on a topic named `chatter` and publish a string message every 0.5 seconds.

```python
# simple_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher') # Node name
        self.publisher_ = self.create_publisher(String, 'chatter', 10) # Message type, topic name, QoS history depth
        self.i = 0
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Simple Publisher Node has been started.')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2 World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args) # Initialize ROS 2 communication
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher) # Keep node alive until Ctrl+C
    simple_publisher.destroy_node()
    rclpy.shutdown() # Shutdown ROS 2 communication

if __name__ == '__main__':
    main()
```

### 2. The Subscriber Node (`simple_subscriber.py`)

This node will subscribe to the `chatter` topic and print any string messages it receives.

```python
# simple_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber') # Node name
        self.subscription = self.create_subscription(
            String, # Message type
            'chatter', # Topic name
            self.listener_callback, # Callback function
            10 # QoS history depth
        )
        self.subscription # prevent unused variable warning
        self.get_logger().info('Simple Subscriber Node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args) # Initialize ROS 2 communication
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber) # Keep node alive until Ctrl+C
    simple_subscriber.destroy_node()
    rclpy.shutdown() # Shutdown ROS 2 communication

if __name__ == '__main__':
    main()
```

### How to Run These Examples

To run these examples, you would typically place them in a ROS 2 package. For a quick test, you can run them directly from your terminal after setting up your ROS 2 environment:

1.  **Open two separate terminals.**

2.  **In the first terminal, run the publisher:**
    ```bash
    python3 simple_publisher.py
    ```

3.  **In the second terminal, run the subscriber:**
    ```bash
    python3 simple_subscriber.py
    ```

You will see the publisher terminal logging "Publishing: ..." messages, and the subscriber terminal logging "I heard: ..." messages, demonstrating the inter-node communication via topics and messages.

Congratulations! You've just created and run your first ROS 2 nodes that communicate using the publish/subscribe pattern. This fundamental understanding will be the basis for all more complex interactions in your humanoid robotics journey. In the next chapter, we'll explore services and actions, other important communication mechanisms in ROS 2.
