---
title: ROS 2 Services, Actions, and Parameters
slug: ros2-services-actions-params
id: ros2-services-actions-params
---
# ROS 2 Services, Actions, and Parameters

Welcome back to Module 1 of "Physical AI & Humanoid Robotics"! In our journey through ROS 2, we've already explored the foundational elements of robot software: **nodes**, **topics**, and **messages**. We learned that nodes are independent processes, and topics facilitate asynchronous, one-way communication using messages.

Now, we'll introduce three more critical concepts that expand how ROS 2 nodes interact:

*   **Services**: For synchronous, request-response interactions.
*   **Actions**: For long-running tasks that require continuous feedback and can be preempted.
*   **Parameters**: For dynamically configuring the behavior of your robot's nodes.

Understanding these mechanisms is vital for building intelligent, responsive, and adaptable humanoid robots. They allow for more sophisticated control flows and system management compared to just publish/subscribe communication.

## 1. Recap: Where We Are in Module 1

In the previous chapter, we established that a ROS 2 system is a collection of interconnected **nodes**, each responsible for a specific task. These nodes primarily communicate using **topics**, which act as data streams for **messages**. This publish/subscribe model is excellent for broadcasting data (like sensor readings or robot positions) where multiple nodes might be interested, and the sender doesn't need to wait for a direct reply.

However, not all communication fits this asynchronous, fire-and-forget model. What if one node needs to *ask* another node to *do something* and then *wait* for a result? Or what if a task is so complex and long-running that you need progress updates and the ability to cancel it?

This is where **synchronous** (services) and more sophisticated **asynchronous** (actions) communication patterns come into play. Topics are like a broadcast radio, while services are more like a direct phone call, and actions are like initiating a complex project with a project manager giving you regular updates.

## 2. What are ROS 2 Services?

**ROS 2 Services** provide a way for nodes to implement **synchronous request/response communication**. This means that when a client node calls a service, it sends a request and then *blocks* (waits) until it receives a response from the service server node.

**Simple Definition:** Services are used for calling a function on another node and getting a result back, similar to how a traditional function call works in programming, but across different processes.

**Request/Response Analogy:**

*   **Vending Machine:** You put in a request (money + selection), and you wait for a response (your snack or an error message). You can't do anything else with the vending machine until it has processed your request.
*   **Asking a Robot to Open a Door:** You (client) *request* the robot (server) to "open the door." The robot processes this request. Once the door is open (or if it encounters an error), the robot sends a *response* back to you: "Door opened successfully!" or "Error: Door jammed."

**Humanoid Robot Example:**
Consider a humanoid robot. Services would be ideal for tasks like:
*   **`calibrate_hand_sensors`**: A client node asks the hand sensor node to perform a quick calibration. The client waits until the calibration is complete and receives a success/failure response.
*   **`get_battery_level`**: A client node queries the power management node for the current battery percentage. The power management node immediately responds with the level.
*   **`reset_pose`**: A client node asks the locomotion node to reset the robot to a default standing pose. The client waits for confirmation that the robot is in the default pose.

Services are best for short-duration, blocking operations where the client needs an immediate result and doesn't require progress updates.

## 3. What are ROS 2 Actions?

While services are great for quick, blocking operations, they aren't suitable for tasks that take a long time or require continuous monitoring. For these scenarios, ROS 2 provides **Actions**.

**Actions are designed for long-running, preemptable tasks that provide regular feedback.** They extend the request/response model by adding the ability to send continuous feedback to the client while the goal is being processed, and also allow the client to cancel the goal if needed.

**Structure: Goal → Feedback → Result**
An action interaction involves three primary messages:

1.  **Goal:** The client sends a `Goal` message to the action server, specifying the task to be performed (e.g., "walk 2 meters").
2.  **Feedback:** While the action server is working on the goal, it periodically sends `Feedback` messages back to the client, providing updates on its progress (e.g., "walked 0.5 meters," "avoiding obstacle").
3.  **Result:** Once the action server completes the goal (successfully or unsuccessfully), it sends a final `Result` message back to the client (e.g., "goal achieved," "failed to reach destination").

Clients can also send a `Cancel` request to stop an ongoing action.

**Humanoid Walking 2 Meters Example:**
Imagine you want your humanoid robot to walk across a room.

*   **Client (e.g., Navigation Node):** Sends a `Goal` to the `locomotion_action_server`: "Walk forward 2 meters."
*   **Action Server (e.g., Locomotion Node):**
    *   Starts walking.
    *   Periodically sends `Feedback` to the client: "Robot has walked 0.5m," "Robot has walked 1.0m, detected a small obstacle, adjusting path slightly."
    *   If it successfully walks 2 meters, it sends a `Result`: "Successfully walked 2 meters."
    *   If it encounters a major obstacle it can't navigate, or runs out of battery, it sends a `Result`: "Failed to walk 2 meters: path blocked."
*   **Client:** Receives feedback, potentially updates a UI, and finally gets the result. If the situation changes, the client could send a `Cancel` request.

**Why Actions Are Needed Instead of Services:**
If the "walk 2 meters" task were implemented as a service, the client would send the request and then *freeze* for the entire duration of the walk (which could be several seconds or minutes). The client wouldn't know if the robot was making progress, was stuck, or had even started. Actions solve this by providing real-time updates and control over long-running operations.

## 4. What are Parameters?

**Parameters** in ROS 2 are dynamic configuration values that nodes can expose and manage. They allow you to change a node's behavior *at runtime* without having to stop, modify, recompile, and restart the node.

**Robot Configuration Values:** Think of parameters as the settings or preferences for a specific piece of robot software. Just like you can adjust settings on your phone or computer, you can adjust parameters of ROS 2 nodes.

**Examples in Humanoid Robotics:**
*   **`walking_speed`**: A locomotion node might have a `walking_speed` parameter. You could change this from `0.5 m/s` to `0.8 m/s` while the robot is running, and its walking pace would immediately adapt.
*   **`head_tilt_angle`**: A head control node might have a `head_tilt_angle` parameter to adjust where the robot's head is looking (e.g., `0.0` for straight ahead, `0.2` radians down).
*   **`camera_exposure`**: A camera driver node could expose `camera_exposure` to adjust brightness in different lighting conditions.
*   **`obstacle_detection_threshold`**: A navigation node might have a parameter to set how close an object needs to be before it's considered an obstacle.

**Why Parameters are Important in Autonomous Humanoids:**

Parameters are incredibly useful for:
*   **Tuning and Optimization:** Fine-tuning robot behavior in different environments without code changes.
*   **Flexibility:** Adapting the robot to various tasks or user preferences.
*   **Debugging:** Modifying behavior on the fly to test different scenarios.
*   **User Customization:** Allowing end-users or other high-level AI components to configure the robot's personality or capabilities.

## 5. Short, Beginner-Friendly Examples (Python, `rclpy`)

Let's look at some simplified examples to illustrate services and parameters. Full action client/server implementations can be quite involved, so we'll use pseudo-code for the action server to focus on the concepts.

### Simple Service Server and Client Example

First, we need to define a service interface. Let's create a service that takes two integers and returns their sum. In ROS 2, service definitions are typically stored in `.srv` files within a package. For this example, imagine we have a `my_interfaces` package with `AddTwoInts.srv`:

```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

Now, let's create the Python nodes:

**Service Server (`add_two_ints_server.py`):**

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Standard ROS 2 service for adding two ints

class AddTwoIntsService(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Add two ints service ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a} b={request.b}')
        self.get_logger().info(f'Sending response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_service = AddTwoIntsService()
    rclpy.spin(add_two_ints_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Client (`add_two_ints_client.py`):**

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class AddTwoIntsClient(Node):

    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print('Usage: python3 add_two_ints_client.py <int1> <int2>')
        return

    add_two_ints_client = AddTwoIntsClient()
    response = add_two_ints_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    add_two_ints_client.get_logger().info(f'Result of add_two_ints: {response.sum}')

    add_two_ints_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run:
1.  In one terminal: `python3 add_two_ints_server.py`
2.  In another terminal: `python3 add_two_ints_client.py 5 7`

You will see the client send the request, and the server process it and send back the sum.

### Pseudo-code for an Action Server

Implementing a full action client and server involves a bit more code, but the core idea is simple. Here's how an action server for our "walk 2 meters" example might look in pseudo-code:

```python
# pseudo_walk_action_server.py
import rclpy
from rclpy.node import Node
# from example_interfaces.action import Walk # Imagine this action definition exists

class WalkActionServer(Node):

    def __init__(self):
        super().__init__('walk_action_server')
        self._action_server = rclpy.action.ActionServer(
            self, # Node
            # Walk, # Action type
            'walk_robot', # Action name
            self.execute_callback
        )
        self.get_logger().info('Walk Action Server ready.')

    def execute_callback(self, goal_handle):
        # Imagine goal_handle.request.distance has the target distance (e.g., 2.0 meters)
        self.get_logger().info('Executing goal: Walking ' + str(goal_handle.request.distance) + ' meters')

        feedback_msg = # Walk.Feedback() # Imagine this feedback message type
        distance_walked = 0.0

        # Simulate walking
        while distance_walked < goal_handle.request.distance:
            if goal_handle.is_cancel_requested:
                goal_handle.set_canceled()
                self.get_logger().info('Goal canceled!')
                return # Walk.Result() # Return appropriate canceled result

            # ... actual robot movement code here ...
            # Simulate walking progress
            distance_walked += 0.1 # Walk 0.1 meters
            feedback_msg.current_distance = distance_walked
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: walked {distance_walked:.1f} meters')

            # Simulate work (e.g., sleep, or actual robot control loop)
            # rclpy.sleep(0.5)

        goal_handle.set_succeeded()

        result = # Walk.Result() # Imagine this result message type
        result.final_distance = distance_walked
        self.get_logger().info('Goal succeeded!')
        return result

def main(args=None):
    rclpy.init(args=args)
    walk_action_server = WalkActionServer()
    rclpy.spin(walk_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This pseudo-code demonstrates the `execute_callback` where the main logic for the action resides, how `feedback` is published, and how the `result` is set. An actual implementation would involve a proper `Walk.action` definition (similar to `.srv` files) and a corresponding action client to send goals and receive feedback/results.

---
