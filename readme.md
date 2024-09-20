# Hyundai PND ELMO Master

This package controls the Hyundai PND modules via Elmo motor drivers using ROS2 and CANopen communication.

## 1. Setup Instructions

### 1.1 Install Peak USB Driver

To set up the Peak USB driver, run the following commands:
```bash
./init_peakusb.sh
ros2 launch elmo_master elmo_master.launch.py
```

### 1.2 Basic Usage

1. **Starting the system**  
   After launching `elmo_master`, call the `elmo/start` service to begin operation:
   ```bash
   ros2 service call /canX/elmo/start std_srvs/srv/Trigger
   ```
   
2. **When the motor does not move as expected**  
   If the motor does not move, try the following sequence:
   ```bash
   ros2 service call /canX/elmo/stop std_srvs/srv/Trigger
   ros2 service call /canX/elmo/recover std_srvs/srv/Trigger
   ```

3. **Quick Stop**  
   To stop the PND module quickly, use the `elmo/stop` service:
   ```bash
   ros2 service call /canX/elmo/stop std_srvs/srv/Trigger
   ```

4. **Exiting Safely**  
   To safely terminate the nodes, use the `elmo/exit` service:
   ```bash
   ros2 service call /canX/elmo/exit std_srvs/srv/Trigger
   ```
   *Note: Using `elmo/exit` may take some time due to thread management.*

5. **Recovering from errors**  
   If unexpected termination occurs, unplug and replug the Peak USB and follow the setup process again.

6. **Node Reset**  
   To fully reset the nodes after stopping, use:
   ```bash
   ros2 service call /canX/elmo/reset std_srvs/srv/Trigger
   ```

**Important Note:**  
Avoid calling services too frequently without waiting for a response, as this may lead to unexpected behavior.

## 2. CAN Devices Info

- **can0** : Left Front and Left Rear PND modules  
   - FL Steering: Node-ID 1 (Device ID 3)  
   - FL Throttle: Node-ID 2 (Device ID 4)  
   - RL Steering: Node-ID 3 (Device ID 1)  
   - RL Throttle: Node-ID 4 (Device ID 2)  

- **can1** : Right Front and Right Rear PND modules  
   - FR Steering: Node-ID 1 (Device ID 3)  
   - FR Throttle: Node-ID 2 (Device ID 4)  
   - RR Steering: Node-ID 3 (Device ID 1)  
   - RR Throttle: Node-ID 4 (Device ID 2)  

## 3. Params

- **elmo_can1.yaml**: CAN bus configuration
- **elmo_master.launch.py**: Sets ROS2 parameters such as:
  ```yaml
  {'type': 'combined'},
  {'can_interface': 'can0'},
  {'can_config_file': 'config/elmo_can1.yaml'},
  {'resolution_position': 33554432}, # 2^25
  {'resolution_velocity': 65536}, # 2^25
  {'max_position': 3.14159}, # pi
  {'min_position': -3.14159}, # -pi
  {'max_velocity': 0.1}, # 0.1m/s (elmo), 5m/s (test motor)
  {'wheel_radius': 0.08}, # 0.08m (elmo), 0.1m (test motor)
  ```

## 4. Topics

After calling `elmo/start`, publish the following topics to control the system:

- Target velocity:
  ```bash
  ros2 topic pub /canX/elmo/target_velocity std_msgs/msg/Float32MultiArray
  ```
  
- Target position:
  ```bash
  ros2 topic pub /canX/elmo/target_position std_msgs/msg/Float32MultiArray
  ```

You can also check the actual values with:

- Actual velocity:
  ```bash
  ros2 topic info /canX/elmo/actual_velocity std_msgs/msg/Float32MultiArray
  ```
  
- Actual position:
  ```bash
  ros2 topic info /canX/elmo/actual_position std_msgs/msg/Float32MultiArray
  ```

## 5. Services

Use the following commands to interact with CAN bus messages:

- **Print out CAN bus messages**:
  ```bash
  candump can0
  candump can1
  ```

- **Reset each CAN bus**:
  ```bash
  cansend can0 000#8100
  cansend can1 000#8100
  ```
---
