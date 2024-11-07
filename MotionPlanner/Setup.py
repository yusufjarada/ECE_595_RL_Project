import pybullet as p
import time
import pybullet_data

# Connect to PyBullet
physicsClient = p.connect(p.GUI)

# Set up environment and load URDFs
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Optionally set a search path for plane
planeId = p.loadURDF("plane.urdf")  # Load a plane for the robot to drive on

# Path to your TurtleBot URDF file
turtlebot_path = "C:/Users/yusuf/Documents/turtlebot3/turtlebot3_description/urdf/turtlebot.urdf"
turtlebot = p.loadURDF(turtlebot_path, [2.5, 2.5, 1])
block1 = p.loadURDF("Objects/block.urdf", [27, 10, 2.5])
block2 = p.loadURDF("Objects/block.urdf", [14, 12, 2.5])
block3 = p.loadURDF("Objects/block.urdf", [40, 20, 2.5])
block4 = p.loadURDF("Objects/block.urdf", [15, 25, 2.5])
goalBlock = p.loadURDF("Objects/goalBlock.urdf", [48,28,1])
wall_1 = p.loadURDF("Objects/longWall.urdf", [25, 30.5, 1.25], p.getQuaternionFromEuler([0, 0, 0]))  # Back wall
wall_2 = p.loadURDF("Objects/longWall.urdf", [25, -0.5, 1.25], p.getQuaternionFromEuler([0, 0, 0]))  # Back wall
wall_3 = p.loadURDF("Objects/shortWall.urdf", [0.5, 15, 1.25], p.getQuaternionFromEuler([0, 0, 1.57]))  # Right wall\
wall_4 = p.loadURDF("Objects/shortWall.urdf", [50.5, 15, 1.25], p.getQuaternionFromEuler([0, 0, 1.57]))  # Right wall\

object_names = {
    block1: "Block 1",
    block2: "Block 2",
    block3: "Block 3",
    block4: "Block 4",
    goalBlock: "Goal Block",
    wall_1: "Wall 1",
    wall_2: "Wall 2",
    wall_3: "Wall 3",
    wall_4: "Wall 4",
}

camera_distance = 20  # Zoom level (increase or decrease as needed)
camera_yaw = 0       # Horizontal rotation (0 to look straight down)
camera_pitch = -89.9   # Vertical angle, -90 degrees for top-down
camera_target = [25, 15, 0]  # Center the camera on the environment

p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target)

p.stepSimulation()
# Set up simulation properties
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(1)

# Define wheel joint indices (update these based on your URDF joint order)
left_wheel_joint = 1
right_wheel_joint = 2

# Simple control loop to move the robot
for i in range(10000):
    # Set velocity for the wheels
    p.setJointMotorControl2(turtlebot, left_wheel_joint, p.VELOCITY_CONTROL, targetVelocity=5)
    p.setJointMotorControl2(turtlebot, right_wheel_joint, p.VELOCITY_CONTROL, targetVelocity=5)

    contact_points = p.getContactPoints(bodyA=turtlebot)
    for contact in contact_points:
        body_b = contact[2]  # ID of the other object in contact
        if body_b in object_names:
            print("Collision detected with:", object_names[body_b])
    time.sleep(1./240.)  # Adjust simulation step rate

# Disconnect from the simulation
p.disconnect()
