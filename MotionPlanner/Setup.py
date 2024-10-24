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
turtlebot = p.loadURDF(turtlebot_path, [0, 0, 0.5])

# Set up simulation properties
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(1)

# Define wheel joint indices (update these based on your URDF joint order)
left_wheel_joint = 1
right_wheel_joint = 2

# Simple control loop to move the robot
for i in range(10000):
    # Set velocity for the wheels
    p.setJointMotorControl2(turtlebot, left_wheel_joint, p.VELOCITY_CONTROL, targetVelocity=50)
    p.setJointMotorControl2(turtlebot, right_wheel_joint, p.VELOCITY_CONTROL, targetVelocity=50)
    time.sleep(1./240.)  # Adjust simulation step rate

# Disconnect from the simulation
p.disconnect()
