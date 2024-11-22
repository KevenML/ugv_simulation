import pybullet as p
import pybullet_data
import time

# Initialize Pybullet
physicsClient = p.connect(p.GUI)

# Set up the simulation environment
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Load Pybullet's data
p.setGravity(0, 0, -9.8)  # Set gravity

# Add the ground plane
planeId = p.loadURDF("plane.urdf")

# Add obstacles (boxes)
obstacle1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1.5, 0.5, 0.5])
p.createMultiBody(baseCollisionShapeIndex=obstacle1, basePosition=[5, -3, 0.5])

obstacle2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 1, 1])
p.createMultiBody(baseCollisionShapeIndex=obstacle2, basePosition=[-2, -2, 1])

obstacle3 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 1, 1])
p.createMultiBody(baseCollisionShapeIndex=obstacle3, basePosition=[3, 3, 1])

# Create a simple UGV (box body with cylindrical wheels)
# Body
body = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 0.6, 0.3])  # Larger UGV
bodyId = p.createMultiBody(baseCollisionShapeIndex=body, baseMass=1, basePosition=[0, 0, 0.3])

# Wheels
wheel_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.2, height=0.1)

# Define wheel positions relative to the body
wheels = [
    [0.5, 0.3, 0.2],  # Front-left
    [0.5, -0.3, 0.2],  # Front-right
    [-0.5, 0.3, 0.2],  # Rear-left
    [-0.5, -0.3, 0.2],  # Rear-right
]

for position in wheels:
    p.createMultiBody(
        baseCollisionShapeIndex=wheel_shape,
        basePosition=position,
        baseOrientation=p.getQuaternionFromEuler([0, 1.57, 0]),  # Rotate the cylinder for wheels
    )

# Add movable objects
movable_object = p.createCollisionShape(p.GEOM_SPHERE, radius=0.5)
p.createMultiBody(baseCollisionShapeIndex=movable_object, baseMass=1, basePosition=[2, 2, 0.5])

falling_box = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
p.createMultiBody(baseCollisionShapeIndex=falling_box, baseMass=2, basePosition=[0, 0, 5])  # Start high up

# Apply forces and torque to the UGV
p.applyExternalForce(
    objectUniqueId=bodyId,
    linkIndex=-1,  # Apply force to the base
    forceObj=[50, 0, 0],  # Apply a force of 50 in the x-direction
    posObj=[0, 0, 0],  # Apply at the center of the UGV
    flags=p.WORLD_FRAME,
)

p.applyExternalTorque(
    objectUniqueId=bodyId,
    linkIndex=-1,
    torqueObj=[0, 0, 10],  # Rotate around the z-axis
    flags=p.WORLD_FRAME,
)

# Simulate for some time
for _ in range(1000):
    p.stepSimulation()
    time.sleep(0.01)

# Save and restore the state (useful for debugging or resetting the simulation)
stateId = p.saveState()
p.restoreState(stateId)

# Disconnect the simulation
p.disconnect()
