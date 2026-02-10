import pybullet as p
import pybullet_data
import time
import numpy as np

###############################################################################
# OPEN PYBULLET
###############################################################################
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setTimeStep(1/240.0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")

###############################################################################
# LOAD MINI CHEETAH
###############################################################################
# Forward-pitched base orientation (≈ 15° pitch forward)
base_pitch = 0.26   # radians (~15°)

robot = p.loadURDF(
    "mini_cheetah/mini_cheetah.urdf",
    basePosition=[0,0,0.28],
    baseOrientation=p.getQuaternionFromEuler([0, base_pitch, 0]),
    useFixedBase=True
)

time.sleep(0.1)

###############################################################################
# LOCK TOES
###############################################################################
for j in range(p.getNumJoints(robot)):
    name = p.getJointInfo(robot,j)[1].decode()
    if name.startswith("toe"):
        p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, 0, force=30)

###############################################################################
# DESIRED BOUNDING POSE
#
#   Front legs  → tucked upward (you requested SAME as previous)
#   Rear legs   → extended backward
###############################################################################
POSE_FRONT = {
    "hip":  -1.00,    # tuck up
    "knee": +2.20
}

POSE_REAR = {
    "hip":  +0.30,    # push leg backward
    "knee": -0.30     # slight extension
}

###############################################################################
# MAP MINI-CHEETAH JOINTS
###############################################################################
leg_map = {
    "FR": ["hip_fr_j", "knee_fr_j"],
    "FL": ["hip_fl_j", "knee_fl_j"],
    "RR": ["hip_hr_j", "knee_hr_j"],
    "RL": ["hip_hl_j", "knee_hl_j"]
}

joint_ids = {}
for j in range(p.getNumJoints(robot)):
    name = p.getJointInfo(robot,j)[1].decode()
    for L, (H, K) in leg_map.items():
        if name == H: joint_ids[(L,"hip")]  = j
        if name == K: joint_ids[(L,"knee")] = j

###############################################################################
# APPLY POSE
###############################################################################
for L in ["FR", "FL"]:    # FRONT LEGS tucked
    p.setJointMotorControl2(robot, joint_ids[(L,"hip")],
        p.POSITION_CONTROL, POSE_FRONT["hip"], force=80)
    p.setJointMotorControl2(robot, joint_ids[(L,"knee")],
        p.POSITION_CONTROL, POSE_FRONT["knee"], force=80)

for L in ["RR", "RL"]:    # REAR LEGS extended
    p.setJointMotorControl2(robot, joint_ids[(L,"hip")],
        p.POSITION_CONTROL, POSE_REAR["hip"], force=80)
    p.setJointMotorControl2(robot, joint_ids[(L,"knee")],
        p.POSITION_CONTROL, POSE_REAR["knee"], force=80)

###############################################################################
# RUN FOR PICTURES
###############################################################################
print("\n✓ Mini-Cheetah in mid-air BOUNDING pose.")
print("✓ Front legs tucked, rear legs extended, body pitched forward.\n")
print("Move camera around and screenshot!\n")

while True:
    p.stepSimulation()
    time.sleep(1/240)
