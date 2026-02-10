{"id":"12945","variant":"standard","title":"quadpybullet_2d_plain_fixed"}
import pybullet as p
import pybullet_data
import numpy as np
import time


#######################################################################
# PARAMETERS
#######################################################################
TIMESTEP = 1.0/240.0
KP = 40
KD = 1.5
MAX_TORQUE = 60

# template geometry
d_template   = 0.125
rho_template = 0.175

# mini cheetah legs
L1 = 0.209
L2 = 0.180
SCALE = (L1+L2) / rho_template

#######################################################################
# LOAD TEMPLATE TRAJECTORY
#######################################################################
data = np.load("template_trajectory.npy", allow_pickle=True).item()
z_traj   = np.array(data["z_com"])
phi_traj = np.array(data["phi"])
r1_traj  = np.array(data["r1"])
r2_traj  = np.array(data["r2"])

#######################################################################
# TEMPLATE FOOT POSITIONS
#######################################################################
def template_feet(z, phi, dT, r1, r2):
    x1 =  dT*np.cos(phi)
    z1 =  z + dT*np.sin(phi)
    x2 = -dT*np.cos(phi)
    z2 =  z - dT*np.sin(phi)
    return np.array([x1, z1 - r1]), np.array([x2, z2 - r2])

#######################################################################
# IK
#######################################################################
def ik_yz(dy, dz, L1, L2):
    D = np.sqrt(dy*dy + dz*dz)
    D = np.clip(D, 1e-6, L1+L2-1e-3)

    ck = (L1*L1 + L2*L2 - D*D)/(2*L1*L2)
    knee = np.pi - np.arccos(np.clip(ck, -1, 1))

    phi = np.arctan2(dy, -dz)
    cb = (L1*L1 + D*D - L2*L2)/(2*L1*D)
    hip = phi - np.arccos(np.clip(cb, -1, 1))
    return hip, knee

#######################################################################
# INIT PHYSICS
#######################################################################
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)
p.setTimeStep(TIMESTEP/10)

p.loadURDF("plane.urdf")

robot = p.loadURDF(
    "mini_cheetah/mini_cheetah.urdf",
    basePosition=[0,0,0.90],
    useFixedBase=True
)

time.sleep(0.1)

#######################################################################
# LOCK TOES
#######################################################################
for j in range(p.getNumJoints(robot)):
    name = p.getJointInfo(robot,j)[1].decode()
    if name.startswith("toe"):
        p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, 0, force=30)

#######################################################################
# MAP HIPS / KNEES
#######################################################################
leg_map = {
    "FR": ["hip_fr_j", "knee_fr_j"],
    "FL": ["hip_fl_j", "knee_fl_j"],
    "RR": ["hip_hr_j", "knee_hr_j"],
    "RL": ["hip_hl_j", "knee_hl_j"]
}

leg_ids = {}
for j in range(p.getNumJoints(robot)):
    name = p.getJointInfo(robot,j)[1].decode()
    for L, (H,K) in leg_map.items():
        if name == H: leg_ids[(L,'hip')] = j
        if name == K: leg_ids[(L,'knee')] = j

#######################################################################
# GET HIP POSE
#######################################################################
def get_hips():
    out = {}
    for L in ["FR","FL","RR","RL"]:
        hip_id = leg_ids[(L,"hip")]
        st = p.getLinkState(robot, hip_id, computeForwardKinematics=True)
        pos = np.array(st[4])
        R = np.array(p.getMatrixFromQuaternion(st[5])).reshape(3,3)
        out[L] = (pos, R)
    return out

#######################################################################
# 2D LOCK (WORKING VERSION)
#
# We "fake" 2D:
#   - force COM.x, COM.y → 0 every step
#   - force roll=0, yaw=0 each step
#   - allow z (vertical)
#   - allow pitch (controlled by PD)
#######################################################################
def enforce_2d():
    pos, orn = p.getBasePositionAndOrientation(robot)
    vel, avel = p.getBaseVelocity(robot)

    # Decompose orientation
    roll, pitch, yaw = p.getEulerFromQuaternion(orn)

    # Enforce 2D constraints:
    corrected_pos = [0.0, 0.0, pos[2]]   # X=0, Y=0
    corrected_orn = p.getQuaternionFromEuler([0.0, pitch, 0.0]) # roll=0, yaw=0

    p.resetBasePositionAndOrientation(robot, corrected_pos, corrected_orn)
    p.resetBaseVelocity(robot, [0,0,vel[2]], [0,avel[1],0])

#######################################################################
# COM + PITCH PD
#######################################################################
KP_COM = 400
KD_COM = 40
KP_PIT = 200
KD_PIT = 15

#######################################################################
# MAIN LOOP
#######################################################################
steps = len(z_traj)

for k in range(steps):

    # Enforce 2D BEFORE anything
    enforce_2d()

    z   = float(z_traj[k])
    phi = float(phi_traj[k])
    r1  = float(r1_traj[k])
    r2  = float(r2_traj[k])

    Ff, Fr = template_feet(z, phi, d_template, r1, r2)
    tf = np.array([0,0,(Ff[1]-z)*SCALE])
    tr = np.array([0,0,(Fr[1]-z)*SCALE])

    hips = get_hips()

    targets = {"FR":tf,"FL":tf,"RR":tr,"RL":tr}

    # IK
    for L in ["FR","FL","RR","RL"]:
        hip_pos, R = hips[L]
        local = R.T @ targets[L]

        hip_raw, knee_raw = ik_yz(local[1], local[2], L1, L2)

        hip_cmd  = -hip_raw
        knee_cmd = -knee_raw

        hip_cmd  = np.clip(hip_cmd, -1.0, 1.0)
        knee_cmd = np.clip(knee_cmd, -2.2, -0.05)

        p.setJointMotorControl2(robot, leg_ids[(L,"hip")],
            p.POSITION_CONTROL, hip_cmd, force=MAX_TORQUE,
            positionGain=KP, velocityGain=KD)

        p.setJointMotorControl2(robot, leg_ids[(L,"knee")],
            p.POSITION_CONTROL, knee_cmd, force=MAX_TORQUE,
            positionGain=KP, velocityGain=KD)


    # COM PD
    pos, orn = p.getBasePositionAndOrientation(robot)
    vel, avel = p.getBaseVelocity(robot)

    z_err = z - pos[2]
    Fz = KP_COM*z_err + KD_COM*(-vel[2])

    p.applyExternalForce(robot, -1, [0,0,Fz], pos, p.WORLD_FRAME)

    # Pitch PD
    roll, pitch, yaw = p.getEulerFromQuaternion(orn)
    pitch_err = phi - pitch
    Ty = KP_PIT*pitch_err + KD_PIT*(-avel[1])

    p.applyExternalTorque(robot, -1, [0,Ty,0], p.WORLD_FRAME)

    p.stepSimulation()
    time.sleep(1/240)

print("\n✓ Finished 2D playback.\n")
p.disconnect()
