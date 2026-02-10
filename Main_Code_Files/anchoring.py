import numpy as np
import pybullet as p

###############################################################################
# TEMPLATE → ANCHOR FEET  (Ψ)
###############################################################################

def template_to_anchor_feet(z, phi, d, rho):
    """
    Returns absolute foot positions in template frame:
    (x_lateral, z_vertical)
    """
    x1 =  d * np.cos(phi)
    z1 =  z + d * np.sin(phi) - rho

    x2 = -d * np.cos(phi)
    z2 =  z - d * np.sin(phi) - rho

    return np.array([x1, z1]), np.array([x2, z2])


###############################################################################
# DIRECT COM MAPPING (NEW)
###############################################################################

def map_template_com_to_anchor(z_template_com, anchor_com):
    """
    Align anchor COM with template COM along vertical axis.
    Returns a 3D position correction vector.
    """
    dz = z_template_com - anchor_com[2]
    return np.array([0.0, 0.0, dz])


###############################################################################
# UTIL: compute anchor COM
###############################################################################

def get_robot_com(robot_id):
    total = 0.0
    weighted = np.zeros(3)

    # Base
    base_pos, _ = p.getBasePositionAndOrientation(robot_id)
    m = p.getDynamicsInfo(robot_id, -1)[0]
    total += m
    weighted += m * np.array(base_pos)

    # Links
    num = p.getNumJoints(robot_id)
    for j in range(num):
        pos = np.array(p.getLinkState(robot_id, j, computeForwardKinematics=True)[0])
        m   = p.getDynamicsInfo(robot_id, j)[0]
        total += m
        weighted += m * pos

    return weighted / total
