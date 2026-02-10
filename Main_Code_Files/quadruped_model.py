import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

###############################################################################
# TEMPLATE PARAMETERS (from your slot hopper)
###############################################################################
d_template = 0.125      # half hip-to-hip distance
rho_template = 0.175    # nominal leg length

###############################################################################
# ANCHOR PARAMETERS
###############################################################################
d_anchor = 0.125        # half hip-to-hip distance (same as template)
L1 = 0.12              # upper leg length
L2 = 0.10              # lower leg length

###############################################################################
# STEP 1: EXTRACT TEMPLATE GEOMETRY - CORRECTED PHYSICS
###############################################################################
def get_template_geometry(z_com, phi, z1, z2, r1, r2):
    """
    Extract geometry from slot hopper using CORRECT contact physics.
    """
    # Hip positions
    x1 = d_template * np.cos(phi)
    x2 = -d_template * np.cos(phi)
    
    # CORRECT CONTACT DETECTION - matches slot hopper event functions
    CONTACT_TOL = 0.001
    
    # Contact occurs when |z_hip - r| < tolerance (from your event functions)
    foot1_z = z1 - r1
    foot2_z = z2 - r2

    contact1 = abs(foot1_z) < CONTACT_TOL
    contact2 = abs(foot2_z) < CONTACT_TOL
        
    # Foot positions - CORRECT LOGIC
    if contact1:
        # Stance: foot at ground
        foot1_z = 0.0
    else:
        # Flight: foot position is hip - leg_length
        foot1_z = z1 - r1
        foot1_z = max(0.0, foot1_z)  # safety clamp
    
    if contact2:
        # Stance: foot at ground
        foot2_z = 0.0
    else:
        # Flight: foot below hip
        foot2_z = z2 - r2
        foot2_z = max(0.0, foot2_z)  # safety clamp
    
    foot1_x = x1
    foot2_x = x2
    
    # Return only 6 values (remove eff_r1, eff_r2 from return)
    return (x1, z1), (x2, z2), (foot1_x, foot1_z), (foot2_x, foot2_z), contact1, contact2

###############################################################################
# STEP 2: MAP TEMPLATE → ANCHOR COM
###############################################################################
def map_com(z_com_template):
    """DIRECT COM MAPPING"""
    return np.array([0.0, z_com_template])


###############################################################################
# STEP 3: COMPUTE ANCHOR HIP POSITIONS
###############################################################################
def compute_anchor_hips(com_anchor, phi, d):
    """Place anchor hips using template body angle"""
    c = np.cos(phi)
    s = np.sin(phi)
    
    hip1 = com_anchor + np.array([d * c, d * s])    # front hip
    hip2 = com_anchor + np.array([-d * c, -d * s])  # rear hip
    
    return hip1, hip2


###############################################################################
# STEP 4: COMPUTE ANCHOR FOOT TARGETS
###############################################################################
def compute_anchor_foot_targets(z1_template, z2_template, 
                                foot1_template, foot2_template, 
                                hip1_anchor, hip2_anchor, 
                                contact1, contact2):
    """
    Map feet preserving the leg extension from template.
    
    Key: leg_extension = hip_z - foot_z
    """
    # Template leg extensions (hip to foot distance)
    leg1_ext = z1_template - foot1_template[1]
    leg2_ext = z2_template - foot2_template[1]
    
    # Apply same extensions in anchor
    foot1_anchor = np.array([hip1_anchor[0], hip1_anchor[1] - leg1_ext])
    foot2_anchor = np.array([hip2_anchor[0], hip2_anchor[1] - leg2_ext])
    
    # If in contact, enforce ground constraint
    if contact1:
        foot1_anchor[1] = 0.0
    if contact2:
        foot2_anchor[1] = 0.0
    
    # Safety clamp
    foot1_anchor[1] = max(foot1_anchor[1], 0.0)
    foot2_anchor[1] = max(foot2_anchor[1], 0.0)
    
    return foot1_anchor, foot2_anchor


###############################################################################
# STEP 5: 2-LINK IK
###############################################################################
def two_link_ik(hip, foot_target, L1, L2):
    """Solve 2-link IK"""
    dx = foot_target[0] - hip[0]
    dz = foot_target[1] - hip[1]
    
    D = np.sqrt(dx**2 + dz**2)
    D = np.clip(D, 0.01, L1 + L2 - 0.01)
    
    # Knee angle
    cos_knee = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    cos_knee = np.clip(cos_knee, -1, 1)
    theta_knee = np.pi - np.arccos(cos_knee)
    
    # Hip angle
    alpha = np.arctan2(dx, -dz)
    cos_beta = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    cos_beta = np.clip(cos_beta, -1, 1)
    beta = np.arccos(cos_beta)
    theta_hip = alpha - beta
    
    return theta_hip, theta_knee


###############################################################################
# STEP 6: FORWARD KINEMATICS
###############################################################################
def forward_kinematics(hip, theta_hip, theta_knee, L1, L2):
    """FK for visualization"""
    knee = hip + np.array([
        L1 * np.sin(theta_hip),
        -L1 * np.cos(theta_hip)
    ])
    
    foot = knee + np.array([
        L2 * np.sin(theta_hip + theta_knee),
        -L2 * np.cos(theta_hip + theta_knee)
    ])
    
    return knee, foot


###############################################################################
# COMPLETE ANCHORING PIPELINE
###############################################################################
def anchor_template_to_quadruped(z_com, phi, z1, z2, r1, r2):
    """
    Complete pipeline with proper contact detection
    """
    # STEP 1: Extract template geometry
    (x1_t, z1_t), (x2_t, z2_t), foot1_t, foot2_t, contact1, contact2 = get_template_geometry(
        z_com, phi, z1, z2, r1, r2
    )
    
    # STEP 2: Map COM
    com_anchor = map_com(z_com)
    
    # STEP 3: Compute anchor hips
    hip1_anchor, hip2_anchor = compute_anchor_hips(com_anchor, phi, d_anchor)
    
    # STEP 4: Compute anchor foot targets
    foot1_target, foot2_target = compute_anchor_foot_targets(
        z1_t, z2_t, foot1_t, foot2_t, hip1_anchor, hip2_anchor, contact1, contact2
    )
    
    # STEP 5: Solve IK
    theta_hip1, theta_knee1 = two_link_ik(hip1_anchor, foot1_target, L1, L2)
    theta_hip2, theta_knee2 = two_link_ik(hip2_anchor, foot2_target, L1, L2)
    
    # STEP 6: FK
    knee1, foot1_actual = forward_kinematics(hip1_anchor, theta_hip1, theta_knee1, L1, L2)
    knee2, foot2_actual = forward_kinematics(hip2_anchor, theta_hip2, theta_knee2, L1, L2)
    
    return {
        'template': {
            'hip1': (x1_t, z1_t),
            'hip2': (x2_t, z2_t),
            'foot1': foot1_t,
            'foot2': foot2_t,
            'com': (0, z_com),
            'leg1_extension': z1_t - foot1_t[1],
            'leg2_extension': z2_t - foot2_t[1],
            'contact1': contact1,
            'contact2': contact2,
            'r1': r1,
            'r2': r2
        },
        'anchor': {
            'com': com_anchor,
            'hip1': hip1_anchor,
            'hip2': hip2_anchor,
            'foot1_target': foot1_target,
            'foot2_target': foot2_target,
            'foot1_actual': foot1_actual,
            'foot2_actual': foot2_actual,
            'knee1': knee1,
            'knee2': knee2,
            'joint_angles': (theta_hip1, theta_knee1, theta_hip2, theta_knee2)
        }
    }


###############################################################################
# VISUALIZATION
###############################################################################
def visualize_anchoring(template_trajectory):
    """Animate the template → anchor mapping"""
    z_com = template_trajectory['z_com']
    phi = template_trajectory['phi']
    z1 = template_trajectory['z1']
    z2 = template_trajectory['z2']
    r1 = template_trajectory['r1']
    r2 = template_trajectory['r2']
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))
    
    def draw_frame(i):
        ax1.clear()
        ax2.clear()
        
        result = anchor_template_to_quadruped(z_com[i], phi[i], z1[i], z2[i], r1[i], r2[i])
        t = result['template']
        a = result['anchor']
        
        # LEFT PLOT: Template
        ax1.set_xlim(-0.4, 0.4)
        ax1.set_ylim(-0.05, 0.8)
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        ax1.set_title('TEMPLATE (Slot Hopper)', fontsize=14, fontweight='bold')
        ax1.axhline(y=0, color='brown', linewidth=3, label='Ground')
        
        # Template body
        ax1.plot([t['hip1'][0], t['hip2'][0]], 
                [t['hip1'][1], t['hip2'][1]], 
                'b-', linewidth=8, alpha=0.6, label='Body')
        
        # Template legs (styled by contact state)
        leg1_color = 'darkgreen' if t['contact1'] else 'lime'
        leg2_color = 'darkred' if t['contact2'] else 'lightcoral'
        leg1_width = 5 if t['contact1'] else 3
        leg2_width = 5 if t['contact2'] else 3
        leg1_style = '-' if t['contact1'] else '--'
        leg2_style = '-' if t['contact2'] else '--'
        
        ax1.plot([t['hip1'][0], t['foot1'][0]], 
                [t['hip1'][1], t['foot1'][1]], 
                leg1_style, color=leg1_color, linewidth=leg1_width, alpha=0.9,
                label=f'Leg 1')
        ax1.plot([t['hip2'][0], t['foot2'][0]], 
                [t['hip2'][1], t['foot2'][1]], 
                leg2_style, color=leg2_color, linewidth=leg2_width, alpha=0.9,
                label=f'Leg 2')
        
        # Template points
        ax1.plot(t['com'][0], t['com'][1], 'r*', markersize=20, label='Body COM')
        ax1.plot(t['hip1'][0], t['hip1'][1], 'go', markersize=12)
        ax1.plot(t['hip2'][0], t['hip2'][1], 'mo', markersize=12)
        ax1.plot(t['foot1'][0], t['foot1'][1], 'gs', markersize=12)
        ax1.plot(t['foot2'][0], t['foot2'][1], 'ms', markersize=12)
        
        ax1.legend(loc='upper right', fontsize=8)

        #Extension = {t['leg1_extension']:.4f}m
        #|z1-r1| = {abs(z1[i]-r1[i]):.5f}m
        #Status: {"CONTACT" if t['contact1'] else "FLIGHT"}


    
        # DETAILED LOGGING - TEMPLATE
        info_text = f"""TEMPLATE STATE (Frame {i}):
COM: z={z_com[i]:.4f}m, φ={np.degrees(phi[i]):.1f}°

LEG 1 (Front):
  Hip z1 = {z1[i]:.4f}m
  Leg r1 = {r1[i]:.4f}m
  Foot z = {t['foot1'][1]:.4f}m
  
LEG 2 (Rear):
  Hip z2 = {z2[i]:.4f}m
  Leg r2 = {r2[i]:.4f}m
  Foot z = {t['foot2'][1]:.4f}m"""
        
        ax1.text(0.02, 0.98, info_text,
                transform=ax1.transAxes, 
                fontsize=8,
                verticalalignment='top',
                bbox=dict(facecolor='white', alpha=0.9),
                family='monospace')
        
        # RIGHT PLOT: Anchor
        ax2.set_xlim(-0.4, 0.4)
        ax2.set_ylim(-0.05, 0.8)
        ax2.set_aspect('equal')
        ax2.grid(True, alpha=0.3)
        ax2.set_title('ANCHOR (2-Link Quadruped)', fontsize=14, fontweight='bold')
        ax2.axhline(y=0, color='brown', linewidth=3)
        
        # Anchor body
        ax2.plot([a['hip1'][0], a['hip2'][0]], 
                [a['hip1'][1], a['hip2'][1]], 
                'b-', linewidth=8, alpha=0.6)
        
        # Anchor legs (match template contact styling)
        anc1_color = 'darkgreen' if t['contact1'] else 'lime'
        anc2_color = 'darkred' if t['contact2'] else 'lightcoral'
        anc1_width = 5 if t['contact1'] else 3
        anc2_width = 5 if t['contact2'] else 3
        
        ax2.plot([a['hip1'][0], a['knee1'][0]], 
                [a['hip1'][1], a['knee1'][1]], 
                '-', color=anc1_color, linewidth=anc1_width)
        ax2.plot([a['knee1'][0], a['foot1_actual'][0]], 
                [a['knee1'][1], a['foot1_actual'][1]], 
                '-', color=anc1_color, linewidth=anc1_width-1, alpha=0.8)
        
        ax2.plot([a['hip2'][0], a['knee2'][0]], 
                [a['hip2'][1], a['knee2'][1]], 
                '-', color=anc2_color, linewidth=anc2_width)
        ax2.plot([a['knee2'][0], a['foot2_actual'][0]], 
                [a['knee2'][1], a['foot2_actual'][1]], 
                '-', color=anc2_color, linewidth=anc2_width-1, alpha=0.8)
        
        # Anchor points
        ax2.plot(a['com'][0], a['com'][1], 'r*', markersize=20)
        ax2.plot(a['hip1'][0], a['hip1'][1], 'go', markersize=12)
        ax2.plot(a['hip2'][0], a['hip2'][1], 'mo', markersize=12)
        ax2.plot(a['knee1'][0], a['knee1'][1], 'go', markersize=8)
        ax2.plot(a['knee2'][0], a['knee2'][1], 'mo', markersize=8)
        ax2.plot(a['foot1_actual'][0], a['foot1_actual'][1], 'gs', markersize=12)
        ax2.plot(a['foot2_actual'][0], a['foot2_actual'][1], 'ms', markersize=12)
        
        # Target feet (X markers)
        ax2.plot(a['foot1_target'][0], a['foot1_target'][1], 'gx', 
                markersize=10, alpha=0.5, markeredgewidth=2)
        ax2.plot(a['foot2_target'][0], a['foot2_target'][1], 'mx', 
                markersize=10, alpha=0.5, markeredgewidth=2)
        
        # DETAILED LOGGING - ANCHOR
        th1, tk1, th2, tk2 = a['joint_angles']
        err1 = np.linalg.norm(a['foot1_actual'] - a['foot1_target'])
        err2 = np.linalg.norm(a['foot2_actual'] - a['foot2_target'])
        
        anchor_text = f"""ANCHOR STATE:
COM: (x={a['com'][0]:.3f}, z={a['com'][1]:.3f})m

LEG 1 (Front):
  Hip: ({a['hip1'][0]:.3f}, {a['hip1'][1]:.3f})
  Target Foot: {a['foot1_target'][1]:.4f}m
  Actual Foot: {a['foot1_actual'][1]:.4f}m
  θ_hip = {np.degrees(th1):+.1f}°
  θ_knee = {np.degrees(tk1):+.1f}°
  IK Error = {err1*1000:.2f}mm

LEG 2 (Rear):
  Hip: ({a['hip2'][0]:.3f}, {a['hip2'][1]:.3f})
  Target Foot: {a['foot2_target'][1]:.4f}m
  Actual Foot: {a['foot2_actual'][1]:.4f}m
  θ_hip = {np.degrees(th2):+.1f}°
  θ_knee = {np.degrees(tk2):+.1f}°
  IK Error = {err2*1000:.2f}mm"""
        
        ax2.text(0.02, 0.98, anchor_text,
                transform=ax2.transAxes,
                fontsize=8,
                verticalalignment='top',
                bbox=dict(facecolor='lightyellow', alpha=0.9),
                family='monospace')
        
        plt.tight_layout()
    
    ani = FuncAnimation(fig, draw_frame, frames=len(z_com), interval=20, repeat=True)
    plt.show()
    return ani

###############################################################################
# OVERLAID TEMPLATE vs ANCHOR PLOTS (3 FIGURES)
###############################################################################
def plot_template_anchor_overlaid(template_data):
    """
    Produce 3 publication-quality figures with template and anchor overlaid:
    1) COM height
    2) Body pitch
    3) Foot heights (front + rear)
    """
    # Extract template values
    z_com = template_data["z_com"]
    phi   = template_data["phi"]
    z1    = template_data["z1"]
    z2    = template_data["z2"]
    r1    = template_data["r1"]
    r2    = template_data["r2"]

    N = len(z_com)
    time = np.arange(N) * 0.001   # adjust if needed

    # Storage for anchor trajectories
    anc_com = np.zeros(N)
    anc_pitch = np.zeros(N)
    anc_foot1 = np.zeros(N)
    anc_foot2 = np.zeros(N)

    # Template foot positions
    foot1_temp = np.clip(z1 - r1, 0, None)
    foot2_temp = np.clip(z2 - r2, 0, None)

    for i in range(N):
        result = anchor_template_to_quadruped(
            z_com[i], phi[i], z1[i], z2[i], r1[i], r2[i]
        )
        a = result["anchor"]

        anc_com[i] = a["com"][1]
        anc_foot1[i] = a["foot1_actual"][1]
        anc_foot2[i] = a["foot2_actual"][1]

        # Anchor pitch = angle between hips
        dx = a["hip1"][0] - a["hip2"][0]
        dz = a["hip1"][1] - a["hip2"][1]
        anc_pitch[i] = np.arctan2(dz, dx)

    # ------------------------------------------------------------
    #  COLORS
    # ------------------------------------------------------------
    template_color = (0.0, 0.0, 0.5)    # light blue
    anchor_color   = (0.4, 0.4, 0.9)    # dark navy

    temp_front = (0.0, 0.4, 0.0)        # light green
    anch_front = (0.3, 0.8, 0.3)        # dark green

    temp_rear  =  (0.6, 0.0, 0.0)       # light red/pink
    anch_rear  = (1.0, 0.6, 0.6)       # dark red

    lw_template = 4.0
    lw_anchor = 1.5

    # ============================================================
    #  FIGURE 1 — COM HEIGHT
    # ============================================================
    plt.figure(figsize=(10,5))
    plt.plot(time, z_com, color=template_color, linewidth=lw_template, label="Template COM")
    plt.plot(time, anc_com, color=anchor_color, linewidth=lw_anchor, linestyle='--', label="Anchor COM")

    #plt.plot(time, anc_com, color=anchor_color, linewidth=lw_anchor, label="Anchor COM")

    plt.title("COM Height: Template vs Anchor", fontsize=14)
    plt.xlabel("Time (s)")
    plt.ylabel("Height (m)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # ============================================================
    #  FIGURE 2 — BODY PITCH
    # ============================================================
    plt.figure(figsize=(10,5))
    plt.plot(time, np.degrees(phi), color=template_color, linewidth=lw_template,
             label="Template Pitch")
    plt.plot(time, np.degrees(anc_pitch), color=anchor_color, linewidth=lw_anchor, linestyle='--', label="Anchor Pitch")

    #plt.plot(time, np.degrees(anc_pitch), color=anchor_color, linewidth=lw_anchor,
    #         label="Anchor Pitch")

    plt.title("Body Pitch: Template vs Anchor", fontsize=14)
    plt.xlabel("Time (s)")
    plt.ylabel("Pitch (deg)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # ============================================================
    #  FIGURE 3 — FOOT HEIGHTS
    # ============================================================
    plt.figure(figsize=(10,5))

    # Template front + rear
    plt.plot(time, foot1_temp, color=temp_front, linewidth=lw_template,
             label="Template Front Foot")
    plt.plot(time, foot2_temp, color=temp_rear, linewidth=lw_template,
             label="Template Rear Foot")

    # Anchor front + rear
    #plt.plot(time, anc_foot1, color=anch_front, linewidth=lw_anchor,
    #         label="Anchor Front Foot")
    plt.plot(time, anc_foot1, color=anch_front, linewidth=lw_anchor, linestyle='--', label="Anchor Front Foot")

    plt.plot(time, anc_foot2, color=anch_rear, linewidth=lw_anchor, linestyle='--', label="Anchor Rear Foot")

    #plt.plot(time, anc_foot2, color=anch_rear, linewidth=lw_anchor,
    #         label="Anchor Rear Foot")

    plt.title("Foot Heights: Template vs Anchor (Pronking)", fontsize=14)
    plt.xlabel("Time (s)")
    plt.ylabel("Foot Height (m)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()

def plot_joint_angles(time, hip1, knee1, hip2, knee2):
    """
    Plot joint angle trajectories for the anchored quadruped.
    time  : vector of timestamps
    hip1  : front hip angle (rad)
    knee1 : front knee angle (rad)
    hip2  : rear hip angle (rad)
    knee2 : rear knee angle (rad)
    """

    # Convert to degrees for readability
    hip1_deg  = np.degrees(hip1)
    knee1_deg = np.degrees(knee1)
    hip2_deg  = np.degrees(hip2)
    knee2_deg = np.degrees(knee2)

    # Colors (template darker, anchor lighter if needed)
    front_color = "tab:green"
    rear_color  = "tab:red"

    lw_front = 2.5
    lw_rear  = 2.5

    ls_front = "-"
    ls_rear  = "--"       # differentiate front/rear legs

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # ---------------------------------------
    # TOP PLOT: HIP ANGLES
    # ---------------------------------------
    ax1.plot(time, hip1_deg, color=front_color, linewidth=lw_front, linestyle=ls_front,
             label="Front Hip")
    ax1.plot(time, hip2_deg, color=rear_color, linewidth=lw_rear, linestyle=ls_rear,
             label="Rear Hip")

    ax1.set_ylabel("Hip Angle (deg)", fontsize=12)
    ax1.set_title("Joint Angle Trajectories — Hip", fontsize=14)
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    # ---------------------------------------
    # BOTTOM PLOT: KNEE ANGLES
    # ---------------------------------------
    ax2.plot(time, knee1_deg, color=front_color, linewidth=lw_front, linestyle=ls_front,
             label="Front Knee")
    ax2.plot(time, knee2_deg, color=rear_color, linewidth=lw_rear, linestyle=ls_rear,
             label="Rear Knee")

    ax2.set_ylabel("Knee Angle (deg)", fontsize=12)
    ax2.set_xlabel("Time (s)", fontsize=12)
    ax2.set_title("Joint Angle Trajectories — Knee", fontsize=14)
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    plt.tight_layout()
    plt.show()

###############################################################################
# MAIN
###############################################################################
if __name__ == "__main__":
    template_data = np.load("template_trajectory.npy", allow_pickle=True).item()
    
    # print("="*70)
    # print("CORRECTED: PROPER SLOT HOPPER CONTACT PHYSICS")
    # print("="*70)
    # print("\nContact Detection (from your event functions):")
    # print("  - Contact constraint: z_hip = r (when foot touches ground)")
    # print("  - Foot position: z_foot = z_hip - r")
    # print("  - Contact occurs when: |z_hip - r| < 0.001m")
    # print("\nDuring STANCE: r is CONSTANT (rdot = 0)")
    # print("During FLIGHT: r changes as leg extends/compresses")
    # print("="*70)
    
    # Debug frames
    # print("\nDEBUGGING FRAMES:")
    # for i in [0, 100, 200, 500, 1000]:
    #     if i < len(template_data['z1']):
    #         z1 = template_data['z1'][i]
    #         z2 = template_data['z2'][i]
    #         r1 = template_data['r1'][i]
    #         r2 = template_data['r2'][i]
    #         print(f"\nFrame {i}:")
    #         print(f"  Leg 1: z1={z1:.4f}, r1={r1:.4f}, |z1-r1|={abs(z1-r1):.5f} → {'CONTACT' if abs(z1-r1)<0.001 else 'FLIGHT'}")
    #         print(f"         foot_z = z1-r1 = {z1-r1:.5f}m")
    #         print(f"  Leg 2: z2={z2:.4f}, r2={r2:.4f}, |z2-r2|={abs(z2-r2):.5f} → {'CONTACT' if abs(z2-r2)<0.001 else 'FLIGHT'}")
    #         print(f"         foot_z = z2-r2 = {z2-r2:.5f}m")
    
    # print("\n✓ Starting visualization with detailed logging...")
    # print("\nLOGGING DISPLAYED IN PLOTS:")
    # print("  LEFT (Template):")
    # print("    - Frame number")
    # print("    - COM height and pitch angle")
    # print("    - For each leg: hip z, leg length r, |z-r|, foot z, extension, contact status")
    # print("  RIGHT (Anchor):")
    # print("    - COM position")
    # print("    - For each leg: hip position, target/actual foot z, joint angles, IK error")
    # print("\n" + "="*70)
    
    ani = visualize_anchoring(template_data)
    #plot_template_anchor_overlaid(template_data)
    #plot_template_anchor_overlaid(template_data)

    # ---------------------------------------
    # Collect joint angles over all frames
    # ---------------------------------------
    z_com = template_data["z_com"]
    phi   = template_data["phi"]
    z1    = template_data["z1"]
    z2    = template_data["z2"]
    r1    = template_data["r1"]
    r2    = template_data["r2"]

    N_frames = len(z_com)
    T2 = np.arange(N_frames) * 0.001

    hip1_list  = []
    knee1_list = []
    hip2_list  = []
    knee2_list = []

    for i in range(N_frames):
        A = anchor_template_to_quadruped(z_com[i], phi[i], z1[i], z2[i], r1[i], r2[i])
        th1, tk1, th2, tk2 = A['anchor']['joint_angles']

        hip1_list.append(th1)
        knee1_list.append(tk1)
        hip2_list.append(th2)
        knee2_list.append(tk2)

    hip1_list  = np.array(hip1_list)
    knee1_list = np.array(knee1_list)
    hip2_list  = np.array(hip2_list)
    knee2_list = np.array(knee2_list)

    # ---------------------------------------
    # PLOT JOINT ANGLES
    # ---------------------------------------
    #plot_joint_angles(T2, hip1_list, knee1_list, hip2_list, knee2_list)
