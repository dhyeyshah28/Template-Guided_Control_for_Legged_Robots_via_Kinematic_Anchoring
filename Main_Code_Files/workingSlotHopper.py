import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Ellipse
from matplotlib.lines import Line2D
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation


# Parameters for Sim
param = {
    'g': 9.8,                       # Gravity in m/s^2 ==  Tested OK
    'rho': 0.175,                    # Nominal leg length in m == Tested OK for small & large vals both 
    'm': 2.5,                       # Mass in kg == Semi Tested
    'k': 300,                       # Spring constant in N/m == Semi Tested
    #'alpha': m
    #'b': 1,                        # Damping coefficient in N·s/m == 
    'kappa': 1.0,                   # Non-dimensional inertia parameter ~~ Higher = Pronking (All 4) == Tested OK
    'd': 0.125,                     # Half hip-to-hip distance in m == Tested OK
    #'omega': 50,                   # Natural frequency in rad/s == Tested OK
    'beta': 20,                     # Damping in N/(m/s)/kg == Tested OK (4)
    'ka': 40.5,                      # Vertical gain in N == Tested OK
    'kp': 0.1,                      # Attitude control gain ~~ Higher = Less Osc between legs, More double Stance == Tested OK
    'kd': 0.1,                    # Phase/Velocity control gain ~~ Attitude Mode (dampen oscillatory tilting), Phase Mode (alters leg coordination, thrust timing)
    'epsilon': 0.9,                 # Perturbation parameter == Tested OK
    'use_phase_control': False,     # ~~ False = Kp = Attitude, True = Kd (Phase Control) == Tested OK
    'thrust_threshold': 0,          # Threshold for thrust visualization in N
    'thrusttime': 0.005             # Duration for thrust visualization in sec
    # ratio of Ka, beta = Stability
    # Front and Back Leg stance phases bg color
    #mass = 2.5kg, rho = 0.175m, d = 0.125m, k = 300, kp = 300 (Passive), kd = 10 (Passive), ka = 4.5, kp = 0.1, kd = -0.15(Bound), 0.1(pronk)
}

# Initial state
# [z, zdot, phi, phidot, r1, r1dot, r2, r2dot]

Q0 = np.array([0.50, 0.0, 0.0, 0.1, param['rho'], 0.0, param['rho'], 0.0])

alpha = param['m']/(1+ (1/param['kappa']))
omega = np.sqrt(param['k']/alpha)


#b = param['beta']/2*alpha*omega

#beta = param['b']/2*alpha*omega

#(4=flight, 1=stance1, 2=stance2, 3=double stance)
phase = 4  

# Simulation time
tbegin, tfinal = 0.0, 15.0  


# Initializing data arrays
T = [tbegin]
Q = [Q0]
thrust_times_1 = []  
thrust_times_2 = []  


###
def get_psi_a(z, zdot, in_stance):
    """Calculating phase and energy coordinates"""
    if in_stance:
        #p = np.array([-zdot, (param['rho']-z)*param['omega']])
        p = np.array([-zdot, (param['rho']-z)*omega])
        a = np.linalg.norm(p)
        psi = np.arctan2(p[1], p[0])
    else:
        arg = 2*param['g']*(z-param['rho']) + zdot**2
        if arg >= 0:
            a = np.sqrt(arg)
            psi = (a - zdot)/(2*a) if a != 0 else 0.0
        else:
            a = 0.0
            psi = 0.0
    return psi, a


###
def control_inputs(Q, param, phase, t):
    """Calculating control inputs u1 and u2 with proper liftoff conditions"""
    z, zdot, phi, phidot, r1, r1dot, r2, r2dot = Q
    z1 = z + param['d']*phi
    z1dot = zdot + param['d']*phidot
    z2 = z - param['d']*phi
    z2dot = zdot - param['d']*phidot
    
    # phase and energy coordinates
    psi1, a1 = get_psi_a(z1, z1dot, phase in [1,3])
    psi2, a2 = get_psi_a(z2, z2dot, phase in [2,3])
    

    # Vertical control components
    v1 = -param['beta']*z1dot - param['ka']*np.cos(psi1)
    v2 = -param['beta']*z2dot - param['ka']*np.cos(psi2)
    

    # Attitude/phase control components
    if param['use_phase_control']:   # Works when param is set to True

        # Phase control from paper
        # w1 = (-1)**0 * param['kd'] * (z1dot - z2dot) * np.sin(psi1)
        # w2 = (-1)**1 * param['kd'] * (z1dot - z2dot) * np.sin(psi2)

        psi1, a1 = get_psi_a(z1, z1dot, phase in [1,3])
        psi2, a2 = get_psi_a(z2, z2dot, phase in [2,3])

        psi1dot = -z1ddot/a1   if a1 > 1e-6 else 0.0
        psi2dot = -z2ddot/a2   if a2 > 1e-6 else 0.0

        w1 = (-1)**0 * param['kd'] * (psi1dot - psi2dot)
        w2 = (-1)**1 * param['kd'] * (psi1dot - psi2dot)


    else:                            # Works when param is set to False (Should be Default)
        # PD control for attitude
        #w1 = (-1)**0 * (param['kp']*(z1 - z2) + param['kd']*(z1dot - z2dot))
        #w2 = (-1)**1 * (param['kp']*(z1 - z2) + param['kd']*(z1dot - z2dot))

        w1 = -(-1)**0 * (param['kp']*phi + param['kd']*phidot)
        w2 = -(-1)**1 * (param['kp']*phi + param['kd']*phidot)


    #phi_f = 
        
    
    # Total control inputs
    #u1 = param['omega']**2 * (param['rho'] - z1) + param['epsilon'] * (v1 + w1)
    #u2 = param['omega']**2 * (param['rho'] - z2) + param['epsilon'] * (v2 + w2)

    u1 = omega**2 * (param['rho'] - z1) + param['epsilon'] * (v1 + w1)
    u2 = omega**2 * (param['rho'] - z2) + param['epsilon'] * (v2 + w2)
    
    # Only applying thrust when leg is inn contact
    if phase in [1,3] and u1 > 0 and (z1 - r1) <= 0.001:
        thrust_times_1.append((t, t + param['thrusttime']))
    #else:
    #    u1 = 0  

    if phase in [2,3] and u2 > 0 and (z2 - r2) <= 0.001:
        thrust_times_2.append((t, t + param['thrusttime']))
    #else:
    #    u2 = 0  
        
    return u1, u2


###
# Equations of Motion
# def EOMFlight(t, Q, param):
#     """Flight phase"""
#     zdot = Q[1]
#     phidot = Q[3]
#     r1dot = Q[5]
#     r2dot = Q[7]
#     return np.array([zdot, -param['g'], phidot, 0, r1dot, 0, r2dot, 0])



def EOMFlight(t, Q, param):
    z, zdot, phi, phidot, r1, _, r2, _ = Q

    # CRITICAL FIX: During flight, legs should extend to nominal length rho
    # Not remain at their previous compressed lengths
    r1_target = param['rho']  # Nominal leg length
    r2_target = param['rho']  # Nominal leg length
    
    # Smooth transition to nominal length (or instant reset)
    r1dot = (r1_target - r1) * 10.0  # Fast convergence to nominal length
    r2dot = (r2_target - r2) * 10.0

    return np.array([
        zdot,
        -param['g'],
        phidot,
        0.0,
        r1dot,    # Legs extend to nominal length during flight
        r1dot,
        r2dot,    # Legs extend to nominal length during flight  
        r2dot
    ])


def EOMStance(t, Q, param, phase):
    """Modified stance phase dynamics with proper liftoff"""
    u1, u2 = control_inputs(Q, param, phase, t)
    z, zdot, phi, phidot, r1, r1dot, r2, r2dot = Q
    

    #liftoff detection, checking both position and force
    leg1_contact = (z + param['d']*phi <= r1 + 0.001) and (u1 > param['thrust_threshold'])
    leg2_contact = (z - param['d']*phi <= r2 + 0.001) and (u2 > param['thrust_threshold'])
    

    # Checking if we should transition to flight
    if phase == 1 and not leg1_contact:
        return EOMFlight(t, Q, param)
    if phase == 2 and not leg2_contact:
        return EOMFlight(t, Q, param)
    if phase == 3 and not (leg1_contact and leg2_contact):
        # Transition to single stance if one leg lifts off
        if not leg1_contact and leg2_contact:
            phase = 2
        elif leg1_contact and not leg2_contact:
            phase = 1
        else:
            return EOMFlight(t, Q, param)
    

    # Normal stance dynamics
    if phase == 1:
        z1ddot = u1
        z2ddot = -param['g'] - (1-param['kappa'])/(1+param['kappa']) * (u1 + param['g'])
        zddot = (z1ddot + z2ddot)/2
        phiddot = (z1ddot - z2ddot)/(2*param['d'])
        
        return np.array([
        zdot,
        zddot,
        phidot,
        phiddot,
        r1,
        0.0,
        r2,
        0.0
        ])

        #return np.array([Q[1], zddot, Q[3], phiddot, Q[5], z1ddot + param['g'], Q[7], z2ddot + param['g']])
    elif phase == 2:
        z2ddot = u2
        z1ddot = -param['g'] - (1-param['kappa'])/(1+param['kappa']) * (u2 + param['g'])
        zddot = (z1ddot + z2ddot)/2
        phiddot = (z1ddot - z2ddot)/(2*param['d'])

        return np.array([
            zdot,
            zddot,
            phidot,
            phiddot,
            r1,
            0.0,
            r2,
            0.0
        ])

        #return np.array([Q[1], zddot, Q[3], phiddot, Q[5], z1ddot + param['g'], Q[7], z2ddot + param['g']])
    else:  # phase == 3
        zddot = (u1 + u2)/2
        phiddot = (u1 - u2)/(2*param['d']*param['kappa'])
        return np.array([
            zdot,
            zddot,
            phidot,
            phiddot,
            r1,
            0.0,
            r2,
            0.0
        ])

        #return np.array([Q[1], zddot, Q[3], phiddot, Q[5], u1 + param['g'], Q[7], u2 + param['g']])



# Event functions
def make_event_touchdown1(param):
    def event(t, Q):
        z, _, phi, _, r1, _, _, _ = Q
        return z + param['d']*phi - r1
    event.terminal = True
    event.direction = -1
    return event

def make_event_touchdown2(param):
    def event(t, Q):
        z, _, phi, _, _, _, r2, _ = Q
        return z - param['d']*phi - r2
    event.terminal = True
    event.direction = -1
    return event

def make_event_liftoff1(param):
    def event(t, Q):
        z, _, phi, _, r1, _, _, _ = Q
        return z + param['d']*phi - r1
    event.terminal = True
    event.direction = 1
    return event

def make_event_liftoff2(param):
    def event(t, Q):
        z, _, phi, _, _, _, r2, _ = Q
        return z - param['d']*phi - r2
    event.terminal = True
    event.direction = 1
    return event


###
# Simulation loop
while T[-1] < tfinal:
    if phase == 1:
        sol = solve_ivp(lambda t, Q: EOMStance(t, Q, param, 1), [tbegin, tfinal], Q0,
                        events=(make_event_liftoff1(param), make_event_touchdown2(param)),
                        t_eval=np.linspace(tbegin, tfinal, 25000))
        if sol.t_events[0].size > 0:
            phase = 4
        elif sol.t_events[1].size > 0:
            phase = 3
    elif phase == 2:
        sol = solve_ivp(lambda t, Q: EOMStance(t, Q, param, 2), [tbegin, tfinal], Q0,
                        events=(make_event_liftoff2(param), make_event_touchdown1(param)),
                        t_eval=np.linspace(tbegin, tfinal, 25000))
        if sol.t_events[0].size > 0:
            phase = 4
        elif sol.t_events[1].size > 0:
            phase = 3
    elif phase == 3:
        sol = solve_ivp(lambda t, Q: EOMStance(t, Q, param, 3), [tbegin, tfinal], Q0,
                        events=(make_event_liftoff1(param), make_event_liftoff2(param)),
                        t_eval=np.linspace(tbegin, tfinal, 25000))
        if sol.t_events[0].size > 0 and sol.t_events[1].size > 0:
            phase = 2 if sol.t_events[0][0] > sol.t_events[1][0] else 1
        elif sol.t_events[0].size > 0:
            phase = 2
        elif sol.t_events[1].size > 0:
            phase = 1
    else:  # phase == 4
        sol = solve_ivp(lambda t, Q: EOMFlight(t, Q, param), [tbegin, tfinal], Q0,
                        events=(make_event_touchdown1(param), make_event_touchdown2(param)),
                        t_eval=np.linspace(tbegin, tfinal, 25000))
        if sol.t_events[0].size > 0 and sol.t_events[1].size > 0:
            phase = 3 if abs(sol.t_events[0][0] - sol.t_events[1][0]) < 0.01 else \
                   1 if sol.t_events[0][0] < sol.t_events[1][0] else 2
        elif sol.t_events[0].size > 0:
            phase = 1
        elif sol.t_events[1].size > 0:
            phase = 2
    if len(sol.t) > 1:
        T.extend(sol.t[1:])
        Q.extend(sol.y.T[1:])
    Q0 = sol.y[:, -1]
    tbegin = T[-1]

# Converting to arrays
T = np.array(T)
Q = np.array(Q)

# Interpolating for smooth visualization
T2 = np.arange(0, tfinal, 0.01)
Q2 = np.array([
    np.interp(T2, T, Q[:, 0]),  # z
    np.interp(T2, T, Q[:, 1]),  # zdot
    np.interp(T2, T, Q[:, 2]),  # phi
    np.interp(T2, T, Q[:, 3]),  # phidot
    np.interp(T2, T, Q[:, 4]),  # r1
    np.interp(T2, T, Q[:, 5]),  # r1dot
    np.interp(T2, T, Q[:, 6]),  # r2
    np.interp(T2, T, Q[:, 7])   # r2dot
]).T


###
# Visualization with springs and thrust
def RunSlotHopperSimulation(T, Q, param, thrust_times_1, thrust_times_2, savefilename='none'):
    """Visualize slot hopper with springs, thrust, and multi-panel plots"""
    # Extract data
    z = Q[:, 0]
    zdot = Q[:, 1]
    phi = Q[:, 2]
    r1 = Q[:, 4]
    r2 = Q[:, 6]
    x1 = param['d'] * np.cos(phi)
    z1 = z + param['d'] * np.sin(phi)
    x2 = -param['d'] * np.cos(phi)
    z2 = z - param['d'] * np.sin(phi)
    
    # Calculating energy and phase variables
    energy = 0.5*param['m']*zdot**2 + param['m']*param['g']*z
    psi1, a1_vals = np.zeros(len(T)), np.zeros(len(T))
    psi2, a2_vals = np.zeros(len(T)), np.zeros(len(T))
    
    for i in range(len(T)):
        # Determining if legs are in contact
        leg1_contact = (z1[i] <= r1[i] + 0.001) and (Q[i,5] < 0)
        leg2_contact = (z2[i] <= r2[i] + 0.001) and (Q[i,7] < 0)
        
        psi1[i], a1_vals[i] = get_psi_a(z1[i], Q[i,1] + param['d']*Q[i,3], leg1_contact)
        psi2[i], a2_vals[i] = get_psi_a(z2[i], Q[i,1] - param['d']*Q[i,3], leg2_contact)
    

    # Colors for Sim Vis
    colorblue = [0, 0.447, 0.741]
    colorred = [0.85, 0.325, 0.098]
    coloryellow = [0.929, 0.694, 0.125]
    colorgreen = [0.466, 0.674, 0.188]
    colorthrust = [1, 0, 0]
    
    # Setting up figure
    fig = plt.figure('Slot Hopper with Springs', figsize=(16, 10), constrained_layout=True)
    plt.subplots_adjust(left=0.05, right=0.95, bottom=0.1, top=0.9)
    
    # Schematic motion subplot
    ax1 = plt.subplot2grid((3, 4), (0, 0), rowspan=2, colspan=2)
    ax1.set_aspect('equal')
    ax1.add_patch(Rectangle((-1, -0.05), 2, 0.05, facecolor=[0.7, 0.7, 0.7], edgecolor=[0.6, 0.6, 0.6], linewidth=2))
    
    # Drawing axes
    ax1.quiver(0, 0, 0.2, 0, color='k', linewidth=2)
    ax1.quiver(0, 0, 0, 0.2, color='k', linewidth=2)
    
    # Body line
    body_line = ax1.plot([x1[0], x2[0]], [z1[0], z2[0]], 'b-', lw=4)[0]
    
    # COM point
    com_point = ax1.plot(0, z[0], 'ro', ms=10, label='COM')[0]
    
    # Hip points
    hip1_point = ax1.plot(x1[0], z1[0], 'go', ms=8, label='Hip 1')[0]
    hip2_point = ax1.plot(x2[0], z2[0], 'mo', ms=8, label='Hip 2')[0]
    
    # Springs 
    spring_x = np.array([0, 0, 0.02, -0.02, 0.02, -0.02, 0.02, -0.02, 0.02, -0.02, 0, 0])
    spring1_y_initial = np.array([
        z1[0], z1[0]-0.3*min(r1[0], z1[0]), z1[0]-0.325*min(r1[0], z1[0]), 
        z1[0]-0.375*min(r1[0], z1[0]), z1[0]-0.425*min(r1[0], z1[0]), 
        z1[0]-0.475*min(r1[0], z1[0]), z1[0]-0.525*min(r1[0], z1[0]), 
        z1[0]-0.575*min(r1[0], z1[0]), z1[0]-0.625*min(r1[0], z1[0]), 
        z1[0]-0.675*min(r1[0], z1[0]), z1[0]-0.7*min(r1[0], z1[0]), 
        z1[0]-min(r1[0], z1[0])
    ])

    spring2_y_initial = np.array([
        z2[0], z2[0]-0.3*min(r2[0], z2[0]), z2[0]-0.325*min(r2[0], z2[0]), 
        z2[0]-0.375*min(r2[0], z2[0]), z2[0]-0.425*min(r2[0], z2[0]), 
        z2[0]-0.475*min(r2[0], z2[0]), z2[0]-0.525*min(r2[0], z2[0]), 
        z2[0]-0.575*min(r2[0], z2[0]), z2[0]-0.625*min(r2[0], z2[0]), 
        z2[0]-0.675*min(r2[0], z2[0]), z2[0]-0.7*min(r2[0], z2[0]), 
        z2[0]-min(r2[0], z2[0])
    ])
    
    spring1, = ax1.plot(x1[0]+spring_x, spring1_y_initial, '-', linewidth=2, color='g')
    spring2, = ax1.plot(x2[0]+spring_x, spring2_y_initial, '-', linewidth=2, color='m')
    
    # Feet
    foot1 = Ellipse((x1[0], max(z1[0]-r1[0], 0)), width=0.02, height=0.02, 
                   facecolor=colorgreen, edgecolor=np.array(colorgreen)*0.7)
    foot2 = Ellipse((x2[0], max(z2[0]-r2[0], 0)), width=0.02, height=0.02, 
                   facecolor=colorred, edgecolor=np.array(colorred)*0.7)
    ax1.add_patch(foot1)
    ax1.add_patch(foot2)
    
    # Thrust indicators
    thrust_flame1 = ax1.quiver(x1[0], max(z1[0]-r1[0], 0), 0, -0.05, color=colorthrust, 
                              scale=1, scale_units='xy', width=0.01, alpha=0)
    thrust_flame2 = ax1.quiver(x2[0], max(z2[0]-r2[0], 0), 0, -0.05, color=colorthrust, 
                              scale=1, scale_units='xy', width=0.01, alpha=0)
    
    # Vectors
    a1 = ax1.quiver(-0.5, 0, 0, z[0], color=colorred, scale=1, scale_units='xy', width=0.005)
    a2 = ax1.quiver(0, z[0], 0, zdot[0], color=coloryellow, scale=1, scale_units='xy', width=0.005)
    hline, = ax1.plot([-0.5, -0.6], [z[0], z[0]], ':', color=colorred, linewidth=2)
    
    # Text indicators
    thrust_text = ax1.text(0.02, 0.95, '', transform=ax1.transAxes, fontsize=10, 
                          bbox=dict(facecolor='white', alpha=0.7))
    phase1_text = ax1.text(0.02, 0.90, 'Leg 1', transform=ax1.transAxes, fontsize=10, color='green')
    phase2_text = ax1.text(0.02, 0.85, 'Leg 2', transform=ax1.transAxes, fontsize=10, color='red')
    

    ax1.set_title('Slot Hopper with Springs (Red flame = Thrust)')
    ax1.legend([a1, a2, hip1_point, hip2_point], ['COM Position', 'COM Velocity', 'Hip 1', 'Hip 2'], loc='best')
    ax1.text(0.15, 0.05, 'x')
    ax1.text(0.05, 0.15, 'y')
    ax1.set_xlim(-0.5, 0.5)
    ax1.set_ylim(-0.1, 0.6)
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('z (m)')
    
    # Position subplot
    ax2 = plt.subplot2grid((3, 4), (0, 2))
    p2, = ax2.plot(T[:1], z[:1], color=colorred, linewidth=2, label='COM')
    p2_hip1, = ax2.plot(T[:1], z1[:1], 'g--', label='Hip 1')
    p2_hip2, = ax2.plot(T[:1], z2[:1], 'm--', label='Hip 2')
    m2, = ax2.plot(T[0], z[0], 'o', color=colorred, markersize=8)
    ax2.set_title('Position vs Time')
    #ax2.set_xlim(0, tfinal)
    #ax2.set_ylim(-0.1, 0.6)
    ax2.set_xlim(0, T[-1])
    ax2.set_ylim(min(min(z), min(z1), min(z2)) - 0.05, max(max(z), max(z1), max(z2)) + 0.05)

    ax2.grid(True)
    ax2.legend()
    ax2.set_xlabel('time (sec)')
    ax2.set_ylabel('position (m)')
    
    # Velocity subplot
    ax3 = plt.subplot2grid((3, 4), (1, 2))
    p3, = ax3.plot(T[:1], zdot[:1], color=coloryellow, linewidth=2)
    m3, = ax3.plot(T[0], zdot[0], 'o', color=coloryellow, markersize=8)
    ax3.set_title('Velocity vs Time')
    ax3.set_xlim(0, tfinal)
    #ax3.set_ylim(-2, 2)
    ax3.set_ylim(min(zdot) - 0.5, max(zdot) + 0.5)
    ax3.grid(True)
    ax3.set_xlabel('time (sec)')
    ax3.set_ylabel('velocity (m/s)')
    
    # State-space subplot
    ax4 = plt.subplot2grid((3, 4), (0, 3))
    p4, = ax4.plot(z[:1], zdot[:1], color=colorgreen, linewidth=2)
    m4, = ax4.plot(z[0], zdot[0], 'o', color=colorgreen, markersize=8)
    ax4.set_title('State Space')
    #ax4.set_xlim(-0.1, 0.6)
    #ax4.set_ylim(-2, 2)
    ax4.set_xlim(min(z) - 0.05, max(z) + 0.05)
    ax4.set_ylim(min(zdot) - 0.5, max(zdot) + 0.5)
    ax4.grid(True)
    ax4.set_xlabel('position (m)')
    ax4.set_ylabel('velocity (m/s)')
    
    # Energy plot
    ax5 = plt.subplot2grid((3, 4), (1, 3))
    p5, = ax5.plot(T[:1], energy[:1], color='purple', linewidth=2)
    m5, = ax5.plot(T[0], energy[0], 'o', color='purple', markersize=8)
    ax5.set_title('Energy vs Time')
    #ax5.set_xlim(0, tfinal)
    ax5.set_xlim(0, T[-1])
    ax5.set_ylim(min(energy) - 0.1 * abs(min(energy)), max(energy) + 0.1 * abs(max(energy)))

    ax5.grid(True)
    ax5.set_xlabel('time (sec)')
    ax5.set_ylabel('energy (J)')
    
    # Phase variables plot
    ax6 = plt.subplot2grid((3, 4), (2, 0), colspan=2)
    p6a, = ax6.plot(T[:1], psi1[:1], 'g-', linewidth=2, label='Leg 1 phase')
    p6b, = ax6.plot(T[:1], psi2[:1], 'm-', linewidth=2, label='Leg 2 phase')
    ax6.set_title('Phase Variables vs Time')
    #ax6.set_xlim(0, tfinal)
    ax6.set_xlim(0, T[-1])
    ax6.set_ylim(min(min(psi1), min(psi2)) - 0.1, max(max(psi1), max(psi2)) + 0.1)

    ax6.grid(True)
    ax6.legend()
    ax6.set_xlabel('time (sec)')
    ax6.set_ylabel('phase (rad)')
    
    # Amplitude variables plot
    #ax7 = plt.subplot2grid((3, 4), (2, 2), colspan=2)
    #p7a, = ax7.plot(T[:1], a1_vals[:1], 'g-', linewidth=2, label='Leg 1 amplitude')
    #p7b, = ax7.plot(T[:1], a2_vals[:1], 'm-', linewidth=2, label='Leg 2 amplitude')
    #ax7.set_title('Amplitude Variables vs Time')
    #ax7.set_xlim(0, tfinal)
    #ax7.set_xlim(0, T[-1])
    #ax7.set_ylim(min(min(a1_vals), min(a2_vals)) - 0.1, max(max(a1_vals), max(a2_vals)) + 0.1)

    # Pitch angle plot
    ax7 = plt.subplot2grid((3, 4), (2, 2), colspan=2)
    p7, = ax7.plot(T[:1], phi[:1], 'b-', linewidth=2, label='Pitch Angle')
    m7, = ax7.plot(T[0], phi[0], 'o', color='blue', markersize=8)
    ax7.set_title('Pitch Angle vs Time')
    ax7.set_xlim(0, T[-1])
    ax7.set_ylim(min(phi) - 0.1, max(phi) + 0.1)
    ax7.grid(True)
    ax7.legend()
    ax7.set_xlabel('time (sec)')
    ax7.set_ylabel('pitch angle (rad)')

    ax7.grid(True)
    ax7.legend()
    ax7.set_xlabel('time (sec)')
    ax7.set_ylabel('amplitude')
    
    #plt.tight_layout()
    

###
    # Animation
    def animate(i):
        current_time = T[i]
        thrust1_active = any(start <= current_time <= end for start, end in thrust_times_1)
        thrust2_active = any(start <= current_time <= end for start, end in thrust_times_2)
        
        # Update body
        body_line.set_data([x1[i], x2[i]], [z1[i], z2[i]])
        
        # Update COM and hips
        com_point.set_data([0], [z[i]])
        hip1_point.set_data([x1[i]], [z1[i]])
        hip2_point.set_data([x2[i]], [z2[i]])
        
        # Determine contact states
        leg1_in_contact = (z1[i] <= r1[i] + 0.001) and (Q[i,5] < 0)
        leg2_in_contact = (z2[i] <= r2[i] + 0.001) and (Q[i,7] < 0)
        
        # Effective spring lengths, capped to prevent ground penetration
        eff_r1 = min(r1[i], max(z1[i], 0)) if leg1_in_contact else param['rho']
        eff_r2 = min(r2[i], max(z2[i], 0)) if leg2_in_contact else param['rho']
        
        # Spring visualization with all points clipped to >= 0
        spring1_y = np.array([
            max(z1[i], 0), 
            max(max(z1[i], 0)-0.3*eff_r1, 0), max(max(z1[i], 0)-0.325*eff_r1, 0), max(max(z1[i], 0)-0.375*eff_r1, 0),
            max(max(z1[i], 0)-0.425*eff_r1, 0), max(max(z1[i], 0)-0.475*eff_r1, 0), max(max(z1[i], 0)-0.525*eff_r1, 0),
            max(max(z1[i], 0)-0.575*eff_r1, 0), max(max(z1[i], 0)-0.625*eff_r1, 0), max(max(z1[i], 0)-0.675*eff_r1, 0),
            max(max(z1[i], 0)-0.7*eff_r1, 0), 
            max(max(z1[i], 0)-eff_r1, 0)
        ])
        spring2_y = np.array([
            max(z2[i], 0),
            max(max(z2[i], 0)-0.3*eff_r2, 0), max(max(z2[i], 0)-0.325*eff_r2, 0), max(max(z2[i], 0)-0.375*eff_r2, 0),
            max(max(z2[i], 0)-0.425*eff_r2, 0), max(max(z2[i], 0)-0.475*eff_r2, 0), max(max(z2[i], 0)-0.525*eff_r2, 0),
            max(max(z2[i], 0)-0.575*eff_r2, 0), max(max(z2[i], 0)-0.625*eff_r2, 0), max(max(z2[i], 0)-0.675*eff_r2, 0),
            max(max(z2[i], 0)-0.7*eff_r2, 0),
            max(max(z2[i], 0)-eff_r2, 0)
        ])
        
        spring1.set_data(x1[i]+spring_x, spring1_y)
        spring2.set_data(x2[i]+spring_x, spring2_y)
        
        # Foot positions
        foot1.center = (x1[i], max(z1[i]-eff_r1, 0))
        foot2.center = (x2[i], max(z2[i]-eff_r2, 0))
        
        # Thrust indicators
        if thrust1_active and leg1_in_contact:
            thrust_flame1.set_UVC(0, -0.1)
            thrust_flame1.set_offsets([[x1[i], max(z1[i]-eff_r1, 0)]])
            thrust_flame1.set_alpha(1)
            thrust_text.set_text('THRUST ACTIVE')
        else:
            thrust_flame1.set_alpha(0)
            
        if thrust2_active and leg2_in_contact:
            thrust_flame2.set_UVC(0, -0.1)
            thrust_flame2.set_offsets([[x2[i], max(z2[i]-eff_r2, 0)]])
            thrust_flame2.set_alpha(1)
            thrust_text.set_text('THRUST ACTIVE')
        else:
            thrust_flame2.set_alpha(0)
            
        if not (thrust1_active or thrust2_active):
            thrust_text.set_text('')
        
        # Update phase texts
        phase1_text.set_text('Leg 1: Stance' if leg1_in_contact else 'Leg 1: Flight')
        phase2_text.set_text('Leg 2: Stance' if leg2_in_contact else 'Leg 2: Flight')
        
        # Update vectors
        a1.set_UVC(0, z[i])
        a2.set_offsets([[0, z[i]]])
        a2.set_UVC(0, zdot[i])
        hline.set_ydata([z[i], z[i]])
        
        # Update plots
        p2.set_data(T[:i+1], z[:i+1])
        p2_hip1.set_data(T[:i+1], z1[:i+1])
        p2_hip2.set_data(T[:i+1], z2[:i+1])
        m2.set_data([T[i]], [z[i]])
        p3.set_data(T[:i+1], zdot[:i+1])
        m3.set_data([T[i]], [zdot[i]])
        p4.set_data(z[:i+1], zdot[:i+1])
        m4.set_data([z[i]], [zdot[i]])
        
        # Update new plots
        p5.set_data(T[:i+1], energy[:i+1])
        m5.set_data([T[i]], [energy[i]])
        p6a.set_data(T[:i+1], psi1[:i+1])
        p6b.set_data(T[:i+1], psi2[:i+1])
        #p7a.set_data(T[:i+1], a1_vals[:i+1])
        #p7b.set_data(T[:i+1], a2_vals[:i+1])
        p7.set_data(T[:i+1], phi[:i+1])
        m7.set_data([T[i]], [phi[i]])
        
        return (body_line, com_point, hip1_point, hip2_point, spring1, spring2, 
                foot1, foot2, thrust_flame1, thrust_flame2, thrust_text, 
                phase1_text, phase2_text, a1, a2, hline, p2, p2_hip1, p2_hip2, 
                m2, p3, m3, p4, m4, p5, m5, p6a, p6b, p7, m7)
    

    # Creating animation
    ani = FuncAnimation(fig, animate, frames=len(T), interval=20, blit=True)
    
    plt.show()
    

    ###
    # Saving GIF if requested
    if savefilename != 'none':
        ani.save(savefilename + '.gif', writer='pillow', fps=30)
    
    return ani


# ------------------------------
# EXPORT FUNCTIONS FOR OTHER FILES
# ------------------------------

# ------------------------------
# EXPORT FUNCTIONS FOR OTHER FILES
# ------------------------------

def getStates():

    raw_z   = Q2[:, 0]    # COM height from template simulation
    raw_phi = Q2[:, 2]    # body pitch

    # --------------------------
    # NEW: Template spring lengths (front and rear)
    # These indices are correct for your workingSlotHopper:
    # Q2[:,4] = r1 (front)
    # Q2[:,6] = r2 (rear)
    # --------------------------
    raw_r1  = Q2[:, 4]
    raw_r2  = Q2[:, 6]

    # --- Filtering ---
    N = len(raw_z)

    z   = np.zeros(N)
    phi = np.zeros(N)
    com = np.zeros(N)

    # r1 and r2 DO NOT need filtering (they are already smooth)
    r1 = raw_r1.copy()
    r2 = raw_r2.copy()

    for i in range(1, N):
        z[i]   = 0.95*z[i-1]   + 0.05*raw_z[i]
        phi[i] = 0.90*phi[i-1] + 0.10*raw_phi[i]
        com[i] = 0.95*com[i-1] + 0.05*raw_z[i]

    # --------------------------------------------
    # RETURN everything needed for anchoring
    # --------------------------------------------
    return {
        "z":   z,
        "phi": phi,
        "com": com,
        "r1":  r1,
        "r2":  r2
    }




def getTime():
    return 0.01          # timestep of Q2 interpolation

### 
# Running the simulation with visualization
#ani = RunSlotHopperSimulation(T2, Q2, param, thrust_times_1, thrust_times_2, 'SlotHopperWithSprings')