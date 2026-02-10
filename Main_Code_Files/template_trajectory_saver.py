import numpy as np
from workingSlotHopper import getStates, getTime

def simulate_slot_hopper():
    """
    CORRECTED: Extract proper state variables with CORRECT leg lengths
    """
    states = getStates()
    dt = getTime()

    # Get raw states
    z_com = np.array(states["z"])      # Body COM
    phi = np.array(states["phi"])      # Body angle
    r1_raw = np.array(states["r1"])    # Front leg length (compressed during stance)
    r2_raw = np.array(states["r2"])    # Rear leg length (compressed during stance)
    
    # CRITICAL: Compute actual hip heights
    d = 0.125  # half hip-to-hip distance from your params
    z1 = z_com + d * np.sin(phi)  # Front hip height
    z2 = z_com - d * np.sin(phi)  # Rear hip height

    # CORRECTED LEG LENGTHS: Use nominal length during stance, raw during flight
    rho = 0.175  # nominal leg length from slot hopper params
    
    # Determine contact states (hip height ≈ compressed leg length means contact)
    CONTACT_TOL = 0.001
    contact1 = np.abs(z1 - r1_raw) < CONTACT_TOL
    contact2 = np.abs(z2 - r2_raw) < CONTACT_TOL
    
    # Use nominal length during stance, actual length during flight
    r1 = np.where(contact1, rho, r1_raw)
    r2 = np.where(contact2, rho, r2_raw)

    return {
        "z_com": z_com,  # Body COM
        "phi": phi,
        "z1": z1,        # Front hip height
        "z2": z2,        # Rear hip height
        "r1": r1,        # CORRECTED: Nominal length during stance
        "r2": r2,        # CORRECTED: Nominal length during stance
        "r1_raw": r1_raw, # For debugging
        "r2_raw": r2_raw, # For debugging
        "contact1": contact1.astype(int), # For debugging
        "contact2": contact2.astype(int), # For debugging
        "dt": dt
    }

if __name__ == "__main__":
    data = simulate_slot_hopper()
    np.save("template_trajectory.npy", data)
    print("✓ Saved template_trajectory.npy with CORRECTED leg lengths:")
    print(f"  Keys: {list(data.keys())}")
    print(f"  Frames: {len(data['z_com'])}")
    
    # Debug sample frames
    for i in [100, 200, 300]:
        print(f"\nSample frame {i}:")
        print(f"  z1 = {data['z1'][i]:.4f}m, r1_raw = {data['r1_raw'][i]:.4f}m, r1_corrected = {data['r1'][i]:.4f}m")
        print(f"  z2 = {data['z2'][i]:.4f}m, r2_raw = {data['r2_raw'][i]:.4f}m, r2_corrected = {data['r2'][i]:.4f}m")
        print(f"  Contact1: {bool(data['contact1'][i])}, Contact2: {bool(data['contact2'][i])}")