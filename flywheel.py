import math

# ============================================================
# Constants
# ============================================================
g = 9.8  # gravitational acceleration (m/s²)

# ============================================================
# User inputs for projectile motion
# ============================================================
distance = float(input("Horizontal range to the hoop (m): "))
angle_deg = float(input("Launch angle of the ball (degrees): "))
launch_height = float(input("Launch height above ground (m) [default 0]: ") or 0)
target_height = float(input("Target hoop height above ground (m) [default 0]: ") or 0)

# Convert angle to radians
angle_rad = math.radians(angle_deg)

# ============================================================
# Calculate required ball exit velocity V_b
# Projectile motion with different start and end heights:
# y = y0 + v0*sinθ * t - 0.5*g*t²
# x = v0*cosθ * t
# Eliminate t: y_target - y0 = tanθ * x - (g*x²)/(2 v0² cos²θ)
# Solve for v0:
# v0² = (g * x²) / (2 cos²θ * (tanθ * x - (y_target - y0)))
# ============================================================
delta_y = target_height - launch_height
cos_theta = math.cos(angle_rad)
tan_theta = math.tan(angle_rad)

denom = 2 * cos_theta**2 * (tan_theta * distance - delta_y)
if denom <= 0:
    print("Error: Invalid trajectory (ball cannot reach that height/distance with given angle).")
    print("Check that tanθ * distance > Δy.")
else:
    V_b = math.sqrt((g * distance**2) / denom)
    print(f"\nRequired ball exit velocity: {V_b:.3f} m/s")

    # ============================================================
    # Wheel parameters
    # ============================================================
    m_w = float(input("\nMass of the wheel (kg): "))
    R_w = float(input("Radius of the wheel (m): "))
    m_b = float(input("Mass of the ball (kg): "))

    # Moment of inertia of wheel (solid aluminum disk)
    I_w = 0.5 * m_w * R_w**2

    # Derived equation from conservation of angular momentum
    # (See image: I_w ω_wi / R_w = I_w ω_wf / R_w + m_b V_b + (2/5) m_b V_b)
    # With ω_wf = 2 V_b / R_w and I_b = (2/5) m_b R_b^2 (solid sphere)
    # After algebra: ω_wi = (2 V_b / R_w) * (1 + (7/5)*(m_b / m_w))
    omega_wi = (2 * V_b / R_w) * (1 + (7/5) * (m_b / m_w))
    RPM = omega_wi * 60 / (2 * math.pi)

    print("\n--- Wheel angular velocity (from no‑slip inelastic collision) ---")
    print(f"Initial angular velocity of the wheel: ω_wi = {omega_wi:.3f} rad/s")
    print(f"Equivalent rim linear speed: {omega_wi * R_w:.3f} m/s")
    print(f"RPM: {RPM:.3f}")
    print("\nNote: Assumes static friction sufficient to prevent slip (grooves added if needed).")