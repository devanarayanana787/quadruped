import math

# ------------------- ROBOT LEG DIMENSIONS -------------------
Coxa = 27.5   # length of coxa link
Femur = 55.0  # length of femur link
Tibia = 77.5  # length of tibia link


def inverse_kinematics(x, y, z):
    """
    Inverse Kinematics for a single leg of quadruped.
    Input:  x, y, z (foot position in 3D space)
    Output: alpha (femur angle), beta (tibia angle), gamma (coxa angle)
    """

    # Step 1: Horizontal distance from body to foot
    L1 = math.sqrt(x**2 + y**2)

    # Step 2: Coxa angle (rotation in XY plane)
    gamma = math.degrees(math.atan2(y, x))

    # Step 3: Distance from femur joint to foot
    L = math.sqrt(z**2 + (L1 - Coxa)**2)

    # Step 4: Angle between femur and vertical line
    alpha1 = math.degrees(math.acos(z / L))

    # Step 5: Angle using cosine rule for femur
    alpha2 = math.degrees(
        math.acos((L**2 + Femur**2 - Tibia**2) / (2 * L * Femur))
    )

    # Step 6: Final femur angle
    alpha = alpha1 + alpha2

    # Step 7: Tibia angle using cosine rule
    beta = math.degrees(
        math.acos((Femur**2 + Tibia**2 - L**2) / (2 * Femur * Tibia))
    )

    return alpha, beta, gamma


# ------------------- TEST -------------------
if __name__ == "__main__":
    # Example foot position (change to test)
    x, y, z = 50.0, 30.0, -60.0

    alpha, beta, gamma = inverse_kinematics(x, y, z)

    print("Inverse Kinematics Results:")
    print(f"Alpha (Femur joint): {alpha:.2f}°")
    print(f"Beta (Tibia joint): {beta:.2f}°")
    print(f"Gamma (Coxa joint): {gamma:.2f}°")
