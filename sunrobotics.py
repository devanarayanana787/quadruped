import math

def cartesian_to_polar(x, y, z):
    # Robot leg dimensions (in mm)
    coxa = 27.5   # length of coxa
    femur = 55.0  # length of femur
    tibia = 77.5 # length of tibia

    # Step 1: Project foot position onto X-Y plane
    w = (1 if x >= 0 else -1) * math.sqrt(x**2 + y**2)

    # Effective horizontal distance for femur+tibia
    v = w - coxa

    # Step 2: Femur angle (alpha)
    femur_angle = math.atan2(z, v) + math.acos(
        (femur**2 - tibia**2 + v**2 + z**2) / (2 * femur * math.sqrt(v**2 + z**2))
    )

    # Step 3: Tibia angle (beta)
    tibia_angle = math.acos(
        (femur**2 + tibia**2 - v**2 - z**2) / (2 * femur * tibia)
    )

    # Step 4: Coxa angle (gamma)
    coxa_angle = math.atan2(y, x) if w >= 0 else math.atan2(-y, -x)

    # Convert radians → degrees
    femur_angle = math.degrees(femur_angle)
    tibia_angle = math.degrees(tibia_angle)
    coxa_angle = math.degrees(coxa_angle)

    return femur_angle, tibia_angle, coxa_angle


# Example usage
x, y, z = 50, 30, -60   # target foot coordinates
femur, tibia, coxa = cartesian_to_polar(x, y, z)

print(f"Target: ({x}, {y}, {z})")
print(f"Femur angle: {femur:.2f}°")
print(f"Tibia angle: {tibia:.2f}°")
print(f"Coxa angle: {coxa:.2f}°")
