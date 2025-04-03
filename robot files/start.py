# from go import *
import numpy as np
import math
import sys
import matplotlib.pyplot as plt
import numpy as np

def calculate_route(robot , angle, x, y, z):
    # x,y,angle = person.real_XYZ[0], person.real_XYZ[1], person.X_angle_degrees
    xP = x
    yP = y
    D = 100
    
    # Step 1: Calculate the PF vector
    alpha = math.radians(angle)
    CP_length = math.sqrt(xP**2 + yP**2)  # Length of vector CP
    PF_x = (xP * math.cos(alpha) - yP * math.sin(alpha)) * (D / CP_length)
    PF_y = (xP * math.sin(alpha) + yP * math.cos(alpha)) * (D / CP_length)
    
    # Step 2: Find point F
    xF = xP - PF_x
    yF = yP - PF_y
    print(f"F(xF, yF) = ({xF}, {yF})")
    sign = -1
    if xF < 0:
        sign *= -1
    
    # Step 3: Calculate vector CF
    CF_x = xF
    CF_y = yF
    CF_length = math.sqrt(CF_x**2 + CF_y**2)
    
    # Step 4: Calculate the angle beta1 between vector CF and vector CP
    beta1 = -1 * sign * math.acos((CF_y * 1) / (CF_length))
    
    print(f"beta1: {math.degrees(beta1)} degrees")
    # robot.rotate(-beta1)

    # Calculate the gamma angle between PF and (1, 0)
    
    F_length = math.sqrt(xF**2 + yF**2)
    # robot.goMeteresAhead(F_length/100)    
    
    # Step 7: Calculate the angle beta2 between vector CF and vector FP
    FP_x = xP - xF
    FP_y = yP - yF
    FP_length = math.sqrt(FP_x**2 + FP_y**2)
    X = (CF_x * FP_x + CF_y * FP_y) / (CF_length * FP_length)
    if X >= 1:
        X = 1
    elif X <= -1:
        X = -1
    if angle < 0:
        sign *= -1
    if xF < 0:
        sign *= -1
    beta2 = sign * math.acos(X)
    print(f"beta2: {math.degrees(beta2)} degrees")
    # robot.rotate(-beta2)
    
    # Return values for plotting:
    # robot's starting position (assumed at (0,0)),
    # approach point F, person's position P,
    # the two computed rotation angles (beta1 and beta2) and the face angle (in radians)
    return (0, 0), (xF, yF), (xP, yP), beta1, -beta2, math.radians(angle)

def plot_trajectory(robot_pos, F, P, beta1, beta2, face_angle_rad):
    """
    Generates a top-down illustration showing:
      - The robot's starting position (assumed at (0,0))
      - The approach point F
      - The person's position P with an arrow indicating face direction
      - The trajectory (dashed line) from the robot to F
      - Two orientation arrows:
          • At the starting point: orientation after the first rotation (theta1)
          • At F: final orientation after the second rotation (theta2)
    """
    plt.figure(figsize=(8, 10))
    ax = plt.gca()
    
    # Assume initial orientation is upward (90° in radians)
    init_orientation = math.radians(90)
    # Orientation after first rotation (beta1)
    theta1 = init_orientation - beta1
    # Final orientation after second rotation (beta2)
    theta2 = theta1 - beta2

    arrow_length = 30  # cm (for illustration)

    # Plot robot starting position (blue square)
    ax.plot(robot_pos[0], robot_pos[1], 'bs', markersize=12, label='Robot Start (Camera)')
    # Draw an arrow at the starting position showing orientation after beta1 rotation (theta1)
    ax.arrow(robot_pos[0], robot_pos[1],
             arrow_length * math.cos(theta1),
             arrow_length * math.sin(theta1),
             head_width=5, head_length=5, fc='magenta', ec='magenta', label='Orientation after β₁')

    # Plot approach point F (red circle)
    ax.plot(F[0], F[1], 'ro', markersize=8, label='Approach Point F')
    
    # Plot trajectory from starting position to F (dashed line)
    trajectory = np.array([robot_pos, F])
    ax.plot(trajectory[:, 0], trajectory[:, 1], 'k--', label='Trajectory to F')
    
    # At F, draw an arrow representing the final orientation after beta2 (theta2)
    ax.arrow(F[0], F[1],
             arrow_length * math.cos(theta2),
             arrow_length * math.sin(theta2),
             head_width=5, head_length=5, fc='orange', ec='orange', label='Final Orientation after β₂')
    
    # Plot the person's position P (green circle)
    ax.plot(P[0], P[1], 'go', markersize=8, label='Person')
    # Draw an arrow at P showing the person's facing direction (using face_angle_rad)
    ax.arrow(P[0], P[1],
             arrow_length * math.cos(face_angle_rad),
             arrow_length * math.sin(face_angle_rad),
             head_width=5, head_length=5, fc='green', ec='green', label='Person Facing')
    
    # Annotate points for clarity
    ax.text(robot_pos[0] - 10, robot_pos[1] - 10, "Robot Start", color='blue')
    ax.text(F[0] + 5, F[1] + 5, "F", color='red')
    ax.text(P[0] + 5, P[1] + 5, "Person", color='green')
    
    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.set_title("Top-Down View: Trajectory and Orientation")
    ax.legend()
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')
    
    plt.show()

if __name__ == "__main__":
    print("Start processing!")
    
    # Check if coordinates were provided
    if len(sys.argv) < 2:
        print("Error: No coordinates provided")
        sys.exit(1)
    
    # The coordinates string, for example:
    # "EVYATAR -16.92 -32.39 130.20 168.88"
    coordinates_str = sys.argv[1]
    print("coordinates_str:", coordinates_str)
    
    # Split the string into its components
    coordinates_list = coordinates_str.split()
    print("coordinates_list:", coordinates_list)
    
    # Extract the name and coordinates
    name = coordinates_list[0]
    print("name:", name)
    x, y, z, alpha = map(float, coordinates_list[1:])
    print("x, y, z, alpha:", x, y, z, alpha)
    
    # my_robot = init_robot()
    my_robot = None
    
    # Call the route calculation function with the original parameter order:
    # (robot, angle, x, y, z)
    robot_pos, F, P, beta1, beta2, face_angle_rad = calculate_route(my_robot, x, y, z, alpha)
    
    # Output a top-down illustration of the scene
    plot_trajectory(robot_pos, F, P, beta1, beta2, face_angle_rad)
