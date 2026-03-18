# from go import *
import numpy as np
import math
import sys
import matplotlib.pyplot as plt
import numpy as np

def calculate_route(robot ,  x, y,angle):
    # x,y,angle = person.real_XYZ[0], person.real_XYZ[1], person.X_angle_degrees
    
    xP = x
    yP = y
    D = 100
    
    # Step 1: Find point F
    alpha = math.radians(angle)
    CP_length = math.sqrt(xP**2 + yP**2)  # Length of vector CP
    PF_x = xP + (D * math.sin(alpha) )
    PF_y = yP - (D * math.cos(alpha) )
    
    xF =PF_x
    yF = PF_y
    print(f"F(xF, yF) = ({xF}, {yF})")
    sign = -1
    if xF < 0:
        sign *= -1
    
    # Step 2: Calculate vector CF
    CF_x = xF
    CF_y = yF
    CF_length = math.sqrt(CF_x**2 + CF_y**2)
    
    # Step 34: Calculate the angle beta1 between vector CF and vector CP
    beta1 = -1 * sign * math.acos((CF_y * 1) / (CF_length))
    
    print(f"beta1: {math.degrees(beta1)} degrees")
    #robot.rotate(-beta1)

    
    # Calculate the gamma angle between PF and (1, 0)
    
    F_length = math.sqrt(xF**2 + yF**2)
    #robot.goMeteresAhead(F_length/100)    
    # Step 7: Calculate the angle beta2 between vector CF and vector FP
    FP_x = xP - xF
    FP_y = yP - yF
    FP_length = math.sqrt(FP_x**2 + FP_y**2)
    X = (CF_x * FP_x + CF_y * FP_y) / (CF_length * FP_length)
    if X >= 1:
        X = 1
    elif X <= -1:
        X = -1
    if angle<0:
        sign*=-1
    if xF <0:
        sign *=-1
    beta2 = sign * math.acos(X)
    beta2=(alpha+beta1)*(-1)
    
    print(f"beta2: {math.degrees(beta2)} degrees")
    #robot.rotate(-beta2)
    # robot.rotate(-beta2)
    
    # Return values for plotting:
    # robot's starting position (assumed at (0,0)),
    # approach point F, person's position P,
    # the two computed rotation angles (beta1 and beta2) and the face angle (in radians)
    return (0, 0), (xF, yF), (xP, yP), beta1, beta2, math.radians(angle)



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
    #beta2= math.radians(-45)
    # Final orientation after second rotation (beta2)
    theta2 = theta1 - beta2
    print(f"theta2: {math.degrees(theta2)} degrees")
    

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
             arrow_length * math.cos(face_angle_rad-math.pi/2),
             arrow_length * math.sin(face_angle_rad-math.pi/2),
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
    # Wait for a key press to close the plot
    print("Press any key to close the plot...")
    plt.waitforbuttonpress()

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
    x, y, z,alpha = map(float, coordinates_list[1:])
    # alpha+=180
    print("x, y, z, alpha:", x, y, z, alpha)
    
    
    # my_robot = init_robot()
    my_robot = None
    
    # Call the route calculation function with the original parameter order:
    # (robot, angle, x, y, z)
    robot_pos, F, P, beta1, beta2, face_angle_rad = calculate_route(my_robot, x, z,alpha)
    
    # Output a top-down illustration of the scene
    plot_trajectory(robot_pos, F, P, beta1, beta2, face_angle_rad)
