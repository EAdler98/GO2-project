
from go import *
import numpy as np
import math
import sys


def calculate_route(robot :SportModeTest, angle, x,y,z):
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
    robot.rotate(-beta1)

    
    # Calculate the gamma angle between PF and (1, 0)
    
    F_length = math.sqrt(xF**2 + yF**2)
    robot.goMeteresAhead(F_length/100)    
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
    print(f"beta2: {math.degrees(beta2)} degrees")
    robot.rotate(-beta2)
    
    # return gamma, math.degrees(beta1), CF_length, math.degrees(beta2)


    

if __name__ == "__main__":
    print("Start processing!")
    

    # Check if coordinates were provided
    if len(sys.argv) < 2:
        print("Error: No coordinates provided")
        sys.exit(1)

    # The coordinates string
    coordinates_str = sys.argv[1]
    
    my_robot = init_robot()
    
    
    # coordinates_str = "EVYATAR -16.92136929607713 -32.387353515625 130.20380488691146 168.88039459708455"
    print("coordinates_str: ", coordinates_str)

    # Split the string into its components
    coordinates_list = coordinates_str.split()
    print("coordinates_list: ", coordinates_list)

    # Extract the name and coordinates
    name = coordinates_list[0]
    print("name: ", name)
    x, y, z, alpha = map(float, coordinates_list[1:])
    print("x, y, z, alpha: ", x, y, z, alpha)
    print("type(x): ", type(x))
    # Print for debugging


    # Call the route calculation function
    calculate_route(my_robot, x, y, z, alpha)
    