



class Person:
    def __init__(self, name, X_angle, center_XY, XYZ ):
        self.name = name
        self.X_angle_degrees = X_angle
        self.center_XY = center_XY
        self.real_XYZ = XYZ

    
    
    def __str__(self):
            return (f"Person(name={self.name}, X_angle={self.X_angle_degrees}, center_XY={self.center_XY}, real_XYZ={self.real_XYZ}")
