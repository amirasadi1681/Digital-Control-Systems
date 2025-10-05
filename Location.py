from typing import List
import numpy as np
import math
def cartesian2polar(cartesian):
        x = cartesian[0][0]
        y = cartesian[1][0]
        r = math.sqrt((x**2) + (y**2))
        if abs(y) < 1e-8:
            if x >0:
                theta = math.pi /2
            else:
                theta = -math.pi /2
        else:
            theta = math.atan2(x, y)
        theta = math.degrees(theta)
        # print("resut of cart2polar: ", (theta, r))
        # print("")
        return (theta, r)

def polar2cartesian(polar_in_degree:tuple):
        theta = polar_in_degree[0]
        r = polar_in_degree[1]
        result =  np.array([[r*math.sin(math.radians(theta))], [r*math.cos(math.radians(theta))]])
        # print("result in polar2crt", result)
        # print("")
        return result

class Location:
    def __init__(self, path_from_root:List[tuple]) -> None:
        """the given alphas should be in degree"""
        self.relative_path = path_from_root
        stack = path_from_root[::-1]
        self.absoluteLocation = np.array([[0], [0]]) # cartesian
        while len(stack) > 1:
            if len(path_from_root) < 2:
                break
            point0 = stack.pop(0) # gets the first and removes it
            point1 = stack.pop(0) # gets the second that now has become the first because of the removing in the prev line
            r0 = point0[1]
            r1 = point1[1]
            alpha0 = math.radians(point0[0])
            alpha1 = math.radians(point1[0])
            T = np.array([[math.cos(alpha1), math.sin(alpha1)], [-math.sin(alpha1), math.cos(alpha1)]])
            relative_location = np.array([[r0*math.sin(alpha0)], [r0*math.cos(alpha0)]])
            transfered_changed =  np.dot(T, relative_location) 
            result = np.array([[r1*math.sin(alpha1)], [r1*math.cos(alpha1)]])
            result = np.add(transfered_changed, result)
            stack.insert(0, cartesian2polar(result))
        
        if len(stack) >= 1:
            self.absoluteLocation = polar2cartesian(stack[0])
    
    def relative_path(self):
        return self.relative_path
    
    def distance(self, other_location):
        x2, y2 = other_location.cartesian_location()
        x1 = self.absoluteLocation[0][0]
        y1 = self.absoluteLocation[1][0]
        return math.sqrt(((x2 - x1)**2) + ((y2 - y1)**2))

    def cartesian_location(self):
        x = self.absoluteLocation[0][0]
        y = self.absoluteLocation[1][0]
        return (x, y)
    def dgpolar_location(self):
        return cartesian2polar(self.absoluteLocation)

    def __str__(self) -> str:
        return f"the relative_path is :{self.relative_path}\nThe absolute Location is:{(self.absoluteLocation[0][0], self.absoluteLocation[1][0])}"

##### Tests for Direction.py
# path1 = [(0, 1) , (90, 1) , (0, 1) ]#, (90, 1) , (0, 1) , (90, 1) , (0, 1) , (0, 1)]
# loc1 = Location(path1)
# print(loc1)
# path = [(-90, 1), (90, 1)]
# loc3 = Location(path)
# print(loc3)
# path = [(-90, 1), (-90, 1), (90, 1)]
# loc2 = Location(path)
# print(loc2)

