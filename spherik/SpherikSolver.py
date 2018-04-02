from spherik.ConformalGeometricAlgebra import ConformalGeometricAlgebra
from spherik.PointChain import PointChain
import math

class SpherikSolver(object):

    def __init__(self):
        self.cga = ConformalGeometricAlgebra()
        self.resolution = 1e-10

    def error(self, target, point_chain):
        return math.sqrt(abs(target | point_chain.get(0, True))) + math.sqrt(abs(self.cga.e_origin | point_chain.get(0, False)))

    def toRotors(self, point_chain):
        previous_direction = self.cga.vector(1.0, 0.0, 0.0)
        previous_position = None
        rotors = []
        for current_position in point_chain.positions:
            if previous_position is None:
                previous_position = current_position
            else:
                current_direction = self.cga.direction(previous_position, current_position)
                rotor = self.cga.toRotor(previous_direction, current_direction)
                rotors.insert(len(rotors), rotor)
                previous_direction = current_direction
                previous_position = current_position
        return rotors

    def solve(self, joint_chain, target_position, max_iterations=100):
        
        return point_chain
