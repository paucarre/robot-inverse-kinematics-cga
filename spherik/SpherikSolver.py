from spherik.ConformalGeometricAlgebra import ConformalGeometricAlgebra
from spherik.PointChain import PointChain
from spherik.JointChain import JointChain
from spherik.Joint import Joint
import math

#TODO: for now assume 2DoF planar robot and keep improving
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


    def solve(self, joint_chain,  target_position):
        point_0 = self.cga.e_origin
        point_2 = target_position
        p_prime = self.cga.act(self.cga.e_origin, self.cga.translator(self.cga.e1))
        point_pair_1 = self.cga.sphere(p_prime, math.sqrt((joint_chain.get(0).distance * joint_chain.get(0).distance) + 1)). \
            meet(self.cga.sphere(point_2, joint_chain.get(1).distance)). \
            meet(self.cga.plane(point_0, point_2, p_prime))
        point_1_first, point_1_second = self.cga.project(point_pair_1)
        return [point_0, point_1_first, point_2], [point_0, point_1_second, point_2]
