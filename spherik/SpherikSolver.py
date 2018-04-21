from spherik.ConformalGeometricAlgebra import ConformalGeometricAlgebra
from spherik.Joint import Joint
import math

#TODO: for now assume 2DoF planar robot and keep improving
class SpherikSolver(object):

    def __init__(self):
        self.cga = ConformalGeometricAlgebra()
        self.resolution = 1e-10

    def error(self, target, point_chain):
        return math.sqrt(abs(target | point_chain.get(0, True))) + math.sqrt(abs(self.cga.e_origin | point_chain.get(0, False)))

    def toRotors(self, point_chain, initial_direction=[1.0, 0.0, 0.0]):
        previous_direction = self.cga.vector(*initial_direction)
        previous_position = None
        rotors = []
        for current_position in point_chain:
            if previous_position is None:
                previous_position = current_position
            else:
                current_direction = self.cga.direction(previous_position, current_position)
                rotor = self.cga.toRotor(previous_direction, current_direction)
                rotors.insert(len(rotors), rotor)
                previous_direction = current_direction
                previous_position = current_position
        return rotors

    def withinConstraints(self, angles, joints):
        angles_within_constraints =[angle for angle, joint in zip(angles, joints) \
            if angle >= - joint.angle_constraint / 2.0 and angle <= joint.angle_constraint / 2.0 ]
        return len(angles_within_constraints) == len(angles)

    def anglesWithinConstraints(self, joints, list_of_points):
        list_of_rotors = [self.toRotors(points) for points in list_of_points]
        list_of_angles = [[self.cga.angleFromRotor(rotor) for rotor in rotors] for rotors in list_of_rotors]
        list_of_angles_with_joints = [list(zip(angles, joints)) for angles in list_of_angles if self.withinConstraints(angles, joints) ]
        return list_of_angles_with_joints

    def solve(self, joint_chain,  target_position):
        point_0 = self.cga.e_origin
        point_2 = target_position
        p_prime = self.cga.act(self.cga.e_origin, self.cga.translator(self.cga.e1))
        rotation_plane = self.cga.plane(point_0, point_2, p_prime)
        sphere_center_p_prime_edge_p1 = self.cga.sphere(p_prime, math.sqrt((joint_chain[0].distance * joint_chain[0].distance) + 1))
        sphere_center_p2_edge_p1 = self.cga.sphere(point_2, joint_chain[1].distance)
        point_pair_1 = sphere_center_p_prime_edge_p1.\
            meet(sphere_center_p2_edge_p1).\
            meet(rotation_plane)
        point_1_first, point_1_second = self.cga.project(point_pair_1)
        return [[point_0, point_1_first, point_2], [point_0, point_1_second, point_2]]
