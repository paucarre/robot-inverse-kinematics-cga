import unittest
from clifford import *
import math

from spherik.ConformalGeometricAlgebra import ConformalGeometricAlgebra
from spherik.SpherikSolver import SpherikSolver
from spherik.Joint import Joint

spherik_solver = SpherikSolver()
cga = ConformalGeometricAlgebra(1e-11)

class TestSpherikSolver(unittest.TestCase):

    def getJoints(self, constraint_angle):
        joint_1 = Joint(constraint_angle, 100)
        joint_2 = Joint(constraint_angle, 100)
        return [joint_1, joint_2]

    def test_toRotors(self):
        points = [cga.point(0,0,0), cga.point(0,1,0), cga.point(0,2,0), cga.point(1,2,0)]
        point_chain = PointChain(points, cga)
        rotors = spherik_solver.toRotors(point_chain)
        expected_angles = [math.pi / 2.0, 0.0, -math.pi / 2.0]
        angles = [cga.angleFromRotor(rotor) for rotor in rotors]
        for (expected_angle, angle) in list(zip(expected_angles,angles)):
            self.assertTrue(abs(expected_angle - angle) < 1e-6)

    def test_solve_simple(self):
        joints = self.getJoints(math.pi)
        target_position = cga.act(cga.e_origin, cga.translator(80.0 ^ cga.e2))
        spherik_solver = SpherikSolver()
        solutions = spherik_solver.solve(joints, target_position)
        point_0, point_1, point_2 = solutions[0]
        self.assertTrue(abs(cga.distance(cga.toVector(point_0), cga.toVector(point_1)) - joints[0].distance) < 1.0)
        self.assertTrue(abs(cga.distance(cga.toVector(point_1), cga.toVector(point_2)) - joints[1].distance) < 1.0)
        self.assertTrue(abs(cga.distance(cga.toVector(point_2), cga.toVector(target_position))) < 1.0)

    def test_toRotors(self):
        joints = self.getJoints(math.pi)
        target_position = cga.act(cga.e_origin, cga.translator(80.0 ^ cga.e2))
        spherik_solver = SpherikSolver()
        list_of_points = spherik_solver.solve(joints, target_position)
        list_of_rotors = [spherik_solver.toRotors(points) for points in list_of_points]
        list_of_angles = [[cga.toDegrees(cga.angleFromRotor(rotor)) for rotor in rotors] for rotors in list_of_rotors]
        self.assertTrue(abs(list_of_angles[0][0] - 24) < 1.0)
        self.assertTrue(abs(list_of_angles[0][1] - 133) < 1.0)
        self.assertTrue(abs(list_of_angles[1][0] - 157) < 1.0)
        self.assertTrue(abs(list_of_angles[1][1] + 132) < 1.0)

    def test_anglesWithinConstraints(self):
        joints = self.getJoints((267.0 * 2.0 * math.pi) / (360.0))#  max 133 degrees
        target_position = cga.act(cga.e_origin, cga.translator(80.0 ^ cga.e2))
        spherik_solver = SpherikSolver()
        list_of_points = spherik_solver.solve(joints, target_position)
        angles_within_constraints = spherik_solver.anglesWithinConstraints(joints, list_of_points)
        self.assertEqual(len(angles_within_constraints), 1)

    def test_unreacheable(self):
        joints = self.getJoints(math.pi)
        # Unreacheable
        target_position = cga.act(cga.e_origin, cga.translator(200.1 ^ cga.e2))
        spherik_solver = SpherikSolver()
        list_of_points = spherik_solver.solve(joints, target_position)
        self.assertEqual(len(list_of_points), 0)
        # Reacheable
        target_position = cga.act(cga.e_origin, cga.translator(200.0 ^ cga.e2))
        spherik_solver = SpherikSolver()
        list_of_points = spherik_solver.solve(joints, target_position)
        self.assertEqual(len(list_of_points), 2)
