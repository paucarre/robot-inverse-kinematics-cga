import unittest
from clifford import *
import math

from spherik.ConformalGeometricAlgebra import ConformalGeometricAlgebra
from spherik.SpherikSolver import SpherikSolver
from spherik.JointChain import JointChain
from spherik.Joint import Joint

spherik_solver = SpherikSolver()
cga = ConformalGeometricAlgebra(1e-11)

class TestSpherikSolver(unittest.TestCase):

    def getRobot(self, constraint_angle):
        joint_1 = Joint(constraint_angle, 100)
        joint_2 = Joint(constraint_angle, 100)
        joint_chain = JointChain([joint_1, joint_2])
        return joint_chain

    def test_toRotors(self):
        points = [cga.point(0,0,0), cga.point(0,1,0), cga.point(0,2,0), cga.point(1,2,0)]
        point_chain = PointChain(points, cga)
        rotors = spherik_solver.toRotors(point_chain)
        expected_angles = [math.pi / 2.0, 0.0, -math.pi / 2.0]
        angles = [cga.angleFromRotor(rotor) for rotor in rotors]
        for (expected_angle, angle) in list(zip(expected_angles,angles)):
            self.assertTrue(abs(expected_angle - angle) < 1e-6)

    def test_solve_simple(self):
        robot = self.getRobot(math.pi)
        target_position = cga.act(cga.e_origin, cga.translator(80.0 ^ cga.e2))
        spherik_solver = SpherikSolver()
        solutions = spherik_solver.solve(robot, target_position)
        point_0, point_1, point_2 = solutions[0]
        self.assertTrue(abs(cga.distance(cga.toVector(point_0), cga.toVector(point_1)) - robot.get(0).distance) < 1.0)
        self.assertTrue(abs(cga.distance(cga.toVector(point_1), cga.toVector(point_2)) - robot.get(1).distance) < 1.0)
        self.assertTrue(abs(cga.distance(cga.toVector(point_2), cga.toVector(target_position))) < 1.0)

    def test_toRotors(self):
        robot = self.getRobot(math.pi)
        target_position = cga.act(cga.e_origin, cga.translator(80.0 ^ cga.e2))
        spherik_solver = SpherikSolver()
        list_of_points = spherik_solver.solve(robot, target_position)
        list_of_rotors = [spherik_solver.toRotors(points) for points in list_of_points]
        list_of_angles = [[cga.toDegrees(cga.angleFromRotor(rotor)) for rotor in rotors] for rotors in list_of_rotors]
        first_solution = list_of_angles[0]
        self.assertTrue(abs(list_of_angles[0][0] - 24) < 1.0)
        self.assertTrue(abs(list_of_angles[0][1] - 133) < 1.0)
        self.assertTrue(abs(list_of_angles[1][0] - 157) < 1.0)
        self.assertTrue(abs(list_of_angles[1][1] + 132) < 1.0)
