import unittest
from spherik.ConformalGeometricAlgebra import ConformalGeometricAlgebra
from clifford import *
from spherik.SpherikSolver import SpherikSolver
from spherik.PointChain import PointChain
from spherik.JointChain import JointChain
from spherik.Joint import Joint
import math
spherik_solver = SpherikSolver()
cga = ConformalGeometricAlgebra(1e-11)

class TestSpherikSolver(unittest.TestCase):

    def getRobot(self):
        joint_1 = Joint(math.pi, 100.0)
        joint_2 = Joint(math.pi, 100.0)
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
        robot = self.getRobot()
        target_position = cga.act(cga.e_origin, cga.translator(80.0 ^ cga.e2))
        spherik_solver = SpherikSolver()
        solutions = spherik_solver.solve(robot, target_position)
        point_0, point_1, point_2 = solutions[0]
        self.assertTrue(abs(cga.distance(cga.toVector(point_0), cga.toVector(point_1)) - robot.get(0).distance) < 1.0)
        self.assertTrue(abs(cga.distance(cga.toVector(point_1), cga.toVector(point_2)) - robot.get(1).distance) < 1.0)
        self.assertTrue(abs(cga.distance(cga.toVector(point_2), cga.toVector(target_position))) < 1.0)
