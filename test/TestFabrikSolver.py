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

class TestFabrikSolver(unittest.TestCase):

    def test_toRotors(self):
        points = [cga.point(0,0,0), cga.point(0,1,0), cga.point(0,2,0), cga.point(1,2,0)]
        point_chain = PointChain(points, cga)
        rotors = spherik_solver.toRotors(point_chain)
        expected_angles = [math.pi / 2.0, 0.0, -math.pi / 2.0]
        angles = [cga.angleFromRotor(rotor) for rotor in rotors]
        for (expected_angle, angle) in list(zip(expected_angles,angles)):
            self.assertTrue(abs(expected_angle - angle) < 1e-6)
