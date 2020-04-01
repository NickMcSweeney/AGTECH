
# mp-test.py -- tests for the mindprobe running under the BTB

from mpclient import *
import sys
import unittest






class TestParameters(unittest.TestCase):
    def setUp(self):
        btb.clear_world()
        btb.set_default_params()
        
    def testSimpleParams(self):
        """Test that simple parameters can be set to non-default values."""
        simple_params = ("robot_crash_radius",
                         "robot_wheelbase",
                         "robot_wheel_diameter",
                         "robot_wheel_clicks",
                         "robot_max_acceleration",
                         "robot_motor_command_max",
                         "max_run_time",
                         "ticks_per_sec",
                         "field_radius",
                         "pot_radius")

        for param in simple_params:
            exec "x0 = get_%s()" % param
            self.assert_(x0 > 0)
            exec "set_%s(x0 + 1)" % param
            exec "x1 = get_%s()" % param
            self.assertEqual(x1, x0 + 1)


    def testBoundaryParams(self):
        """Test that the boundary can be set."""
        (x0, y0, x1, y1) = get_boundary_location()
        set_boundary_location(x0 + 2, y0 - 2, x1 + 3, y1 - 4)
        (x0b, y0b, x1b, y1b) = get_boundary_location()

        self.assertEqual(x0b, x0 + 2)
        self.assertEqual(y0b, y0 - 2)
        self.assertEqual(x1b, x1 + 3)
        self.assertEqual(y1b, y1 - 4)

class TestRobotMotion(unittest.TestCase):
    
    def setUp(self):
        btb.clear_world()
        btb.set_default_params()

        self.x0 = 1
        self.y0 = 1
        self.h0 = deg2rad(45)
        self.r = create_robot(x = self.x0, y = self.y0, heading = self.h0)

    def testRobotStopped(self):
        """Test that a robot stopped does not move."""
        send_magic_message(self.r, "stop")
        run_world(seconds = 1)
        (x, y, heading) = get_robot_position(self.r)
        self.assertEqual(x, self.x0)
        self.assertEqual(y, self.y0)
        self.assertEqual(heading, deg2rad(45))
    
    def testRobotForward(self):
        """Test that a robot moves forward"""
        send_magic_message(self.r, "forward")
        send_magic_message(self.r, "slow")
        run_world(seconds = 1)
        (x, y, heading) = get_robot_position(self.r)
        self.assertEqual(x, y)
        self.assert_(x > self.x0)
        self.assertEqual(heading, self.h0)

    def testRobotTurnLeft(self):
        """Test that a robot turns left"""
        send_magic_message(self.r, "left")
        send_magic_message(self.r, "slow")
        run_world(seconds = 1)
        (x, y, heading) = get_robot_position(self.r)
        self.assert_(heading > self.h0)
        self.assert_(x > self.x0)
        self.assert_(y > self.y0)

    def testRobotTurnRight(self):
        """Test that a robot turns left"""
        send_magic_message(self.r, "right")
        send_magic_message(self.r, "slow")
        run_world(seconds = 1)
        (x, y, heading) = get_robot_position(self.r)
        self.assert_(heading < self.h0)
        self.assert_(x > self.x0)
        self.assert_(y > self.y0)


class TestRobotErrors(unittest.TestCase):

    def setUp(self):
        btb.clear_world()
        btb.set_default_params()

    def testRobotRobotCrash(self):
        """Test that robots crashing generates an exception"""

        r0 = create_robot(x = 0, y = 0, heading = 0)
        r1 = create_robot(x = 2, y = 0, heading = 0)

        send_magic_message(r0, "forward")
        send_magic_message(r0, "slow")
        send_magic_message(r1, "reverse")
        send_magic_message(r1, "fast")

        try:
            run_world()  # kablam!
        except RuntimeError, (message,):
            self.assertEqual(message, "robot-robot crash detected")

        (x0, y0, h0) = get_robot_position(r0)
        (x1, y1, h1) = get_robot_position(r1)

        self.assertEqual(h0, 0)
        self.assertEqual(h1, 0)

        radius = get_robot_crash_radius()

        self.assert_(distance(x0, y0, x1, y1) <= 2 * radius)

    def testRobotAwol(self):
        """Test that BTB detects robot wandering off"""

        field_radius = 3

        r0 = create_robot(x = 0, y = 0, heading = 0)

        set_field_radius(field_radius)

        send_magic_message(r0, "forward")
        send_magic_message(r0, "fast")

        try:
            run_world()
        except RuntimeError, (message,):
            self.assertEqual(message, "robot wandered off")

        (x0, y0, h0) = get_robot_position(r0)

        self.assertEqual(h0, 0)

        self.assert_(distance(0, 0, x0, y0) >= field_radius)


if __name__ == '__main__':
    unittest.main()
