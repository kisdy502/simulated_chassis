#!/usr/bin/env python3
"""Expand installed models and verify control/semantic contracts without Gazebo."""
import tempfile
import os
import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml
from simulated_chassis_ur5e.configuration import (
    robot_description, moveit_parameters, merged_controllers, manager_parameters, gripper_config)


class ToolModels(unittest.TestCase):
    def model(self, tool):
        return ET.fromstring(robot_description(tool_type=tool)['robot_description'])

    def test_exclusive_tools(self):
        for tool in ('gripper', 'camera', 'none'):
            model = self.model(tool)
            links = {link.get('name') for link in model.findall('link')}
            self.assertEqual('ur5e_gripper_base' in links, tool == 'gripper')
            self.assertEqual('ur5e_camera_link' in links, tool == 'camera')
            for joint in model.findall('joint'):
                self.assertIn(joint.find('parent').get('link'), links)
                self.assertIn(joint.find('child').get('link'), links)
            self.assertEqual(len(links), len(model.findall('link')))

    def test_opposing_fingers_and_physical_gap(self):
        model = self.model('gripper')
        faces = []
        for side, sign in (('left', 1), ('right', -1)):
            joint = model.find(f"joint[@name='ur5e_gripper_{side}_joint']")
            self.assertEqual(joint.get('type'), 'prismatic')
            axis = list(map(float, joint.find('axis').get('xyz').split()))
            self.assertEqual(axis, [0, sign, 0])
            qmax = float(joint.find('limit').get('upper'))
            center_y = float(joint.find('origin').get('xyz').split()[1])
            faces.append(center_y + sign * qmax - sign * 0.006)
            interface = model.find(f"ros2_control[@name='GenericGripperSystem']/joint[@name='{joint.get('name')}']")
            self.assertAlmostEqual(float(interface.find("state_interface[@name='position']/param").text), 0.03)
        self.assertAlmostEqual(faces[0] - faces[1], 0.08)
        self.assertEqual(len(model.findall('ros2_control')), 3)  # chassis, UR, gripper; one manager plugin
        plugins = model.findall("gazebo/plugin[@name='ign_ros2_control::IgnitionROS2ControlPlugin']")
        self.assertEqual(len(plugins), 1)

    def test_planning_and_controller_selection(self):
        for tool in ('gripper', 'camera', 'none'):
            parameters = moveit_parameters(tool)
            semantic = ET.fromstring(parameters[1]['robot_description_semantic'])
            chain = semantic.find("group[@name='ur5e_ur_manipulator']/chain")
            expected = 'ur5e_gripper_tcp' if tool == 'gripper' else 'ur5e_tool0'
            self.assertEqual(chain.get('tip_link'), expected)
            self.assertEqual(manager_parameters(tool)['tool_link'], expected)
            controllers = merged_controllers(tool)['controller_manager']['ros__parameters']
            self.assertEqual('gripper_controller' in controllers, tool == 'gripper')
            if tool == 'gripper':
                state = semantic.find("group_state[@name='open'][@group='ur5e_gripper']")
                self.assertEqual([float(j.get('value')) for j in state], [0.04, 0.04])
                self.assertIsNotNone(semantic.find("end_effector[@name='parallel_gripper']"))

    def test_invalid_configuration_rejected(self):
        with self.assertRaises(ValueError):
            robot_description(tool_type='unknown')
        for key, value in [('initial_width', 0.09), ('max_width', -1), ('finger_speed', float('nan'))]:
            config = gripper_config()
            config[key] = value
            with tempfile.TemporaryDirectory() as directory:
                filename = Path(directory) / 'gripper.yaml'
                filename.write_text(yaml.safe_dump(config))
                with self.assertRaises(ValueError):
                    robot_description(gripper_file=str(filename))

    @unittest.skipUnless(os.environ.get('GRIPPER_PLANNING_CHECK'), 'MoveIt helper is supplied by CTest')
    def test_stowed_gripper_clearance(self):
        parameters = moveit_parameters('gripper')
        with tempfile.TemporaryDirectory() as directory:
            urdf, srdf = Path(directory) / 'robot.urdf', Path(directory) / 'robot.srdf'
            urdf.write_text(parameters[0]['robot_description'])
            srdf.write_text(parameters[1]['robot_description_semantic'])
            for width in (0.0, gripper_config()['initial_width'], gripper_config()['max_width']):
                subprocess.run([os.environ['GRIPPER_PLANNING_CHECK'], str(urdf), str(srdf), str(width)],
                               check=True, timeout=20)


if __name__ == '__main__':
    unittest.main()
