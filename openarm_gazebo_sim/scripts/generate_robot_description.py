#!/usr/bin/env python3
"""Generate the v11 simulation URDF with Gazebo-specific model repairs."""

from io import StringIO
from pathlib import Path
import re
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
import xacro


HAND_MOUNT_LINKS = (
    "openarm_left_link7",
    "openarm_left_hand",
    "openarm_right_link7",
    "openarm_right_hand",
)

GRIPPER_JOINTS = (
    "openarm_left_finger_joint1",
    "openarm_left_finger_joint2",
    "openarm_right_finger_joint1",
    "openarm_right_finger_joint2",
)

V11_INCLUDE = "$(find openarm_description)/urdf/robot/v11.urdf.xacro"
MALFORMED_EFFORT_EXPRESSION = re.compile(r"\$\{effort[^}]+\}")


def find_named_element(document, tag_name, name):
    for element in document.getElementsByTagName(tag_name):
        if element.getAttribute("name") == name:
            return element
    raise RuntimeError(f"Missing {tag_name} '{name}' in generated v11 URDF")


def add_minimal_inertial(document, link):
    if any(
        child.nodeType == child.ELEMENT_NODE and child.tagName == "inertial"
        for child in link.childNodes
    ):
        return

    # 50 g (not 1 g): the grasp plugin welds the object to these links and a
    # too-light welded body destabilises ODE, flinging the object away
    inertial = document.createElement("inertial")
    origin = document.createElement("origin")
    origin.setAttribute("xyz", "0 0 0")
    origin.setAttribute("rpy", "0 0 0")
    mass = document.createElement("mass")
    mass.setAttribute("value", "0.05")
    inertia = document.createElement("inertia")
    for attribute, value in {
        "ixx": "5e-4",
        "ixy": "0",
        "ixz": "0",
        "iyy": "5e-4",
        "iyz": "0",
        "izz": "5e-4",
    }.items():
        inertia.setAttribute(attribute, value)

    inertial.appendChild(origin)
    inertial.appendChild(mass)
    inertial.appendChild(inertia)
    link.appendChild(inertial)


def add_gripper_joint_dynamics(document, joint):
    for child in joint.childNodes:
        if child.nodeType == child.ELEMENT_NODE and child.tagName == "dynamics":
            child.setAttribute("damping", "4.0")
            child.setAttribute("friction", "0.2")
            return

    dynamics = document.createElement("dynamics")
    dynamics.setAttribute("damping", "4.0")
    dynamics.setAttribute("friction", "0.2")
    joint.appendChild(dynamics)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: generate_robot_description.py XACRO_FILE")

    wrapper_path = Path(sys.argv[1])
    wrapper_xml = wrapper_path.read_text(encoding="utf-8")
    v11_path = (
        Path(get_package_share_directory("openarm_description"))
        / "urdf"
        / "robot"
        / "v11.urdf.xacro"
    )
    v11_xml = v11_path.read_text(encoding="utf-8")

    if MALFORMED_EFFORT_EXPRESSION.search(v11_xml):
        print(
            f"warning: repairing malformed effort expression in {v11_path} "
            "for this simulation only",
            file=sys.stderr,
        )
        v11_xml = MALFORMED_EFFORT_EXPRESSION.sub("${effort}", v11_xml)
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".urdf.xacro", encoding="utf-8"
        ) as temporary_v11:
            temporary_v11.write(v11_xml)
            temporary_v11.flush()
            wrapper_xml = wrapper_xml.replace(V11_INCLUDE, temporary_v11.name)
            document = xacro.parse(StringIO(wrapper_xml), str(wrapper_path))
            xacro.process_doc(
                document,
                mappings={"bimanual": "true", "ros2_control": "false"},
            )
    else:
        document = xacro.process_file(
            str(wrapper_path),
            mappings={"bimanual": "true", "ros2_control": "false"},
        )

    # Gazebo Classic removes massless fixed-link chains and all downstream
    # prismatic joints. Tiny inertias preserve the two v11 grippers.
    for link_name in HAND_MOUNT_LINKS:
        add_minimal_inertial(
            document, find_named_element(document, "link", link_name)
        )

    # The v11 gripper fingers are light prismatic bodies. Gazebo's default
    # zero-damping sliders can oscillate while ros2_control holds position,
    # especially on the mirrored left hand.
    for joint_name in GRIPPER_JOINTS:
        add_gripper_joint_dynamics(
            document, find_named_element(document, "joint", joint_name)
        )

    sys.stdout.write(document.toxml())


if __name__ == "__main__":
    main()
