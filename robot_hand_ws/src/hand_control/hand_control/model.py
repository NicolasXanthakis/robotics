"""Shared hand model metadata and wire ordering."""

import xml.etree.ElementTree as ET

JOINT_NAMES = tuple(
    f'{finger}_{joint}'
    for finger in ('index', 'middle', 'ring', 'pinky')
    for joint in ('mcp', 'pip', 'dip')
) + ('thumb_base', 'thumb_ip')


def load_joints(description):
    """Read joint names, position bounds and velocity limits from expanded URDF."""
    root = ET.fromstring(description)
    return [
        (joint.attrib['name'],
         float(joint.find('limit').attrib['lower']),
         float(joint.find('limit').attrib['upper']),
         float(joint.find('limit').attrib['velocity']))
        for joint in root.findall('joint')
        if joint.attrib['type'] == 'revolute'
    ]
