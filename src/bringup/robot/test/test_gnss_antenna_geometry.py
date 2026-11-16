"""Keep the antenna mounting yaw and all shipped robot descriptions consistent."""
import ast
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def test_main_sub_geometry_matches_heading_correction():
    share = Path(__file__).parents[1]
    tree = ET.parse(share/'urdf/robot.urdf.xacro')
    values = {node.attrib['name']: float(node.attrib['value'])
              for node in tree.getroot().iter()
              if node.tag.endswith('}property') and node.attrib.get('name', '').startswith(('gnss1_', 'gnss2_'))}
    main = [values[f'gnss1_{axis}'] for axis in 'xyz']
    sub = [values[f'gnss2_{axis}'] for axis in 'xyz']
    for filename in ('robot.urdf_modified.urdf', 'robot_agx.urdf_modified.urdf'):
        robot = ET.parse(share/'urdf'/filename).getroot()
        for joint, expected in [('gnss_joint', main), ('gnss_sub_joint', sub)]:
            origin = robot.find(f"joint[@name='{joint}']/origin")
            assert list(map(float, origin.attrib['xyz'].split())) == expected
    launch = share.parents[1]/'driver/gnss/um982_driver/launch/um982.launch.py'
    assignments = [node for node in ast.walk(ast.parse(launch.read_text()))
                   if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == 'arg_heading_baseline_yaw' for target in node.targets)]
    assert len(assignments) == 1
    default = next(keyword.value for keyword in assignments[0].value.keywords if keyword.arg == 'default_value')
    configured = float(ast.literal_eval(default))
    expected = math.atan2(sub[1]-main[1], sub[0]-main[0])
    assert abs(math.atan2(math.sin(configured-expected), math.cos(configured-expected))) < 1e-12
