from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[1]
WORKSPACE_ROOT = Path(__file__).parents[3]


def test_sim_model_has_required_frames_and_plugins():
    model_path = WORKSPACE_ROOT / "src/traymover_robot_description/urdf/traymover_sim.urdf.xacro"
    root = ET.parse(model_path).getroot()
    links = {node.attrib.get("name") for node in root.findall("link")}
    assert {"base_link", "laser"} <= links
    text = model_path.read_text()
    assert "gz-sim-diff-drive-system" in text
    assert "gz-sim-lidar-system" in text or "gpu_lidar" in text
    # Gazebo Sim system plugins must be model-scoped in the generated SDF;
    # reference-scoping this plugin puts it below a link, where it is ignored.
    assert '<gazebo reference="base_link">' not in text
    assert "<gazebo>" in text
    assert "<gz_frame_id>laser</gz_frame_id>" in text
    assert "<frame_id>laser</frame_id>" not in text
    assert '<xacro:drive_wheel name="right_wheel" y="-${wheel_separation / 2.0}" axis="1"/>' in text
    assert "<topic>/cmd_vel</topic>" in text
    assert "<odom_topic>/odom</odom_topic>" in text
    assert "<topic>/scan</topic>" in text


def test_demo_map_has_expected_metadata():
    yaml_text = (ROOT / "maps/detour_demo.yaml").read_text()
    assert "detour_demo.pgm" in yaml_text
    assert "resolution: 0.1" in yaml_text
    assert "origin: [0.0, -3.0, 0.0]" in yaml_text


def test_world_and_dynamic_box_use_required_geometry():
    world_text = (ROOT / "worlds/traymover_detour.sdf").read_text()
    box_text = (ROOT / "models/dynamic_box/model.sdf").read_text()
    # The boundary walls are the fixed scene geometry; keep the runtime box
    # as the sole navigational obstacle so its detour has ample clearance.
    assert "<model name=\"wall_north\">" in world_text
    assert "static_block_north" not in world_text
    assert "static_block_south" not in world_text
    assert "dynamic_box" not in world_text
    assert "<size>0.9 0.9 0.9</size>" in box_text
    assert "<pose>4.0 0.0 0.45" in box_text


def test_world_has_bright_materials_lighting_and_top_down_camera():
    world_text = (ROOT / "worlds/traymover_detour.sdf").read_text()
    root = ET.fromstring(world_text)
    world = root.find("world")
    assert world is not None

    scene = world.find("scene")
    assert scene is not None
    assert scene.findtext("ambient") == "0.65 0.65 0.65 1"
    assert scene.findtext("background") == "0.88 0.92 0.98 1"

    sun = world.find("light[@name='sun']")
    assert sun is not None
    assert sun.attrib.get("type") == "directional"
    assert sun.findtext("diffuse") == "1 1 1 1"

    gui = world.find("gui")
    assert gui is not None
    scene_view = gui.find("plugin[@filename='GzScene3D']")
    assert scene_view is not None
    assert scene_view.findtext("camera_pose") == "4 0 9 0 1.5708 0"

    materials = {
        model.attrib["name"]: model.find("link/visual/material/diffuse")
        for model in world.findall("model")
        if model.find("link/visual/material/diffuse") is not None
    }
    assert {"floor", "wall_west", "wall_east", "wall_north", "wall_south"} <= materials.keys()
    assert len({material.text for material in materials.values()}) >= 4


def test_runtime_dynamic_box_is_spawned_but_not_pushable():
    box_text = (ROOT / "models/dynamic_box/model.sdf").read_text()
    root = ET.fromstring(box_text)
    model = root.find("model")
    assert model is not None
    # The obstacle appears at runtime through ros_gz_sim/create, but it must
    # remain fixed so the demo measures path detouring rather than pushing.
    assert model.findtext("static") == "true"


def test_dynamic_box_and_sim_robot_use_high_contrast_colors():
    box_text = (ROOT / "models/dynamic_box/model.sdf").read_text()
    box_root = ET.fromstring(box_text)
    box_diffuse = box_root.findtext("model/link/visual/material/diffuse")
    assert box_diffuse == "0.95 0.12 0.04 1"

    robot_text = WORKSPACE_ROOT / "src/traymover_robot_description/urdf/traymover_sim.urdf.xacro"
    robot = robot_text.read_text()
    assert 'name="traymover_blue"><color rgba="0.03 0.55 0.95 1"' in robot
    assert 'name="traymover_yellow"><color rgba="1.0 0.72 0.0 1"' in robot


def test_bridge_lists_clock_scan_odom_and_cmd_vel_with_correct_directions():
    bridge = (ROOT / "config/bridge.yaml").read_text()
    for topic, ros_type, gz_type, direction in (
        ("/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", "GZ_TO_ROS"),
        ("/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan", "GZ_TO_ROS"),
        ("/odom", "nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS"),
        ("/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist", "ROS_TO_GZ"),
    ):
        assert f"ros_topic_name: {topic}" in bridge
        assert f"ros_type_name: {ros_type}" in bridge
        assert f"gz_type_name: {gz_type}" in bridge
        assert f"direction: {direction}" in bridge
    assert bridge.count("direction: GZ_TO_ROS") == 3
    assert bridge.count("direction: ROS_TO_GZ") == 1


def test_map_is_ascii_pgm_with_requested_dimensions_and_walls():
    values = (ROOT / "maps/detour_demo.pgm").read_text().split()
    assert values[:4] == ["P2", "80", "60", "255"]
    pixels = values[4:]
    assert len(pixels) == 80 * 60
    assert all(p in {"0", "255"} for p in pixels)
    assert pixels[0] == pixels[79] == "0"
    assert pixels[-80] == pixels[-1] == "0"
