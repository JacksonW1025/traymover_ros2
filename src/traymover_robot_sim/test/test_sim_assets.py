from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[1]


def test_sim_model_has_required_frames_and_plugins():
    model_path = Path("src/traymover_robot_description/urdf/traymover_sim.urdf.xacro")
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
    assert "static_block_north" in world_text
    assert "static_block_south" in world_text
    assert "dynamic_box" not in world_text
    assert "<size>0.9 0.9 0.9</size>" in box_text
    assert "<pose>4.0 0.0 0.45" in box_text


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
