from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "dataset_contract_check"
LOADER = SourceFileLoader("dataset_contract_check", str(SCRIPT_PATH))
SPEC = importlib.util.spec_from_loader("dataset_contract_check", LOADER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def metadata(topic_overrides=None, duration=150.0):
    topics = dict(MODULE.EXPECTED_TOPICS)
    if topic_overrides:
        topics.update(topic_overrides)
    entries = []
    for name, message_type in topics.items():
        entries.append(
            {
                "topic_metadata": {"name": name, "type": message_type},
                "message_count": 1500 if name == "/lidar_3d/points" else 1,
            }
        )
    return {
        "rosbag2_bagfile_information": {
            "storage_identifier": "mcap",
            "duration": {"nanoseconds": int(duration * 1.0e9)},
            "topics_with_message_count": entries,
        }
    }


def test_expected_fixed_input_contract_is_accepted():
    errors, duration, topics = MODULE.validate_metadata(metadata(), 120.0, 1200)

    assert errors == []
    assert duration == 150.0
    assert len(topics) == len(MODULE.EXPECTED_TOPICS)


def test_algorithm_output_and_dynamic_tf_are_rejected():
    data = metadata(
        {
            "/tf": "tf2_msgs/msg/TFMessage",
            "/rtabmap/map": "nav_msgs/msg/OccupancyGrid",
        }
    )

    errors, _, _ = MODULE.validate_metadata(data, 120.0, 1200)

    assert any("unexpected topics" in error for error in errors)


def test_missing_wrong_type_and_short_bag_are_rejected():
    data = metadata({"/odom": "geometry_msgs/msg/PoseStamped"}, duration=5.0)
    entries = data["rosbag2_bagfile_information"]["topics_with_message_count"]
    entries[:] = [
        entry
        for entry in entries
        if entry["topic_metadata"]["name"] != "/imu/data"
    ]
    for entry in entries:
        if entry["topic_metadata"]["name"] == "/lidar_3d/points":
            entry["message_count"] = 10

    errors, _, _ = MODULE.validate_metadata(data, 120.0, 1200)

    assert any("duration" in error for error in errors)
    assert any("missing topics" in error for error in errors)
    assert any("/odom type" in error for error in errors)
    assert any("point cloud count" in error for error in errors)
