from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "rgbd_dataset_contract_check"
)
LOADER = SourceFileLoader("rgbd_dataset_contract_check", str(SCRIPT_PATH))
SPEC = importlib.util.spec_from_loader("rgbd_dataset_contract_check", LOADER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def metadata(duration=180.0, image_count=1800, cloud_count=1800):
    entries = []
    for name, type_name in MODULE.EXPECTED_TOPICS.items():
        count = 1
        if name in (MODULE.COLOR_TOPIC, MODULE.DEPTH_TOPIC):
            count = image_count
        elif "camera_info" in name:
            count = image_count
        elif name == "/lidar_3d/points":
            count = cloud_count
        entries.append(
            {
                "topic_metadata": {"name": name, "type": type_name},
                "message_count": count,
            }
        )
    return {
        "rosbag2_bagfile_information": {
            "storage_identifier": "mcap",
            "duration": {"nanoseconds": int(duration * 1.0e9)},
            "custom_data": {
                "dataset": "structured_rgbd_reference",
                "contract_version": 1,
                "camera_rate_hz": 10.0,
                "storage_profile": "zstd_fast",
            },
            "topics_with_message_count": entries,
        }
    }


def validate(value):
    return MODULE.validate_metadata(value, 150.0, 1500, 1500, 9.0, 11.0)[0]


def test_valid_fixed_rgbd_metadata_passes():
    assert validate(metadata()) == []


def test_recorded_route_laps_must_be_positive_when_present():
    value = metadata()
    custom_data = value["rosbag2_bagfile_information"]["custom_data"]
    custom_data["route_laps"] = "2"
    assert validate(value) == []

    custom_data["route_laps"] = "0"
    assert any("route_laps" in error for error in validate(value))


def test_missing_camera_topic_and_wrong_rate_fail():
    value = metadata(image_count=1200)
    entries = value["rosbag2_bagfile_information"]["topics_with_message_count"]
    entries[:] = [
        entry
        for entry in entries
        if entry["topic_metadata"]["name"] != MODULE.DEPTH_INFO_TOPIC
    ]

    errors = validate(value)

    assert any("missing topics" in error for error in errors)
    assert any("image counts are too low" in error for error in errors)
    assert any("image rate" in error for error in errors)


def test_unexpected_algorithm_output_is_rejected():
    value = metadata()
    value["rosbag2_bagfile_information"]["topics_with_message_count"].append(
        {
            "topic_metadata": {
                "name": "/rtabmap/map",
                "type": "nav_msgs/msg/OccupancyGrid",
            },
            "message_count": 10,
        }
    )

    assert any("unexpected topics" in error for error in validate(value))
