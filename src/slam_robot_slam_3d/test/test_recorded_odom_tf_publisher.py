from importlib.machinery import SourceFileLoader
from pathlib import Path

from nav_msgs.msg import Odometry
import pytest


MODULE = SourceFileLoader(
    "recorded_odom_tf_publisher",
    str(
        Path(__file__).parents[1] / "scripts" / "recorded_odom_tf_publisher"
    ),
).load_module()


def make_odometry():
    message = Odometry()
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 500_000_000
    message.header.frame_id = "odom"
    message.child_frame_id = "base_footprint"
    message.pose.pose.position.x = 1.5
    message.pose.pose.position.y = -2.5
    message.pose.pose.position.z = 0.25
    message.pose.pose.orientation.z = 0.3826834
    message.pose.pose.orientation.w = 0.9238795
    return message


def test_transform_copies_the_recorded_pose_and_stamp_verbatim():
    transform = MODULE.transform_from_odometry(make_odometry(), "odom", "base_footprint")

    assert transform.header.stamp.sec == 12
    assert transform.header.stamp.nanosec == 500_000_000
    assert transform.header.frame_id == "odom"
    assert transform.child_frame_id == "base_footprint"
    assert transform.transform.translation.x == pytest.approx(1.5)
    assert transform.transform.translation.y == pytest.approx(-2.5)
    assert transform.transform.translation.z == pytest.approx(0.25)
    assert transform.transform.rotation.z == pytest.approx(0.3826834)
    assert transform.transform.rotation.w == pytest.approx(0.9238795)


def test_frames_fall_back_to_the_recorded_message_frames():
    transform = MODULE.transform_from_odometry(make_odometry(), "", "")

    assert transform.header.frame_id == "odom"
    assert transform.child_frame_id == "base_footprint"
