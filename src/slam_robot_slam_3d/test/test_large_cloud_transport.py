"""
Keep every durable writer inside a shared-memory segment that can hold it.

Fast DDS gives a shared-memory segment 512 KiB by default. A reliable writer
whose sample does not fit fragments it, and losing one fragment costs the
reader a full 3 s heartbeat period before the gap is repaired. The symptom is
a single map update stalling for seconds while every small topic keeps
flowing, which reads like a compute problem rather than a transport one.

Transient-local durability makes this easier to reach rather than harder:
every late joiner replays the whole payload, so RViz restarting or the Nav2
static layer reconnecting is enough to trigger it.

This is a static test on purpose. Reproducing the stall needs a loaded host and
a late joiner arriving at the wrong moment, so a runtime test would be the kind
of low-frequency failure that gets closed by re-running it. What can be checked
deterministically is the invariant: every durable writer in the workspace is
either covered by the large-segment profile or has a measured payload that
fits the default one, and a new writer belongs to neither set until someone
says which.

The launch file is read as source rather than executed, matching
test_exploration_seed_wiring: the value is a substitution object nested in
launch actions, and comparing the text that was written is closer to what a
reader has to keep consistent.
"""

import ast
from pathlib import Path
import re
import xml.etree.ElementTree as ElementTree

WORKSPACE = Path(__file__).resolve().parents[3]
PACKAGE = Path(__file__).resolve().parents[1]
PROFILE = PACKAGE / "config" / "fastdds_large_cloud.xml"
FRONT_END_LAUNCH = PACKAGE / "launch" / "custom_3d_front_end.launch.py"
PROFILES_VARIABLE = "FASTRTPS_DEFAULT_PROFILES_FILE"

# The Fast DDS default this whole file exists to get out from under.
DEFAULT_SEGMENT_BYTES = 512 * 1024

# Largest payload actually measured on a durable topic, from the localization
# regression in docs/results/2026-08-12-snapshot-localization.json: the restored
# global cloud is installed whole as the match target, so local_map carries
# 110,544 PointXYZI points at a 32-byte point_step.
LARGEST_MEASURED_PAYLOAD_BYTES = 110544 * 32

# Durable writers that need the large segment. The count is part of the
# contract: adding a fifth topic to this node should still pass, but only after
# someone has looked at whether it fits.
WRITERS_NEEDING_THE_PROFILE = {
    "scan_to_map_odometry_node.cpp": 4,
}

# Durable writers left on the default segment, with the measurement that says
# they fit. Every entry here is a claim someone checked, not an omission.
#
#   scan_matcher_odometry_node.cpp: the 2D occupancy grid measures 244 x 204
#   cells on the saved reference map, and 26 x 20 m at 0.05 m -- the largest
#   world -- is 208 KiB. The pose-graph Path is downsampled by distance and
#   length-capped.
#
#   frontier_explorer_node.cpp: a Bool and two MarkerArrays holding one marker
#   per frontier cluster. The /map subscription is a reader, and a reader does
#   not size the segment; the writer does.
WRITERS_ON_THE_DEFAULT_SEGMENT = {
    "scan_matcher_odometry_node.cpp": 2,
    "frontier_explorer_node.cpp": 3,
}

# Executables started by the launch file that injects the profile.
COVERED_EXECUTABLES = {
    "point_cloud_preprocessor_node",
    "scan_to_map_odometry_node",
}


def statements(text, opening):
    """Yield each call to `opening` up to the semicolon that ends it."""
    for start in (match.end() for match in re.finditer(re.escape(opening), text)):
        end = text.find(";", start)
        yield text[start:end if end >= 0 else len(text)]


def durable_writer_counts():
    counts = {}
    for source in sorted(WORKSPACE.glob("src/*/src/*.cpp")):
        text = source.read_text()
        durable = sum(
            1 for statement in statements(text, "create_publisher")
            if "transient_local" in statement
        )
        if durable:
            counts[source.name] = durable
    return counts


def launch_syntax():
    return ast.parse(FRONT_END_LAUNCH.read_text())


def calls_named(name):
    for node in ast.walk(launch_syntax()):
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        found = function.id if isinstance(function, ast.Name) else getattr(
            function, "attr", None)
        if found == name:
            yield node


def literals(node):
    return {
        child.value for child in ast.walk(node)
        if isinstance(child, ast.Constant) and isinstance(child.value, str)
    }


def transports():
    root = ElementTree.parse(PROFILE).getroot()
    namespace = {"dds": root.tag[1:root.tag.index("}")]} if root.tag.startswith(
        "{") else {}
    prefix = "dds:" if namespace else ""
    return root.findall(
        f".//{prefix}transport_descriptors/{prefix}transport_descriptor",
        namespace), namespace, prefix


def test_every_durable_writer_is_declared_as_covered_or_measured():
    # The failure this guards against is a new large topic added to a node the
    # profile never reaches, which is exactly how the original incident was
    # described: protecting the first hop is not protecting the writers.
    declared = dict(WRITERS_NEEDING_THE_PROFILE)
    declared.update(WRITERS_ON_THE_DEFAULT_SEGMENT)

    assert durable_writer_counts() == declared


def test_the_covered_writers_run_under_the_launch_that_injects_the_profile():
    started = {
        value for node in calls_named("Node") for value in literals(node)
    }

    for source in WRITERS_NEEDING_THE_PROFILE:
        assert source.replace(".cpp", "") in started
    assert COVERED_EXECUTABLES <= started


def test_the_front_end_launch_injects_the_profile():
    injections = [
        node for node in calls_named("SetEnvironmentVariable")
        if PROFILES_VARIABLE in literals(node)
    ]

    assert len(injections) == 1


def test_a_profile_the_operator_already_exported_wins():
    # Silently replacing an operator's own profile is worse than a large
    # payload: theirs may carry the discovery or security settings the run
    # depends on, and nothing would report the substitution.
    defaults = [
        node for node in calls_named("EnvironmentVariable")
        if PROFILES_VARIABLE in literals(node)
    ]

    assert len(defaults) == 1


def test_the_default_profile_is_the_one_this_package_ships():
    assert PROFILE.exists()
    assert PROFILE.name in FRONT_END_LAUNCH.read_text()


def test_the_segment_holds_the_largest_measured_payload():
    descriptors, namespace, prefix = transports()
    sizes = [
        int(descriptor.find(f"{prefix}segment_size", namespace).text)
        for descriptor in descriptors
        if descriptor.find(f"{prefix}type", namespace).text == "SHM"
    ]

    assert sizes, "the profile declares no shared-memory transport"
    assert min(sizes) > DEFAULT_SEGMENT_BYTES
    # Whole samples for several readers, not one in flight.
    assert min(sizes) >= 4 * LARGEST_MEASURED_PAYLOAD_BYTES


def test_the_profile_keeps_a_transport_that_leaves_the_host():
    # Naming a user transport turns the builtin ones off. A profile that then
    # lists only shared memory silently drops every peer that cannot map the
    # segment, and discovery failure looks nothing like a transport setting.
    descriptors, namespace, prefix = transports()
    kinds = {
        descriptor.find(f"{prefix}type", namespace).text
        for descriptor in descriptors
    }

    assert "SHM" in kinds
    assert kinds - {"SHM"}
