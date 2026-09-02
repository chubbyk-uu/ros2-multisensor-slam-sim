"""
Keep tests off shared, predictable paths.

Tests in this workspace run in parallel: colcon runs packages concurrently, and
nothing stops a second `colcon test` from starting while the first is going.
A test that writes to a name it picked itself under /tmp is sharing state with
whatever else guessed the same name, and the failure does not look like a
sharing problem -- the file another case is halfway through reading gets
deleted, and the case reports a truncated or missing snapshot, which reads as a
defect in the format.

The invariant is narrow on purpose: derive the name from mkdtemp (or pytest's
tmp_path), never from a literal. A literal that ends in the mkdtemp template is
the intended construction and is allowed.

The unit tests here are all in-process -- no ROS nodes, no DDS -- so the shared
medium is the filesystem rather than a domain ID. Should a launch test ever be
added, its isolation is a separate question this file does not answer.
"""

from pathlib import Path
import re

WORKSPACE = Path(__file__).resolve().parents[3]

# The system temporary directory joined with a string literal, unless that
# literal is an mkdtemp template. Written without an example of the pattern
# it matches, because this file is one of the files it scans.
LITERAL_TEMPORARY = re.compile(
    r"temp_directory_path\(\)\s*/\s*\(?\s*\"([^\"]*)\"")

# A test writing straight into a path it spelled out itself.
LITERAL_WRITE = re.compile(
    r"(?:open|write_text|write_bytes|mkdir|touch)\(\s*[\"']/tmp/")


def test_files():
    return sorted(WORKSPACE.glob("src/*/test/*.cpp")) + sorted(
        WORKSPACE.glob("src/*/test/*.py"))


def test_the_workspace_has_tests_to_check():
    # Without this the two checks below pass on an empty glob, which is exactly
    # the shape of vacuous success they are meant to rule out elsewhere.
    assert len(test_files()) > 20


def test_no_test_builds_a_temporary_path_from_a_literal_name():
    offenders = []
    for source in test_files():
        for name in LITERAL_TEMPORARY.findall(source.read_text()):
            if not name.endswith("XXXXXX"):
                offenders.append(f"{source.name}: {name}")

    assert not offenders, (
        "derive these from mkdtemp instead, so two concurrent runs cannot "
        f"collide: {offenders}")


def test_no_test_writes_to_a_hardcoded_tmp_path():
    offenders = [
        source.name for source in test_files()
        if LITERAL_WRITE.search(source.read_text())
    ]

    assert not offenders
