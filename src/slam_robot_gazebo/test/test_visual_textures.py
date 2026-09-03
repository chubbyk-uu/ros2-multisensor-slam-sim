from pathlib import Path
import struct
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).parents[1]
WORLD = PACKAGE_ROOT / "worlds" / "structured_loop_3d.sdf"
TEXTURES = PACKAGE_ROOT / "materials" / "textures"


def test_corridor_walls_use_eight_unique_visual_only_textures():
    root = ET.parse(WORLD).getroot()
    walls = (
        "outer_south",
        "inner_south",
        "outer_east",
        "inner_east",
        "outer_north",
        "inner_north",
        "outer_west",
        "inner_west",
    )
    references = []
    for wall in walls:
        visual = root.find(f".//visual[@name='{wall}_visual']")
        assert visual is not None
        albedo = visual.find("./material/pbr/metal/albedo_map")
        assert albedo is not None
        references.append(albedo.text)

        collision = root.find(f".//collision[@name='{wall}_collision']")
        assert collision is not None
        assert collision.find(".//albedo_map") is None

    assert len(set(references)) == len(walls)


def test_generated_textures_are_portable_rgb_png_assets():
    for path in sorted(TEXTURES.glob("structured_rgbd_*.png")):
        data = path.read_bytes()
        assert data.startswith(b"\x89PNG\r\n\x1a\n")
        width, height = struct.unpack(">II", data[16:24])
        assert (width, height) == (1024, 128)

    assert len(list(TEXTURES.glob("structured_rgbd_*.png"))) == 8
