from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BODY_LEN = 0.036
BODY_WIDTH = 0.018
BODY_THICKNESS = 0.008
BODY_FRONT_X = BODY_LEN / 2.0
PIVOT_X = -0.014

PLUG_LEN = 0.012
PLUG_WIDTH = 0.012
PLUG_THICKNESS = 0.0048

WINDOW_LEN = 0.0048
WINDOW_WIDTH = 0.0028
WINDOW_THICKNESS = 0.0008

COVER_LEN = 0.044
COVER_WIDTH = 0.020
METAL_THICKNESS = 0.001
SHELL_CLEARANCE = 0.0003
COVER_PLATE_Z = BODY_THICKNESS / 2.0 + SHELL_CLEARANCE + METAL_THICKNESS / 2.0
FRONT_BRIDGE_LEN = 0.003


def _make_materials() -> dict[str, Material]:
    return {
        "matte_black": Material(
            name="matte_black",
            rgba=(0.11, 0.12, 0.13, 1.0),
        ),
        "graphite_trim": Material(
            name="graphite_trim",
            rgba=(0.20, 0.21, 0.23, 1.0),
        ),
        "brushed_steel": Material(
            name="brushed_steel",
            rgba=(0.77, 0.79, 0.81, 1.0),
        ),
        "polished_steel": Material(
            name="polished_steel",
            rgba=(0.90, 0.91, 0.93, 1.0),
        ),
        "dark_polymer": Material(
            name="dark_polymer",
            rgba=(0.05, 0.05, 0.06, 1.0),
        ),
        "smoked_window": Material(
            name="smoked_window",
            rgba=(0.30, 0.48, 0.70, 0.55),
        ),
    }


def _build_body_shell_mesh():
    profile = rounded_rect_profile(
        BODY_THICKNESS,
        BODY_WIDTH,
        radius=0.0022,
        corner_segments=8,
    )
    geom = ExtrudeGeometry(profile, BODY_LEN, center=True)
    geom.rotate_y(pi / 2.0)
    return mesh_from_geometry(geom, ASSETS.mesh_path("usb_flash_drive_body.obj"))


def _build_cover_mesh():
    plate_len = COVER_LEN - FRONT_BRIDGE_LEN
    bridge_thickness = BODY_THICKNESS + (2.0 * SHELL_CLEARANCE) + (2.0 * METAL_THICKNESS)

    top_plate = BoxGeometry((plate_len, COVER_WIDTH, METAL_THICKNESS))
    top_plate.translate(plate_len / 2.0, 0.0, COVER_PLATE_Z)

    bottom_plate = BoxGeometry((plate_len, COVER_WIDTH, METAL_THICKNESS))
    bottom_plate.translate(plate_len / 2.0, 0.0, -COVER_PLATE_Z)

    front_bridge = BoxGeometry((FRONT_BRIDGE_LEN, COVER_WIDTH, bridge_thickness))
    front_bridge.translate(plate_len + (FRONT_BRIDGE_LEN / 2.0), 0.0, 0.0)

    top_reinforcement = BoxGeometry((0.010, 0.012, METAL_THICKNESS * 1.4))
    top_reinforcement.translate(0.006, 0.0, COVER_PLATE_Z)

    bottom_reinforcement = BoxGeometry((0.010, 0.012, METAL_THICKNESS * 1.4))
    bottom_reinforcement.translate(0.006, 0.0, -COVER_PLATE_Z)

    cover_geom = top_plate
    cover_geom.merge(bottom_plate)
    cover_geom.merge(front_bridge)
    cover_geom.merge(top_reinforcement)
    cover_geom.merge(bottom_reinforcement)
    return mesh_from_geometry(cover_geom, ASSETS.mesh_path("usb_flash_drive_cover.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_flash_drive", assets=ASSETS)

    materials = _make_materials()
    model.materials.extend(materials.values())

    body_shell = _build_body_shell_mesh()
    cover_shell = _build_cover_mesh()

    body = model.part("body")
    body.visual(
        body_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=materials["matte_black"],
        name="body_shell",
    )
    body.visual(
        Box((0.0016, BODY_WIDTH * 1.01, BODY_THICKNESS)),
        origin=Origin(xyz=(BODY_FRONT_X - 0.0008, 0.0, 0.0)),
        material=materials["graphite_trim"],
        name="front_band",
    )
    body.visual(
        Box((PLUG_LEN, PLUG_WIDTH, PLUG_THICKNESS)),
        origin=Origin(
            xyz=(BODY_FRONT_X + (PLUG_LEN / 2.0) - 0.0002, 0.0, 0.0),
        ),
        material=materials["brushed_steel"],
        name="usb_shell",
    )
    body.visual(
        Box((PLUG_LEN - 0.0025, PLUG_WIDTH - 0.0030, 0.0012)),
        origin=Origin(
            xyz=(BODY_FRONT_X + (PLUG_LEN / 2.0) + 0.0003, 0.0, 0.0),
        ),
        material=materials["dark_polymer"],
        name="usb_tongue",
    )
    body.visual(
        Cylinder(radius=0.0016, length=BODY_THICKNESS + 0.0012),
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.0)),
        material=materials["polished_steel"],
        name="pivot_pin",
    )
    body.visual(
        Cylinder(radius=0.0041, length=0.0006),
        origin=Origin(xyz=(PIVOT_X, 0.0, BODY_THICKNESS / 2.0 - 0.0003)),
        material=materials["polished_steel"],
        name="pivot_head_top",
    )
    body.visual(
        Cylinder(radius=0.0041, length=0.0006),
        origin=Origin(xyz=(PIVOT_X, 0.0, -(BODY_THICKNESS / 2.0) + 0.0003)),
        material=materials["polished_steel"],
        name="pivot_head_bottom",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_LEN + PLUG_LEN + 0.002, BODY_WIDTH, BODY_THICKNESS + 0.002)),
        mass=0.018,
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
    )

    status_window = model.part("status_window")
    status_window.visual(
        Box((WINDOW_LEN, WINDOW_WIDTH, WINDOW_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, WINDOW_THICKNESS / 2.0)),
        material=materials["smoked_window"],
        name="status_window_lens",
    )
    status_window.inertial = Inertial.from_geometry(
        Box((WINDOW_LEN, WINDOW_WIDTH, WINDOW_THICKNESS)),
        mass=0.0002,
        origin=Origin(xyz=(0.0, 0.0, WINDOW_THICKNESS / 2.0)),
    )

    cover = model.part("cover")
    cover.visual(
        cover_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=materials["brushed_steel"],
        name="swivel_cover",
    )
    cover.inertial = Inertial.from_geometry(
        Box(
            (
                COVER_LEN,
                COVER_WIDTH,
                BODY_THICKNESS + (2.0 * SHELL_CLEARANCE) + (2.0 * METAL_THICKNESS),
            )
        ),
        mass=0.009,
        origin=Origin(xyz=(COVER_LEN / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_status_window",
        ArticulationType.FIXED,
        parent="body",
        child="status_window",
        origin=Origin(xyz=(-0.0045, 0.0, BODY_THICKNESS / 2.0)),
    )
    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent="body",
        child="cover",
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=6.0,
            lower=0.0,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_within("status_window", "body", axes="xy")
    ctx.expect_aabb_gap("status_window", "body", axis="z", max_gap=0.001, max_penetration=0.0007)

    ctx.expect_joint_motion_axis(
        "body_to_cover",
        "cover",
        world_axis="x",
        direction="negative",
        min_delta=0.03,
    )

    with ctx.pose(body_to_cover=0.0):
        ctx.expect_aabb_overlap("cover", "body", axes="xy", min_overlap=0.014)
        ctx.expect_origin_distance("cover", "body", axes="xy", max_dist=0.016)

    with ctx.pose(body_to_cover=pi / 2.0):
        ctx.expect_aabb_overlap("cover", "body", axes="xy", min_overlap=0.008)
        ctx.expect_origin_distance("cover", "body", axes="xy", max_dist=0.030)

    with ctx.pose(body_to_cover=pi):
        ctx.expect_aabb_overlap("cover", "body", axes="xy", min_overlap=0.003)
        ctx.expect_origin_distance("cover", "body", axes="xy", max_dist=0.038)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
