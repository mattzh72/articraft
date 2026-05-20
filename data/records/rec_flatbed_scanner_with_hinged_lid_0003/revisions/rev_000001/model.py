from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import pathlib

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        os.chdir("/")
        return "/"


os.getcwd = _safe_getcwd
pathlib.os.getcwd = _safe_getcwd
pathlib.Path.cwd = classmethod(lambda cls: pathlib.Path(_safe_getcwd()))
_safe_getcwd()
if "__file__" in globals() and not os.path.isabs(__file__):
    __file__ = os.path.join(_safe_getcwd(), __file__)

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

BODY_WIDTH = 0.34
BODY_DEPTH = 0.255
BODY_HEIGHT = 0.032
BODY_CORNER_RADIUS = 0.018

LID_WIDTH = 0.334
LID_DEPTH = 0.242
LID_THICKNESS = 0.026
LID_CORNER_RADIUS = 0.016
LID_REAR_FACE_OFFSET = 0.010
LID_CENTER_Y = LID_REAR_FACE_OFFSET + (LID_DEPTH / 2.0)
LID_CENTER_Z = 0.010

HINGE_Y = -(BODY_DEPTH / 2.0) + 0.0025
HINGE_Z = 0.038
HINGE_RADIUS = 0.006
HINGE_LEAF_DEPTH = 0.008
HINGE_LEAF_CENTER_OFFSET = HINGE_RADIUS + (HINGE_LEAF_DEPTH / 2.0)
HINGE_SEGMENT_LENGTH = 0.041
HINGE_SEGMENT_GAP = 0.003
HINGE_SEGMENT_COUNT = 7


def _hinge_segment_centers() -> list[float]:
    total_length = (HINGE_SEGMENT_COUNT * HINGE_SEGMENT_LENGTH) + (
        (HINGE_SEGMENT_COUNT - 1) * HINGE_SEGMENT_GAP
    )
    first_center = -(total_length / 2.0) + (HINGE_SEGMENT_LENGTH / 2.0)
    return [
        first_center + idx * (HINGE_SEGMENT_LENGTH + HINGE_SEGMENT_GAP)
        for idx in range(HINGE_SEGMENT_COUNT)
    ]


def _rounded_shell_mesh(
    width: float,
    depth: float,
    height: float,
    radius: float,
    filename: str,
):
    geometry = ExtrudeGeometry(
        rounded_rect_profile(width, depth, radius, corner_segments=10),
        height,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flatbed_photo_scanner", assets=ASSETS)

    body_light = model.material("body_light", rgba=(0.90, 0.91, 0.92, 1.0))
    body_dark = model.material("body_dark", rgba=(0.28, 0.31, 0.35, 1.0))
    lid_top = model.material("lid_top", rgba=(0.84, 0.85, 0.87, 1.0))
    lid_inner = model.material("lid_inner", rgba=(0.95, 0.96, 0.97, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.17, 0.19, 0.21, 1.0))
    glass = model.material("glass", rgba=(0.08, 0.10, 0.13, 0.35))
    accent = model.material("accent", rgba=(0.22, 0.48, 0.92, 1.0))

    body = model.part("body")
    body_shell_mesh = _rounded_shell_mesh(
        BODY_WIDTH,
        BODY_DEPTH,
        BODY_HEIGHT,
        BODY_CORNER_RADIUS,
        "scanner_body_shell.obj",
    )
    body.visual(
        body_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
        material=body_light,
        name="body_shell",
    )
    body.visual(
        Box((0.300, 0.190, 0.005)),
        origin=Origin(xyz=(0.0, -0.006, 0.0325)),
        material=body_dark,
        name="scan_bezel",
    )
    body.visual(
        Box((0.244, 0.154, 0.0015)),
        origin=Origin(xyz=(0.0, -0.006, 0.0310)),
        material=glass,
        name="scan_glass",
    )
    body.visual(
        Box((0.272, 0.018, 0.003)),
        origin=Origin(xyz=(0.0, 0.106, 0.0335)),
        material=body_dark,
        name="control_strip",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.003),
        origin=Origin(xyz=(0.128, 0.106, 0.0335)),
        material=accent,
        name="status_button",
    )
    body.visual(
        Box((0.328, HINGE_LEAF_DEPTH, 0.003)),
        origin=Origin(
            xyz=(0.0, HINGE_Y + HINGE_LEAF_CENTER_OFFSET, 0.0335),
        ),
        material=hinge_dark,
        name="rear_hinge_plate",
    )

    hinge_centers = _hinge_segment_centers()
    body_knuckle_names = {
        0: "body_knuckle_left",
        2: "body_knuckle_inner_left",
        4: "body_knuckle_inner_right",
        6: "body_knuckle_right",
    }
    for idx, x_center in enumerate(hinge_centers):
        if idx % 2 == 0:
            body.visual(
                Cylinder(radius=HINGE_RADIUS, length=HINGE_SEGMENT_LENGTH),
                origin=Origin(
                    xyz=(x_center, HINGE_Y, HINGE_Z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=hinge_dark,
                name=body_knuckle_names[idx],
            )

    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
    )

    lid = model.part("lid")
    lid_shell_mesh = _rounded_shell_mesh(
        LID_WIDTH,
        LID_DEPTH,
        LID_THICKNESS,
        LID_CORNER_RADIUS,
        "scanner_lid_shell.obj",
    )
    lid.visual(
        lid_shell_mesh,
        origin=Origin(xyz=(0.0, LID_CENTER_Y, LID_CENTER_Z)),
        material=lid_top,
        name="lid_shell",
    )
    lid.visual(
        Box((0.284, 0.182, 0.010)),
        origin=Origin(xyz=(0.0, 0.129, 0.002)),
        material=lid_inner,
        name="lid_liner",
    )
    lid.visual(
        Box((0.326, HINGE_LEAF_DEPTH, 0.003)),
        origin=Origin(xyz=(0.0, HINGE_LEAF_CENTER_OFFSET, -0.0015)),
        material=hinge_dark,
        name="rear_leaf",
    )
    lid.visual(
        Box((0.120, 0.018, 0.004)),
        origin=Origin(
            xyz=(0.0, LID_REAR_FACE_OFFSET + LID_DEPTH - 0.009, -0.001),
        ),
        material=body_dark,
        name="front_lip",
    )

    lid_knuckle_names = {
        1: "lid_knuckle_left",
        3: "lid_knuckle_center",
        5: "lid_knuckle_right",
    }
    for idx, x_center in enumerate(hinge_centers):
        if idx % 2 == 1:
            lid.visual(
                Cylinder(radius=HINGE_RADIUS, length=HINGE_SEGMENT_LENGTH),
                origin=Origin(
                    xyz=(x_center, 0.0, 0.0),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=hinge_dark,
                name=lid_knuckle_names[idx],
            )

    lid.inertial = Inertial.from_geometry(
        Box((LID_WIDTH, LID_DEPTH, LID_THICKNESS)),
        mass=1.6,
        origin=Origin(xyz=(0.0, LID_CENTER_Y, LID_CENTER_Z)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("lid_hinge")

    body_shell = body.get_visual("body_shell")
    scan_bezel = body.get_visual("scan_bezel")
    scan_glass = body.get_visual("scan_glass")
    rear_hinge_plate = body.get_visual("rear_hinge_plate")
    body_knuckle_left = body.get_visual("body_knuckle_left")
    body_knuckle_right = body.get_visual("body_knuckle_right")

    lid_shell = lid.get_visual("lid_shell")
    lid_liner = lid.get_visual("lid_liner")
    rear_leaf = lid.get_visual("rear_leaf")
    front_lip = lid.get_visual("front_lip")
    lid_knuckle_left = lid.get_visual("lid_knuckle_left")
    lid_knuckle_right = lid.get_visual("lid_knuckle_right")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    ctx.expect_within(lid, body, axes="xy", inner_elem=lid_shell, outer_elem=body_shell)
    ctx.expect_within(body, body, axes="xy", inner_elem=scan_glass, outer_elem=scan_bezel)
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=lid_shell,
        negative_elem=scan_bezel,
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem=rear_leaf,
        negative_elem=rear_hinge_plate,
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="x",
        min_overlap=0.30,
        elem_a=rear_leaf,
        elem_b=rear_hinge_plate,
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="yz",
        min_overlap=0.006,
        elem_a=lid_knuckle_left,
        elem_b=body_knuckle_left,
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="yz",
        min_overlap=0.006,
        elem_a=lid_knuckle_right,
        elem_b=body_knuckle_right,
    )
    ctx.expect_overlap(body, lid, axes="xy", min_overlap=0.10, elem_a=scan_glass, elem_b=lid_liner)

    limits = lid_hinge.motion_limits
    assert limits is not None
    assert limits.lower is not None
    assert limits.upper is not None

    with ctx.pose({lid_hinge: limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=lid_shell,
            negative_elem=scan_bezel,
            name="lid_hinge_lower_shell_seat",
        )

    with ctx.pose({lid_hinge: limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.12,
            positive_elem=front_lip,
            negative_elem=scan_bezel,
            name="lid_hinge_upper_front_clearance",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            min_overlap=0.30,
            elem_a=rear_leaf,
            elem_b=rear_hinge_plate,
            name="lid_hinge_upper_hinge_width_alignment",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
