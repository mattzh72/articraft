from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_X = 0.24
BASE_Y = 0.18
BASE_Z = 0.028

CHEEK_X = 0.052
CHEEK_Y = 0.094
CHEEK_Z = 0.084
CHEEK_CENTER_X = 0.072
CHEEK_CENTER_Y = -0.028

CARTRIDGE_RADIUS = 0.034
CARTRIDGE_LENGTH = 0.082
CARTRIDGE_CENTER_Y = -0.021
CARTRIDGE_CENTER_Z = 0.094

JOINT_Y = CARTRIDGE_CENTER_Y + CARTRIDGE_LENGTH / 2.0
JOINT_Z = CARTRIDGE_CENTER_Z

OUTPUT_HUB_RADIUS = 0.022
OUTPUT_HUB_LENGTH = 0.024
FACE_X = 0.090
FACE_Y = 0.016
FACE_Z = 0.090


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def _housing_shell_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_X, BASE_Y, BASE_Z)
    saddle_block = (
        cq.Workplane("XY")
        .box(0.188, 0.118, 0.080)
        .translate((0.0, -0.030, BASE_Z / 2.0 + 0.040))
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.128, 0.052, 0.034)
        .translate((0.0, -0.050, 0.070))
    )
    shell = base.union(saddle_block).union(rear_bridge)

    center_window = (
        cq.Workplane("XY")
        .box(0.084, 0.122, 0.058)
        .translate((0.0, -0.010, 0.082))
    )
    front_mouth = (
        cq.Workplane("XY")
        .box(0.112, 0.056, 0.034)
        .translate((0.0, 0.014, 0.060))
    )
    cartridge_relief = (
        cq.Workplane("XZ")
        .circle(0.041)
        .extrude(0.104)
        .translate((0.0, 0.052, CARTRIDGE_CENTER_Z))
    )
    shell = shell.cut(center_window).cut(front_mouth).cut(cartridge_relief)
    return shell


def _output_shell_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("XZ")
        .circle(OUTPUT_HUB_RADIUS)
        .extrude(OUTPUT_HUB_LENGTH)
        .translate((0.0, OUTPUT_HUB_LENGTH, 0.0))
    )
    shoulder = (
        cq.Workplane("XZ")
        .circle(0.031)
        .extrude(0.010)
        .translate((0.0, 0.028, 0.0))
    )
    faceplate = (
        cq.Workplane("XY")
        .box(FACE_X, FACE_Y, FACE_Z)
        .edges("|Y")
        .fillet(0.010)
        .translate((0.0, OUTPUT_HUB_LENGTH + FACE_Y / 2.0, 0.0))
    )
    pilot = (
        cq.Workplane("XZ")
        .circle(0.018)
        .extrude(0.008)
        .translate((0.0, OUTPUT_HUB_LENGTH + FACE_Y, 0.0))
    )
    return hub.union(shoulder).union(faceplate).union(pilot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="saddle_body_roll_module")

    model.material("housing_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("cartridge_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("output_black", rgba=(0.13, 0.14, 0.15, 1.0))
    model.material("indicator_red", rgba=(0.73, 0.16, 0.12, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shell_shape(), "housing_shell"),
        material="housing_gray",
        name="housing_shell",
    )
    housing.visual(
        Cylinder(radius=CARTRIDGE_RADIUS, length=CARTRIDGE_LENGTH),
        origin=Origin(
            xyz=(0.0, CARTRIDGE_CENTER_Y, CARTRIDGE_CENTER_Z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="cartridge_gray",
        name="cartridge",
    )
    housing.visual(
        Cylinder(radius=0.041, length=0.012),
        origin=Origin(
            xyz=(0.0, JOINT_Y - 0.006, CARTRIDGE_CENTER_Z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="cartridge_gray",
        name="cartridge_face",
    )
    housing.visual(
        Box((0.132, 0.016, 0.024)),
        origin=Origin(xyz=(0.0, -0.050, 0.082)),
        material="housing_gray",
        name="cartridge_saddle",
    )
    housing.inertial = Inertial.from_geometry(
        Box((BASE_X, BASE_Y, 0.132)),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
    )

    output_face = model.part("output_face")
    output_face.visual(
        mesh_from_cadquery(_output_shell_shape(), "output_shell"),
        material="output_black",
        name="output_shell",
    )
    output_face.visual(
        Box((0.016, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, OUTPUT_HUB_LENGTH + FACE_Y - 0.001, 0.046)),
        material="indicator_red",
        name="indicator_tab",
    )
    output_face.inertial = Inertial.from_geometry(
        Box((FACE_X, OUTPUT_HUB_LENGTH + FACE_Y, FACE_Z)),
        mass=0.7,
        origin=Origin(xyz=(0.0, (OUTPUT_HUB_LENGTH + FACE_Y) / 2.0, 0.0)),
    )

    model.articulation(
        "roll_output",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=output_face,
        origin=Origin(xyz=(0.0, JOINT_Y, JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=3.2,
            lower=-2.8,
            upper=2.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    housing = object_model.get_part("housing")
    output_face = object_model.get_part("output_face")
    roll_output = object_model.get_articulation("roll_output")

    limits = roll_output.motion_limits
    ctx.check(
        "roll axis is supported lateral axis",
        tuple(round(v, 6) for v in roll_output.axis) == (0.0, 1.0, 0.0),
        details=f"axis={roll_output.axis}",
    )
    ctx.check(
        "roll travel is substantial",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -2.5
        and limits.upper >= 2.5,
        details=f"limits={limits}",
    )

    with ctx.pose({roll_output: 0.0}):
        ctx.expect_gap(
            output_face,
            housing,
            axis="y",
            min_gap=0.0,
            max_gap=0.001,
            negative_elem="cartridge",
            name="output face seats at cartridge front plane",
        )
        ctx.expect_overlap(
            output_face,
            housing,
            axes="xz",
            min_overlap=0.050,
            name="output remains centered on the housing support",
        )

        housing_aabb = ctx.part_world_aabb(housing)
        output_aabb = ctx.part_world_aabb(output_face)
        if housing_aabb is not None and output_aabb is not None:
            housing_size = tuple(hi - lo for lo, hi in zip(*housing_aabb))
            output_size = tuple(hi - lo for lo, hi in zip(*output_aabb))
            ctx.check(
                "grounded body is larger than moving member",
                housing_size[0] > output_size[0] and housing_size[2] > output_size[2],
                details=f"housing_size={housing_size}, output_size={output_size}",
            )
        else:
            ctx.fail("grounded body is larger than moving member", "missing part AABB")

        rest_tab_center = _aabb_center(
            ctx.part_element_world_aabb(output_face, elem="indicator_tab")
        )

    with ctx.pose({roll_output: 1.2}):
        moved_tab_center = _aabb_center(
            ctx.part_element_world_aabb(output_face, elem="indicator_tab")
        )

    ctx.check(
        "positive roll rotates the indicator forward in +x",
        rest_tab_center is not None
        and moved_tab_center is not None
        and moved_tab_center[0] > rest_tab_center[0] + 0.020,
        details=f"rest={rest_tab_center}, moved={moved_tab_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
