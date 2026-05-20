from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
)


BODY_OUTER_WIDTH = 0.58
BODY_OUTER_DEPTH = 0.72
BODY_INNER_WIDTH = 0.54
BODY_INNER_DEPTH = 0.68
BODY_HEIGHT = 0.86
WALL_THICKNESS = 0.012
RIM_THICKNESS = 0.018
LIP_HEIGHT = 0.045
WHEEL_RADIUS = 0.125
WHEEL_WIDTH = 0.048
AXLE_HEIGHT = 0.145
AXLE_TRACK = 0.60


def _add_body_shell(part, *, shell, trim, hardware) -> None:
    floor_width = BODY_OUTER_WIDTH - 2.0 * WALL_THICKNESS
    floor_depth = BODY_OUTER_DEPTH - 2.0 * (WALL_THICKNESS + 0.01)
    floor_thickness = 0.020
    floor_top = floor_thickness

    part.visual(
        Box((floor_width, floor_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.01, floor_thickness * 0.5)),
        material=shell,
        name="floor",
    )

    part.visual(
        Box((BODY_OUTER_WIDTH, WALL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, BODY_OUTER_DEPTH * 0.5 - WALL_THICKNESS * 0.5, BODY_HEIGHT * 0.5)),
        material=shell,
        name="front_wall",
    )
    part.visual(
        Box((BODY_OUTER_WIDTH - 0.03, WALL_THICKNESS, BODY_HEIGHT - 0.03)),
        origin=Origin(xyz=(0.0, -BODY_OUTER_DEPTH * 0.5 + WALL_THICKNESS * 0.5 + 0.02, (BODY_HEIGHT - 0.03) * 0.5 + 0.02)),
        material=shell,
        name="rear_wall",
    )
    part.visual(
        Box((WALL_THICKNESS, BODY_OUTER_DEPTH - 2.0 * (WALL_THICKNESS + 0.01), BODY_HEIGHT - 0.02)),
        origin=Origin(xyz=(BODY_OUTER_WIDTH * 0.5 - WALL_THICKNESS * 0.5, 0.01, (BODY_HEIGHT - 0.02) * 0.5 + 0.01)),
        material=shell,
        name="left_wall",
    )
    part.visual(
        Box((WALL_THICKNESS, BODY_OUTER_DEPTH - 2.0 * (WALL_THICKNESS + 0.01), BODY_HEIGHT - 0.02)),
        origin=Origin(xyz=(-BODY_OUTER_WIDTH * 0.5 + WALL_THICKNESS * 0.5, 0.01, (BODY_HEIGHT - 0.02) * 0.5 + 0.01)),
        material=shell,
        name="right_wall",
    )

    rim_z = BODY_HEIGHT - RIM_THICKNESS * 0.5
    part.visual(
        Box((BODY_OUTER_WIDTH, RIM_THICKNESS, RIM_THICKNESS)),
        origin=Origin(xyz=(0.0, BODY_OUTER_DEPTH * 0.5 - RIM_THICKNESS * 0.5, rim_z)),
        material=trim,
        name="front_rim",
    )
    part.visual(
        Box((BODY_OUTER_WIDTH - 0.04, RIM_THICKNESS, RIM_THICKNESS)),
        origin=Origin(xyz=(0.0, -BODY_OUTER_DEPTH * 0.5 + 0.04, rim_z)),
        material=trim,
        name="rear_rim",
    )
    part.visual(
        Box((RIM_THICKNESS, BODY_OUTER_DEPTH - 0.08, RIM_THICKNESS)),
        origin=Origin(xyz=(BODY_OUTER_WIDTH * 0.5 - RIM_THICKNESS * 0.5, 0.01, rim_z)),
        material=trim,
        name="left_rim",
    )
    part.visual(
        Box((RIM_THICKNESS, BODY_OUTER_DEPTH - 0.08, RIM_THICKNESS)),
        origin=Origin(xyz=(-BODY_OUTER_WIDTH * 0.5 + RIM_THICKNESS * 0.5, 0.01, rim_z)),
        material=trim,
        name="right_rim",
    )

    lip_y = BODY_OUTER_DEPTH * 0.5 - RIM_THICKNESS - WALL_THICKNESS * 0.5
    lip_z = BODY_HEIGHT - LIP_HEIGHT * 0.5 - RIM_THICKNESS
    part.visual(
        Box((BODY_INNER_WIDTH - 0.04, WALL_THICKNESS, LIP_HEIGHT)),
        origin=Origin(xyz=(0.0, lip_y, lip_z)),
        material=trim,
        name="stop_lip_front",
    )
    part.visual(
        Box((WALL_THICKNESS, BODY_INNER_DEPTH - 0.10, LIP_HEIGHT)),
        origin=Origin(xyz=(BODY_OUTER_WIDTH * 0.5 - RIM_THICKNESS - WALL_THICKNESS * 0.5, 0.01, lip_z)),
        material=trim,
        name="stop_lip_left",
    )
    part.visual(
        Box((WALL_THICKNESS, BODY_INNER_DEPTH - 0.10, LIP_HEIGHT)),
        origin=Origin(xyz=(-BODY_OUTER_WIDTH * 0.5 + RIM_THICKNESS + WALL_THICKNESS * 0.5, 0.01, lip_z)),
        material=trim,
        name="stop_lip_right",
    )

    handle_bar_z = BODY_HEIGHT - 0.09
    part.visual(
        Box((0.26, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, -BODY_OUTER_DEPTH * 0.5 + 0.005, handle_bar_z)),
        material=hardware,
        name="rear_handle_grip",
    )
    part.visual(
        Box((0.040, 0.030, 0.18)),
        origin=Origin(xyz=(0.12, -BODY_OUTER_DEPTH * 0.5 + 0.005, handle_bar_z - 0.09)),
        material=hardware,
        name="rear_handle_left_post",
    )
    part.visual(
        Box((0.040, 0.030, 0.18)),
        origin=Origin(xyz=(-0.12, -BODY_OUTER_DEPTH * 0.5 + 0.005, handle_bar_z - 0.09)),
        material=hardware,
        name="rear_handle_right_post",
    )

    part.visual(
        Box((0.09, 0.10, 0.09)),
        origin=Origin(xyz=(0.21, -BODY_OUTER_DEPTH * 0.5 + 0.05, 0.08)),
        material=shell,
        name="left_wheel_pod",
    )
    part.visual(
        Box((0.09, 0.10, 0.09)),
        origin=Origin(xyz=(-0.21, -BODY_OUTER_DEPTH * 0.5 + 0.05, 0.08)),
        material=shell,
        name="right_wheel_pod",
    )

    axle_origin = Origin(xyz=(0.0, -BODY_OUTER_DEPTH * 0.5 + 0.045, AXLE_HEIGHT), rpy=(0.0, pi * 0.5, 0.0))
    part.visual(
        Cylinder(radius=0.016, length=AXLE_TRACK - 0.05),
        origin=axle_origin,
        material=hardware,
        name="axle_beam",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.035),
        origin=Origin(
            xyz=(AXLE_TRACK * 0.5 - 0.0175, -BODY_OUTER_DEPTH * 0.5 + 0.045, AXLE_HEIGHT),
            rpy=(0.0, pi * 0.5, 0.0),
        ),
        material=hardware,
        name="left_axle_stub",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.035),
        origin=Origin(
            xyz=(-AXLE_TRACK * 0.5 + 0.0175, -BODY_OUTER_DEPTH * 0.5 + 0.045, AXLE_HEIGHT),
            rpy=(0.0, pi * 0.5, 0.0),
        ),
        material=hardware,
        name="right_axle_stub",
    )

    part.visual(
        Box((0.42, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, -BODY_OUTER_DEPTH * 0.5 + 0.055, BODY_HEIGHT - 0.055)),
        material=hardware,
        name="hinge_rail",
    )


def _add_lid_visuals(part, *, lid_shell, hardware) -> None:
    lid_width = BODY_OUTER_WIDTH + 0.02
    lid_depth = BODY_OUTER_DEPTH + 0.03
    panel_thickness = 0.018
    panel_z = 0.046

    part.visual(
        Box((lid_width, lid_depth, panel_thickness)),
        origin=Origin(xyz=(0.0, lid_depth * 0.5, panel_z)),
        material=lid_shell,
        name="lid_panel",
    )
    part.visual(
        Box((0.50, 0.034, 0.050)),
        origin=Origin(xyz=(0.0, 0.017, 0.025)),
        material=hardware,
        name="rear_hinge_flange",
    )
    part.visual(
        Box((lid_width, 0.028, 0.056)),
        origin=Origin(xyz=(0.0, lid_depth - 0.014, 0.008)),
        material=lid_shell,
        name="front_skirt",
    )
    part.visual(
        Box((0.028, lid_depth - 0.05, 0.052)),
        origin=Origin(xyz=(lid_width * 0.5 + 0.014, lid_depth * 0.5 + 0.01, 0.010)),
        material=lid_shell,
        name="left_skirt",
    )
    part.visual(
        Box((0.028, lid_depth - 0.05, 0.052)),
        origin=Origin(xyz=(-lid_width * 0.5 - 0.014, lid_depth * 0.5 + 0.01, 0.010)),
        material=lid_shell,
        name="right_skirt",
    )
    part.visual(
        Box((0.18, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, lid_depth - 0.010, 0.028)),
        material=hardware,
        name="front_grip",
    )


def _add_wheel_visuals(part, *, side_sign: float, tire, hub) -> None:
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(xyz=(side_sign * WHEEL_WIDTH * 0.5, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=tire,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.056, length=0.030),
        origin=Origin(xyz=(side_sign * 0.015, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=hub,
        name="inner_hub",
    )
    part.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(side_sign * 0.037, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=hub,
        name="outer_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelie_bin")

    shell = model.material("shell_grey", rgba=(0.25, 0.28, 0.26, 1.0))
    trim = model.material("rim_grey", rgba=(0.20, 0.22, 0.21, 1.0))
    hardware = model.material("hardware_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    hub_dark = model.material("hub_dark", rgba=(0.14, 0.15, 0.16, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((BODY_OUTER_WIDTH, BODY_OUTER_DEPTH, BODY_HEIGHT)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )
    _add_body_shell(body, shell=shell, trim=trim, hardware=hardware)

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((BODY_OUTER_WIDTH + 0.02, BODY_OUTER_DEPTH + 0.03, 0.10)),
        mass=2.2,
        origin=Origin(xyz=(0.0, (BODY_OUTER_DEPTH + 0.03) * 0.5, 0.04)),
    )
    _add_lid_visuals(lid, lid_shell=shell, hardware=hardware)

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=1.6,
        origin=Origin(xyz=(WHEEL_WIDTH * 0.5, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
    )
    _add_wheel_visuals(left_wheel, side_sign=1.0, tire=wheel_rubber, hub=hub_dark)

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=1.6,
        origin=Origin(xyz=(-WHEEL_WIDTH * 0.5, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
    )
    _add_wheel_visuals(right_wheel, side_sign=-1.0, tire=wheel_rubber, hub=hub_dark)

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -BODY_OUTER_DEPTH * 0.5 + 0.055, BODY_HEIGHT - 0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(AXLE_TRACK * 0.5, -BODY_OUTER_DEPTH * 0.5 + 0.045, AXLE_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(-AXLE_TRACK * 0.5, -BODY_OUTER_DEPTH * 0.5 + 0.045, AXLE_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
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
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid_hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_rim",
            max_gap=0.012,
            max_penetration=0.0,
            name="closed lid panel stays just above the front rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            elem_a="lid_panel",
            elem_b="front_rim",
            min_overlap=BODY_OUTER_WIDTH - 0.04,
            name="lid spans the full bin width",
        )
        ctx.expect_contact(
            left_wheel,
            body,
            elem_a="inner_hub",
            elem_b="left_axle_stub",
            name="left wheel seats on the axle stub",
        )
        ctx.expect_contact(
            right_wheel,
            body,
            elem_a="inner_hub",
            elem_b="right_axle_stub",
            name="right wheel seats on the axle stub",
        )

    with ctx.pose({lid_hinge: 1.10}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_skirt",
            negative_elem="front_rim",
            min_gap=0.20,
            name="open lid lifts the front edge clear of the rim",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
