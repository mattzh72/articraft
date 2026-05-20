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


FRAME_INNER_SPAN = 0.160
SIDE_PLATE_THICKNESS = 0.012
BOSS_EXTENSION = 0.014
FRAME_DEPTH = 0.090
FRAME_HEIGHT = 0.110
FOOT_HEIGHT = 0.012
FOOT_DEPTH = 0.110
BASE_RAIL_DEPTH = 0.062
BASE_RAIL_HEIGHT = 0.020
TOP_BRIDGE_DEPTH = 0.024
TOP_BRIDGE_HEIGHT = 0.016
SHAFT_AXIS_Z = 0.078

SHAFT_RADIUS = 0.012
SHAFT_LENGTH = 0.290
SHAFT_CLEARANCE_RADIUS = 0.0135
BOSS_RADIUS = 0.032
BEARING_POCKET_RADIUS = 0.022
BEARING_PAD_SIZE = (0.010, 0.018, 0.012)

COLLAR_RADIUS = 0.026
COLLAR_WIDTH = 0.018
COLLAR_CENTER_X = 0.120
CLAMP_LUG_SIZE = (0.014, 0.012, 0.010)
CLAMP_LUG_CENTER = (
    COLLAR_CENTER_X,
    0.0,
    COLLAR_RADIUS + CLAMP_LUG_SIZE[2] / 2.0 - 0.004,
)


def _box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_x(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True).translate((cx, cy, cz))


def _make_side_support(sign: float) -> cq.Workplane:
    plate_center_x = FRAME_INNER_SPAN / 2.0 + SIDE_PLATE_THICKNESS / 2.0
    plate_x = sign * plate_center_x
    boss_center_x = sign * (plate_center_x + SIDE_PLATE_THICKNESS / 2.0 + BOSS_EXTENSION / 2.0)
    bore_center_x = sign * (plate_center_x + BOSS_EXTENSION / 2.0)

    support = (
        _box_at(
            (SIDE_PLATE_THICKNESS, FRAME_DEPTH, FRAME_HEIGHT),
            (plate_x, 0.0, FRAME_HEIGHT / 2.0),
        )
        .union(
            _box_at(
                (SIDE_PLATE_THICKNESS, FOOT_DEPTH, FOOT_HEIGHT),
                (plate_x, 0.0, FOOT_HEIGHT / 2.0),
            )
        )
        .union(_cylinder_x(BOSS_RADIUS, BOSS_EXTENSION, (boss_center_x, 0.0, SHAFT_AXIS_Z)))
    )
    return support.cut(
        _cylinder_x(
            SHAFT_CLEARANCE_RADIUS,
            SIDE_PLATE_THICKNESS + BOSS_EXTENSION + 0.010,
            (bore_center_x, 0.0, SHAFT_AXIS_Z),
        )
    )


def _make_shaft_body_shape() -> cq.Workplane:
    return _cylinder_x(SHAFT_RADIUS, SHAFT_LENGTH, (0.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="through_shaft_rotary_module")

    model.material("frame_paint", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("collar_black", rgba=(0.16, 0.17, 0.19, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_make_side_support(-1.0), "left_support"),
        material="frame_paint",
        name="left_support",
    )
    frame.visual(
        mesh_from_cadquery(_make_side_support(1.0), "right_support"),
        material="frame_paint",
        name="right_support",
    )
    frame.visual(
        Box((FRAME_INNER_SPAN + 2.0 * SIDE_PLATE_THICKNESS, BASE_RAIL_DEPTH, BASE_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_RAIL_HEIGHT / 2.0)),
        material="frame_paint",
        name="base_rail",
    )
    frame.visual(
        Box((FRAME_INNER_SPAN + 2.0 * SIDE_PLATE_THICKNESS, TOP_BRIDGE_DEPTH, TOP_BRIDGE_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.028, FRAME_HEIGHT - TOP_BRIDGE_HEIGHT / 2.0)),
        material="frame_paint",
        name="top_bridge",
    )
    frame.visual(
        Box(BEARING_PAD_SIZE),
        origin=Origin(
            xyz=(
                -(FRAME_INNER_SPAN / 2.0 + SIDE_PLATE_THICKNESS / 2.0 + BOSS_EXTENSION / 2.0),
                0.0,
                SHAFT_AXIS_Z - SHAFT_RADIUS - BEARING_PAD_SIZE[2] / 2.0,
            )
        ),
        material="steel",
        name="left_bearing_pad",
    )
    frame.visual(
        Box(BEARING_PAD_SIZE),
        origin=Origin(
            xyz=(
                FRAME_INNER_SPAN / 2.0 + SIDE_PLATE_THICKNESS / 2.0 + BOSS_EXTENSION / 2.0,
                0.0,
                SHAFT_AXIS_Z - SHAFT_RADIUS - BEARING_PAD_SIZE[2] / 2.0,
            )
        ),
        material="steel",
        name="right_bearing_pad",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_INNER_SPAN + 2.0 * (SIDE_PLATE_THICKNESS + BOSS_EXTENSION), FOOT_DEPTH, FRAME_HEIGHT)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0)),
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="steel",
        name="shaft_body",
    )
    shaft.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_WIDTH),
        origin=Origin(xyz=(COLLAR_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="collar_black",
        name="output_collar",
    )
    shaft.visual(
        Box(CLAMP_LUG_SIZE),
        origin=Origin(xyz=CLAMP_LUG_CENTER),
        material="collar_black",
        name="collar_clamp_lug",
    )
    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        mass=0.9,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "frame_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
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

    frame = object_model.get_part("frame")
    shaft = object_model.get_part("shaft")
    spin = object_model.get_articulation("frame_to_shaft")

    limits = spin.motion_limits
    ctx.check(
        "shaft uses a continuous x-axis articulation",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in spin.axis) == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=f"type={spin.articulation_type}, axis={spin.axis}, limits={limits}",
    )

    ctx.expect_overlap(
        shaft,
        frame,
        axes="x",
        elem_a="shaft_body",
        min_overlap=0.180,
        name="shaft spans the full bearing frame width",
    )
    ctx.expect_overlap(
        shaft,
        frame,
        axes="yz",
        elem_a="shaft_body",
        min_overlap=0.020,
        name="shaft is aligned with the bearing axis through the frame",
    )
    ctx.expect_gap(
        shaft,
        frame,
        axis="x",
        positive_elem="output_collar",
        negative_elem="right_support",
        min_gap=0.003,
        max_gap=0.011,
        name="output collar sits just outside the right bearing housing",
    )
    ctx.expect_contact(
        shaft,
        frame,
        elem_a="shaft_body",
        elem_b="left_bearing_pad",
        name="left bearing pad physically supports the shaft",
    )
    ctx.expect_contact(
        shaft,
        frame,
        elem_a="shaft_body",
        elem_b="right_bearing_pad",
        name="right bearing pad physically supports the shaft",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    shaft_aabb = ctx.part_element_world_aabb(shaft, elem="shaft_body")
    protrudes_both_sides = (
        frame_aabb is not None
        and shaft_aabb is not None
        and shaft_aabb[0][0] < frame_aabb[0][0] - 0.030
        and shaft_aabb[1][0] > frame_aabb[1][0] + 0.030
    )
    ctx.check(
        "through shaft protrudes beyond both side bearings",
        protrudes_both_sides,
        details=f"frame_aabb={frame_aabb}, shaft_aabb={shaft_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
