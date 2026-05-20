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


BASE_LENGTH = 1.38
BASE_WIDTH = 1.10
SIDE_FRAME_LENGTH = 1.26
SIDE_FRAME_THICKNESS = 0.10
SIDE_FRAME_HEIGHT = 0.76
SIDE_FRAME_Y = 0.50
SIDE_RAIL_Y = 0.405
SIDE_RAIL_LENGTH = 1.16
SIDE_RAIL_WIDTH = 0.024
SIDE_RAIL_HEIGHT = 0.028
SIDE_RAIL_Z = SIDE_FRAME_HEIGHT + (SIDE_RAIL_HEIGHT / 2.0) - 0.002

SPOILBOARD_LENGTH = 0.98
SPOILBOARD_WIDTH = 0.68
SPOILBOARD_THICKNESS = 0.036
SPOILBOARD_Z = 0.138

BRIDGE_Z = 0.94
BRIDGE_DEPTH = 0.18
BRIDGE_SPAN = 0.84
BRIDGE_HEIGHT = 0.16
BRIDGE_CHEEK_Y = 0.36
BRIDGE_RUNNER_LENGTH = 0.20
BRIDGE_RUNNER_WIDTH = 0.045
BRIDGE_RUNNER_HEIGHT = 0.06
BRIDGE_RUNNER_LOCAL_Z = (
    SIDE_RAIL_Z + (SIDE_RAIL_HEIGHT / 2.0) + (BRIDGE_RUNNER_HEIGHT / 2.0) - BRIDGE_Z
)

CARRIAGE_BODY_X = 0.14
CARRIAGE_BODY_Y = 0.22
CARRIAGE_BODY_Z = 0.24
CARRIAGE_FRONT_X = 0.055
CARRIAGE_FRONT_Y = 0.16
CARRIAGE_FRONT_Z = 0.12
CARRIAGE_MAX_X = 0.14

TOOL_PLATE_X = 0.08
TOOL_PLATE_Y = 0.18
TOOL_PLATE_Z = 0.30
TOOL_CLAMP_X = 0.12
TOOL_CLAMP_Y = 0.12
TOOL_CLAMP_Z = 0.12

BRIDGE_TRAVEL = 0.34
CARRIAGE_TRAVEL = 0.24
TOOL_TRAVEL = 0.20


def _box_wp(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(xyz)


def _side_frame_wall(y_center: float) -> cq.Workplane:
    wall = _box_wp(
        (SIDE_FRAME_LENGTH, SIDE_FRAME_THICKNESS, SIDE_FRAME_HEIGHT),
        (0.0, y_center, SIDE_FRAME_HEIGHT / 2.0),
    )
    for x_center in (-0.33, 0.0, 0.33):
        wall = wall.cut(
            _box_wp(
                (0.20, SIDE_FRAME_THICKNESS + 0.02, 0.42),
                (x_center, y_center, 0.29),
            )
        )
    return wall


def _base_shape():
    shape = _side_frame_wall(SIDE_FRAME_Y).union(_side_frame_wall(-SIDE_FRAME_Y))
    for x_center in (-0.58, 0.0, 0.58):
        beam_depth = 0.10 if abs(x_center) > 0.5 else 0.06
        beam_height = 0.12 if abs(x_center) > 0.5 else 0.10
        shape = shape.union(
            _box_wp(
                (beam_depth, 0.92, beam_height),
                (x_center, 0.0, beam_height / 2.0),
            )
        )
    for y_center in (-0.22, 0.22):
        shape = shape.union(
            _box_wp(
                (1.00, 0.05, 0.06),
                (0.0, y_center, 0.09),
            )
        )
    for x_center in (-0.60, 0.60):
        for y_center in (-0.48, 0.48):
            shape = shape.union(
                _box_wp(
                    (0.16, 0.12, 0.05),
                    (x_center, y_center, 0.025),
                )
            )
    return shape.findSolid()


def _bridge_shape():
    shape = _box_wp((BRIDGE_DEPTH, BRIDGE_SPAN, BRIDGE_HEIGHT), (0.0, 0.0, 0.0))
    for y_center in (-BRIDGE_CHEEK_Y, BRIDGE_CHEEK_Y):
        shape = shape.union(
            _box_wp(
                (0.16, 0.045, 0.28),
                (0.0, y_center, -0.18),
            )
        )
        shape = shape.union(
            _box_wp(
                (0.08, 0.08, 0.10),
                (0.0, y_center * 0.94, -0.08),
            )
        )
    return shape.findSolid()


def _carriage_shape():
    shape = _box_wp((0.12, CARRIAGE_BODY_Y, CARRIAGE_BODY_Z), (0.06, 0.0, 0.0))
    shape = shape.union(_box_wp((0.04, 0.18, 0.32), (0.02, 0.0, -0.04)))
    shape = shape.union(
        _box_wp(
            (CARRIAGE_FRONT_X, CARRIAGE_FRONT_Y, CARRIAGE_FRONT_Z),
            (0.1125, 0.0, -0.08),
        )
    )
    return shape.findSolid()


def _tool_slide_shape():
    shape = _box_wp((TOOL_PLATE_X, TOOL_PLATE_Y, TOOL_PLATE_Z), (0.04, 0.0, -0.15))
    shape = shape.union(_box_wp((0.06, 0.14, 0.08), (0.03, 0.0, -0.04)))
    shape = shape.union(_box_wp((TOOL_CLAMP_X, TOOL_CLAMP_Y, TOOL_CLAMP_Z), (0.07, 0.0, -0.31)))
    return shape.findSolid()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_gantry_router")

    model.material("painted_steel", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("rail_steel", rgba=(0.64, 0.67, 0.72, 1.0))
    model.material("beam_aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    model.material("carriage_black", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("slide_gray", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("spindle_metal", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("spoilboard", rgba=(0.71, 0.58, 0.41, 1.0))

    base = model.part("base_frame")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_frame_body"),
        material="painted_steel",
        name="base_body",
    )
    base.visual(
        Box((SIDE_RAIL_LENGTH, SIDE_RAIL_WIDTH, SIDE_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, SIDE_RAIL_Y, SIDE_RAIL_Z)),
        material="rail_steel",
        name="rail_left",
    )
    base.visual(
        Box((SIDE_RAIL_LENGTH, 0.07, 0.055)),
        origin=Origin(xyz=(0.0, 0.435, 0.7325)),
        material="painted_steel",
        name="left_rail_saddle",
    )
    base.visual(
        Box((SIDE_RAIL_LENGTH, SIDE_RAIL_WIDTH, SIDE_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, -SIDE_RAIL_Y, SIDE_RAIL_Z)),
        material="rail_steel",
        name="rail_right",
    )
    base.visual(
        Box((SIDE_RAIL_LENGTH, 0.07, 0.055)),
        origin=Origin(xyz=(0.0, -0.435, 0.7325)),
        material="painted_steel",
        name="right_rail_saddle",
    )
    base.visual(
        Box((SPOILBOARD_LENGTH, SPOILBOARD_WIDTH, SPOILBOARD_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, SPOILBOARD_Z)),
        material="spoilboard",
        name="spoilboard",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, SIDE_FRAME_HEIGHT)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, SIDE_FRAME_HEIGHT / 2.0)),
    )

    bridge = model.part("bridge")
    bridge.visual(
        mesh_from_cadquery(_bridge_shape(), "bridge_body"),
        material="beam_aluminum",
        name="bridge_body",
    )
    bridge.visual(
        Box((BRIDGE_RUNNER_LENGTH, BRIDGE_RUNNER_WIDTH, BRIDGE_RUNNER_HEIGHT)),
        origin=Origin(xyz=(0.0, SIDE_RAIL_Y, BRIDGE_RUNNER_LOCAL_Z)),
        material="carriage_black",
        name="left_runner",
    )
    bridge.visual(
        Box((BRIDGE_RUNNER_LENGTH, BRIDGE_RUNNER_WIDTH, BRIDGE_RUNNER_HEIGHT)),
        origin=Origin(xyz=(0.0, -SIDE_RAIL_Y, BRIDGE_RUNNER_LOCAL_Z)),
        material="carriage_black",
        name="right_runner",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((BRIDGE_DEPTH, BRIDGE_SPAN, 0.44)),
        mass=46.0,
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage_body"),
        material="carriage_black",
        name="carriage_body",
    )
    carriage.visual(
        Box((0.02, 0.02, 0.34)),
        origin=Origin(xyz=(0.118, 0.045, -0.03)),
        material="rail_steel",
        name="z_rail_left",
    )
    carriage.visual(
        Box((0.02, 0.02, 0.34)),
        origin=Origin(xyz=(0.118, -0.045, -0.03)),
        material="rail_steel",
        name="z_rail_right",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.14, 0.22, 0.38)),
        mass=14.0,
        origin=Origin(xyz=(0.07, 0.0, -0.03)),
    )

    tool_head = model.part("tool_head")
    tool_head.visual(
        mesh_from_cadquery(_tool_slide_shape(), "tool_slide_body"),
        material="slide_gray",
        name="tool_slide_body",
    )
    tool_head.visual(
        Cylinder(radius=0.04, length=0.24),
        origin=Origin(xyz=(0.072, 0.0, -0.44)),
        material="spindle_metal",
        name="spindle_body",
    )
    tool_head.visual(
        Cylinder(radius=0.015, length=0.04),
        origin=Origin(xyz=(0.072, 0.0, -0.58)),
        material="spindle_metal",
        name="collet_nut",
    )
    tool_head.visual(
        Cylinder(radius=0.0035, length=0.11),
        origin=Origin(xyz=(0.072, 0.0, -0.655)),
        material="spindle_metal",
        name="tool_bit",
    )
    tool_head.inertial = Inertial.from_geometry(
        Box((0.14, 0.18, 0.78)),
        mass=16.0,
        origin=Origin(xyz=(0.07, 0.0, -0.35)),
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-BRIDGE_TRAVEL,
            upper=BRIDGE_TRAVEL,
            effort=900.0,
            velocity=0.75,
        ),
    )
    model.articulation(
        "bridge_to_carriage",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=carriage,
        origin=Origin(xyz=(BRIDGE_DEPTH / 2.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-CARRIAGE_TRAVEL,
            upper=CARRIAGE_TRAVEL,
            effort=320.0,
            velocity=0.60,
        ),
    )
    model.articulation(
        "carriage_to_tool",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=tool_head,
        origin=Origin(xyz=(CARRIAGE_MAX_X, 0.0, 0.15)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TOOL_TRAVEL,
            effort=240.0,
            velocity=0.40,
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

    base = object_model.get_part("base_frame")
    bridge = object_model.get_part("bridge")
    carriage = object_model.get_part("carriage")
    tool_head = object_model.get_part("tool_head")

    bridge_slide = object_model.get_articulation("base_to_bridge")
    carriage_slide = object_model.get_articulation("bridge_to_carriage")
    tool_slide = object_model.get_articulation("carriage_to_tool")

    ctx.expect_overlap(
        bridge,
        base,
        axes="x",
        elem_a="left_runner",
        elem_b="rail_left",
        min_overlap=0.18,
        name="left bridge runner stays on the left base rail",
    )
    ctx.expect_overlap(
        bridge,
        base,
        axes="x",
        elem_a="right_runner",
        elem_b="rail_right",
        min_overlap=0.18,
        name="right bridge runner stays on the right base rail",
    )
    ctx.expect_gap(
        bridge,
        base,
        axis="z",
        positive_elem="left_runner",
        negative_elem="rail_left",
        min_gap=0.0,
        max_gap=0.003,
        name="left bridge runner sits on the rail crest",
    )
    ctx.expect_gap(
        carriage,
        bridge,
        axis="x",
        negative_elem="bridge_body",
        min_gap=0.0,
        max_gap=0.003,
        name="carriage mounts flush to the bridge face",
    )
    ctx.expect_gap(
        tool_head,
        carriage,
        axis="x",
        positive_elem="tool_slide_body",
        negative_elem="carriage_body",
        min_gap=0.0,
        max_gap=0.003,
        name="tool slide mounts flush to the carriage face",
    )
    ctx.expect_gap(
        tool_head,
        base,
        axis="z",
        positive_elem="tool_bit",
        negative_elem="spoilboard",
        min_gap=0.20,
        name="tool bit starts safely above the spoilboard",
    )

    rest_bridge = ctx.part_world_position(bridge)
    with ctx.pose({bridge_slide: BRIDGE_TRAVEL}):
        ctx.expect_overlap(
            bridge,
            base,
            axes="x",
            elem_a="left_runner",
            elem_b="rail_left",
            min_overlap=0.18,
            name="left bridge runner remains captured at max X travel",
        )
        moved_bridge = ctx.part_world_position(bridge)
    ctx.check(
        "bridge moves forward along the base rails",
        rest_bridge is not None
        and moved_bridge is not None
        and moved_bridge[0] > rest_bridge[0] + 0.20
        and abs(moved_bridge[1] - rest_bridge[1]) < 1e-6,
        details=f"rest={rest_bridge}, moved={moved_bridge}",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({carriage_slide: CARRIAGE_TRAVEL}):
        moved_carriage = ctx.part_world_position(carriage)
    ctx.check(
        "carriage moves laterally across the bridge beam",
        rest_carriage is not None
        and moved_carriage is not None
        and moved_carriage[1] > rest_carriage[1] + 0.15
        and abs(moved_carriage[0] - rest_carriage[0]) < 1e-6,
        details=f"rest={rest_carriage}, moved={moved_carriage}",
    )

    rest_tool = ctx.part_world_position(tool_head)
    with ctx.pose({tool_slide: TOOL_TRAVEL}):
        ctx.expect_gap(
            tool_head,
            base,
            axis="z",
            positive_elem="tool_bit",
            negative_elem="spoilboard",
            min_gap=0.02,
            name="tool bit still clears the spoilboard at full drop",
        )
        dropped_tool = ctx.part_world_position(tool_head)
    ctx.check(
        "tool head descends when the Z slide extends",
        rest_tool is not None
        and dropped_tool is not None
        and dropped_tool[2] < rest_tool[2] - 0.10,
        details=f"rest={rest_tool}, dropped={dropped_tool}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
