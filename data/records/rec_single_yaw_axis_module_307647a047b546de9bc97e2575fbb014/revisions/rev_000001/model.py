from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_DEPTH = 0.110
BRIDGE_DEPTH = 0.080
FRAME_HEIGHT = 0.150
PLATE_THICKNESS = 0.010
INNER_GAP = 0.074
OUTER_WIDTH = INNER_GAP + (2.0 * PLATE_THICKNESS)
PLATE_CENTER_Y = (INNER_GAP / 2.0) + (PLATE_THICKNESS / 2.0)

BASE_THICKNESS = 0.014
TOP_THICKNESS = 0.014
BOSS_HEIGHT = 0.016
BOSS_OUTER_RADIUS = 0.021
AXIS_Z = FRAME_HEIGHT / 2.0

WINDOW_X = 0.064
WINDOW_Z = 0.068
WINDOW_FILLET = 0.010

BLOCK_X = 0.054
BLOCK_Y = 0.044
BLOCK_Z = 0.034
CAP_RADIUS = 0.020

TAB_X = 0.023
TAB_LENGTH = 0.020
TAB_WIDTH = 0.014
TAB_THICKNESS = 0.006


def _side_plate(y_center: float) -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(FRAME_DEPTH, PLATE_THICKNESS, FRAME_HEIGHT)
        .translate((0.0, y_center, FRAME_HEIGHT / 2.0))
    )
    window = (
        cq.Workplane("XY")
        .box(WINDOW_X, PLATE_THICKNESS + 0.004, WINDOW_Z)
        .edges("|Y")
        .fillet(WINDOW_FILLET)
        .translate((0.0, y_center, AXIS_Z))
    )
    return plate.cut(window)


def _lower_support() -> cq.Workplane:
    base_bridge = (
        cq.Workplane("XY")
        .box(BRIDGE_DEPTH, OUTER_WIDTH, BASE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
    )
    lower_boss = (
        cq.Workplane("XY", origin=(0.0, 0.0, BASE_THICKNESS))
        .circle(BOSS_OUTER_RADIUS)
        .extrude(BOSS_HEIGHT)
    )
    return base_bridge.union(lower_boss)


def _upper_support() -> cq.Workplane:
    top_bridge = (
        cq.Workplane("XY")
        .box(BRIDGE_DEPTH, OUTER_WIDTH, TOP_THICKNESS)
        .translate((0.0, 0.0, FRAME_HEIGHT - (TOP_THICKNESS / 2.0)))
    )
    upper_boss = (
        cq.Workplane("XY", origin=(0.0, 0.0, FRAME_HEIGHT - TOP_THICKNESS - BOSS_HEIGHT))
        .circle(BOSS_OUTER_RADIUS)
        .extrude(BOSS_HEIGHT)
    )
    return top_bridge.union(upper_boss)


def _top_block_main() -> cq.Workplane:
    body = cq.Workplane("XY").box(BLOCK_X, BLOCK_Y, BLOCK_Z).edges("|Z").fillet(0.004)
    hub = cq.Workplane("XY", origin=(0.0, 0.0, -0.045)).circle(CAP_RADIUS).extrude(0.090)
    return body.union(hub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_plate_yaw_carriage")

    frame_material = model.material("frame_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    block_material = model.material("block_steel", rgba=(0.23, 0.25, 0.29, 1.0))
    accent_material = model.material("index_accent", rgba=(0.85, 0.38, 0.12, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_side_plate(PLATE_CENTER_Y), "frame_left_plate"),
        material=frame_material,
        name="left_plate",
    )
    frame.visual(
        mesh_from_cadquery(_side_plate(-PLATE_CENTER_Y), "frame_right_plate"),
        material=frame_material,
        name="right_plate",
    )
    frame.visual(
        mesh_from_cadquery(_lower_support(), "frame_lower_support"),
        material=frame_material,
        name="lower_support",
    )
    frame.visual(
        mesh_from_cadquery(_upper_support(), "frame_upper_support"),
        material=frame_material,
        name="upper_support",
    )

    top_block = model.part("top_block")
    top_block.visual(
        mesh_from_cadquery(_top_block_main(), "top_block_main"),
        material=block_material,
        name="main_body",
    )
    top_block.visual(
        Box((TAB_LENGTH, TAB_WIDTH, TAB_THICKNESS)),
        origin=Origin(xyz=(TAB_X, 0.0, (BLOCK_Z / 2.0) + (TAB_THICKNESS / 2.0))),
        material=accent_material,
        name="indicator_tab",
    )

    model.articulation(
        "frame_to_top_block",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=top_block,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-2.7,
            upper=2.7,
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

    frame = object_model.get_part("frame")
    top_block = object_model.get_part("top_block")
    yaw = object_model.get_articulation("frame_to_top_block")

    ctx.check(
        "yaw axis is vertical",
        tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw.axis}",
    )

    with ctx.pose({yaw: 0.0}):
        ctx.expect_gap(
            frame,
            top_block,
            axis="y",
            positive_elem="left_plate",
            negative_elem="main_body",
            min_gap=0.008,
            name="left plate clears top block at rest",
        )
        ctx.expect_gap(
            top_block,
            frame,
            axis="y",
            positive_elem="main_body",
            negative_elem="right_plate",
            min_gap=0.008,
            name="right plate clears top block at rest",
        )
        ctx.expect_within(
            top_block,
            frame,
            axes="x",
            inner_elem="main_body",
            outer_elem="lower_support",
            margin=0.0,
            name="top block stays within the bridge depth at rest",
        )

    with ctx.pose({yaw: 1.0}):
        ctx.expect_gap(
            frame,
            top_block,
            axis="y",
            positive_elem="left_plate",
            negative_elem="main_body",
            min_gap=0.001,
            name="left plate clears top block in turned pose",
        )
        ctx.expect_gap(
            top_block,
            frame,
            axis="y",
            positive_elem="main_body",
            negative_elem="right_plate",
            min_gap=0.001,
            name="right plate clears top block in turned pose",
        )
        ctx.expect_within(
            top_block,
            frame,
            axes="x",
            inner_elem="main_body",
            outer_elem="lower_support",
            margin=0.0,
            name="top block stays within the bridge depth in turned pose",
        )

    rest_aabb = ctx.part_element_world_aabb(top_block, elem="indicator_tab")
    with ctx.pose({yaw: 1.0}):
        turned_aabb = ctx.part_element_world_aabb(top_block, elem="indicator_tab")

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) / 2.0 for i in range(3))

    rest_center = _center_from_aabb(rest_aabb)
    turned_center = _center_from_aabb(turned_aabb)
    ctx.check(
        "positive yaw swings the indicator toward +y",
        rest_center is not None
        and turned_center is not None
        and turned_center[1] > rest_center[1] + 0.015
        and turned_center[0] < rest_center[0] - 0.008,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
