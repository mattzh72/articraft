from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_WIDTH = 0.92
BODY_HEIGHT = 0.66
BODY_DEPTH = 0.24

SIDE_WALL_THICKNESS = 0.05
TOP_BOTTOM_THICKNESS = 0.06
BACK_THICKNESS = 0.03
FRAME_THICKNESS = 0.016

OPENING_WIDTH = 0.48
OPENING_HEIGHT = 0.40
OPENING_CENTER_X = -0.08
OPENING_CENTER_Z = 0.0

OUTER_LEFT_X = -BODY_WIDTH / 2.0
OUTER_RIGHT_X = BODY_WIDTH / 2.0
OPENING_LEFT_X = OPENING_CENTER_X - OPENING_WIDTH / 2.0
OPENING_RIGHT_X = OPENING_CENTER_X + OPENING_WIDTH / 2.0

LEFT_JAMB_WIDTH = OPENING_LEFT_X - OUTER_LEFT_X
RIGHT_JAMB_WIDTH = OUTER_RIGHT_X - OPENING_RIGHT_X
TOP_BOTTOM_FRAME_HEIGHT = BODY_HEIGHT / 2.0 - OPENING_HEIGHT / 2.0

HINGE_BARREL_RADIUS = 0.0065
HINGE_BARREL_LENGTH = 0.055
HINGE_AXIS_X = OPENING_LEFT_X + HINGE_BARREL_RADIUS + 0.0005
HINGE_AXIS_Y = BODY_DEPTH / 2.0 + 0.008
DOOR_LEAF_X_OFFSET = 0.002
DOOR_RIGHT_CLEARANCE = 0.005

DOOR_WIDTH = OPENING_RIGHT_X - DOOR_RIGHT_CLEARANCE - (HINGE_AXIS_X + DOOR_LEAF_X_OFFSET)
DOOR_HEIGHT = OPENING_HEIGHT - 0.008
DOOR_THICKNESS = 0.024
DOOR_PANEL_CENTER_Y = -0.010


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_access_panel")

    body_paint = model.material("body_paint", rgba=(0.23, 0.25, 0.27, 1.0))
    door_paint = model.material("door_paint", rgba=(0.35, 0.37, 0.39, 1.0))
    cavity_finish = model.material("cavity_finish", rgba=(0.08, 0.09, 0.10, 1.0))
    hardware = model.material("hardware", rgba=(0.62, 0.64, 0.66, 1.0))
    gasket = model.material("gasket", rgba=(0.05, 0.05, 0.05, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BACK_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH / 2.0 + BACK_THICKNESS / 2.0, 0.0)),
        material=cavity_finish,
        name="back_panel",
    )
    body.visual(
        Box((SIDE_WALL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(OUTER_LEFT_X + SIDE_WALL_THICKNESS / 2.0, 0.0, 0.0),
        ),
        material=body_paint,
        name="left_wall",
    )
    body.visual(
        Box((SIDE_WALL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(OUTER_RIGHT_X - SIDE_WALL_THICKNESS / 2.0, 0.0, 0.0),
        ),
        material=body_paint,
        name="right_wall",
    )
    body.visual(
        Box((BODY_WIDTH - 2.0 * SIDE_WALL_THICKNESS, BODY_DEPTH, TOP_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0 - TOP_BOTTOM_THICKNESS / 2.0)),
        material=body_paint,
        name="top_wall",
    )
    body.visual(
        Box((BODY_WIDTH - 2.0 * SIDE_WALL_THICKNESS, BODY_DEPTH, TOP_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -BODY_HEIGHT / 2.0 + TOP_BOTTOM_THICKNESS / 2.0)),
        material=body_paint,
        name="bottom_wall",
    )
    body.visual(
        Box((BODY_WIDTH, FRAME_THICKNESS, TOP_BOTTOM_FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH / 2.0 - FRAME_THICKNESS / 2.0,
                OPENING_CENTER_Z + OPENING_HEIGHT / 2.0 + TOP_BOTTOM_FRAME_HEIGHT / 2.0,
            )
        ),
        material=body_paint,
        name="top_frame",
    )
    body.visual(
        Box((BODY_WIDTH, FRAME_THICKNESS, TOP_BOTTOM_FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH / 2.0 - FRAME_THICKNESS / 2.0,
                OPENING_CENTER_Z - OPENING_HEIGHT / 2.0 - TOP_BOTTOM_FRAME_HEIGHT / 2.0,
            )
        ),
        material=body_paint,
        name="bottom_frame",
    )
    body.visual(
        Box((LEFT_JAMB_WIDTH, FRAME_THICKNESS, OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                OUTER_LEFT_X + LEFT_JAMB_WIDTH / 2.0,
                BODY_DEPTH / 2.0 - FRAME_THICKNESS / 2.0,
                OPENING_CENTER_Z,
            )
        ),
        material=body_paint,
        name="left_jamb",
    )
    body.visual(
        Box((0.018, 0.024, OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                OPENING_LEFT_X - 0.006,
                HINGE_AXIS_Y - 0.010,
                OPENING_CENTER_Z,
            )
        ),
        material=hardware,
        name="body_hinge_leaf",
    )
    for name, z_pos in (
        ("body_hinge_knuckle_lower", -0.0725),
        ("body_hinge_knuckle_upper", 0.0725),
    ):
        body.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=0.090),
            origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, z_pos)),
            material=hardware,
            name=name,
        )
    body.visual(
        Box((RIGHT_JAMB_WIDTH, FRAME_THICKNESS, OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                OPENING_RIGHT_X + RIGHT_JAMB_WIDTH / 2.0,
                BODY_DEPTH / 2.0 - FRAME_THICKNESS / 2.0,
                OPENING_CENTER_Z,
            )
        ),
        material=body_paint,
        name="right_jamb",
    )
    body.visual(
        Box((OPENING_WIDTH, 0.004, OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                OPENING_CENTER_X,
                BODY_DEPTH / 2.0 - FRAME_THICKNESS - 0.002,
                OPENING_CENTER_Z,
            )
        ),
        material=gasket,
        name="opening_gasket",
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_LEAF_X_OFFSET + DOOR_WIDTH / 2.0, DOOR_PANEL_CENTER_Y, 0.0)),
        material=door_paint,
        name="door_leaf",
    )
    door.visual(
        Box((DOOR_WIDTH - 0.040, DOOR_THICKNESS * 0.55, DOOR_HEIGHT - 0.050)),
        origin=Origin(
            xyz=(DOOR_LEAF_X_OFFSET + DOOR_WIDTH / 2.0 + 0.006, DOOR_PANEL_CENTER_Y - 0.002, 0.0),
        ),
        material=body_paint,
        name="inner_stiffener",
    )
    for index, z_pos in enumerate((-0.145, 0.0, 0.145), start=1):
        door.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=hardware,
            name=f"hinge_knuckle_{index}",
        )
    door.visual(
        Box((0.020, 0.018, 0.082)),
        origin=Origin(xyz=(DOOR_LEAF_X_OFFSET + DOOR_WIDTH - 0.040, 0.011, 0.0)),
        material=hardware,
        name="latch_pull",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, OPENING_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("body_to_door")

    left_jamb = body.get_visual("left_jamb")
    right_jamb = body.get_visual("right_jamb")

    left_width = left_jamb.geometry.size[0]
    right_width = right_jamb.geometry.size[0]
    ctx.check(
        "opening offset leaves a heavier latch-side body",
        right_width > left_width + 0.10,
        details=f"left_jamb_width={left_width:.3f}, right_jamb_width={right_width:.3f}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="x",
            min_gap=0.003,
            max_gap=0.012,
            positive_elem="door_leaf",
            negative_elem="left_jamb",
            name="door keeps a hinge-side reveal",
        )
        ctx.expect_gap(
            body,
            door,
            axis="x",
            min_gap=0.003,
            max_gap=0.012,
            positive_elem="right_jamb",
            negative_elem="door_leaf",
            name="door keeps a latch-side reveal",
        )
        ctx.expect_gap(
            body,
            door,
            axis="z",
            min_gap=0.003,
            max_gap=0.010,
            positive_elem="top_frame",
            negative_elem="door_leaf",
            name="door keeps a top reveal",
        )
        ctx.expect_gap(
            door,
            body,
            axis="z",
            min_gap=0.003,
            max_gap=0.010,
            positive_elem="door_leaf",
            negative_elem="bottom_frame",
            name="door keeps a bottom reveal",
        )

        closed_pull = ctx.part_element_world_aabb(door, elem="latch_pull")

    with ctx.pose({hinge: hinge.motion_limits.upper}):
        open_pull = ctx.part_element_world_aabb(door, elem="latch_pull")

    closed_pull_max_y = closed_pull[1][1] if closed_pull is not None else None
    open_pull_max_y = open_pull[1][1] if open_pull is not None else None
    ctx.check(
        "positive hinge motion swings the latch outward",
        closed_pull_max_y is not None
        and open_pull_max_y is not None
        and open_pull_max_y > closed_pull_max_y + 0.16,
        details=f"closed_pull_max_y={closed_pull_max_y}, open_pull_max_y={open_pull_max_y}",
    )

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
