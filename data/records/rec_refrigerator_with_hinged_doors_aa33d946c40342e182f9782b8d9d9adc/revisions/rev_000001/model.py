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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


WIDTH = 0.92
DEPTH = 0.72
HEIGHT = 1.78

WALL = 0.03
BACK_PANEL = 0.02
DIVIDER = 0.04

DOOR_BOTTOM = 0.04
DOOR_HEIGHT = 1.72
DOOR_THICKNESS = 0.065
FRONT_GAP = 0.0
DOOR_HINGE_Y = DEPTH / 2.0 + FRONT_GAP + DOOR_THICKNESS / 2.0

LEFT_DOOR_WIDTH = 0.39
RIGHT_DOOR_WIDTH = 0.524

DIVIDER_CENTER_X = -0.067

RIGHT_DOOR_CORE_THICKNESS = 0.040
RIGHT_DOOR_CORE_Y = -0.012
RIGHT_DOOR_FACE_THICKNESS = 0.026
RIGHT_DOOR_FACE_Y = 0.019

SERVICE_OPENING_LEFT = -0.353
SERVICE_OPENING_RIGHT = -0.177
SERVICE_OPENING_BOTTOM = 1.000
SERVICE_OPENING_TOP = 1.306

SERVICE_FLAP_WIDTH = 0.170
SERVICE_FLAP_HEIGHT = 0.300
SERVICE_FLAP_THICKNESS = 0.020
SERVICE_FLAP_LEFT = SERVICE_OPENING_LEFT
SERVICE_FLAP_BOTTOM = 1.003
SERVICE_FLAP_Y = 0.022


def _add_vertical_handle(
    part,
    *,
    x: float,
    y: float,
    z: float,
    bar_length: float,
    bar_radius: float,
    post_length: float,
    post_radius: float,
    post_offset: float,
    bar_name: str,
) -> None:
    part.visual(
        Cylinder(radius=bar_radius, length=bar_length),
        origin=Origin(xyz=(x, y, z)),
        material="handle",
        name=bar_name,
    )
    for sign, suffix in ((-1.0, "lower"), (1.0, "upper")):
        part.visual(
            Cylinder(radius=post_radius, length=post_length),
            origin=Origin(
                xyz=(x, y - 0.001, z + sign * post_offset),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material="handle",
            name=f"{bar_name}_{suffix}_post",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_by_side_refrigerator")

    model.material("stainless", rgba=(0.82, 0.84, 0.86, 1.0))
    model.material("cabinet_inner", rgba=(0.93, 0.94, 0.96, 1.0))
    model.material("handle", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("seal", rgba=(0.10, 0.11, 0.12, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((WALL, DEPTH, HEIGHT)),
        origin=Origin(xyz=(-WIDTH / 2.0 + WALL / 2.0, 0.0, HEIGHT / 2.0)),
        material="stainless",
        name="cabinet_left_wall",
    )
    cabinet.visual(
        Box((WALL, DEPTH, HEIGHT)),
        origin=Origin(xyz=(WIDTH / 2.0 - WALL / 2.0, 0.0, HEIGHT / 2.0)),
        material="stainless",
        name="cabinet_right_wall",
    )
    cabinet.visual(
        Box((WIDTH, DEPTH, DOOR_BOTTOM)),
        origin=Origin(xyz=(0.0, 0.0, DOOR_BOTTOM / 2.0)),
        material="stainless",
        name="cabinet_bottom",
    )
    cabinet.visual(
        Box((WIDTH, DEPTH, HEIGHT - (DOOR_BOTTOM + DOOR_HEIGHT))),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                DOOR_BOTTOM + DOOR_HEIGHT + (HEIGHT - (DOOR_BOTTOM + DOOR_HEIGHT)) / 2.0,
            )
        ),
        material="stainless",
        name="cabinet_top",
    )
    cabinet.visual(
        Box((WIDTH, BACK_PANEL, HEIGHT)),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0 + BACK_PANEL / 2.0, HEIGHT / 2.0)),
        material="cabinet_inner",
        name="cabinet_back",
    )
    cabinet.visual(
        Box((DIVIDER, DEPTH - BACK_PANEL, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(
                DIVIDER_CENTER_X,
                BACK_PANEL / 2.0,
                DOOR_BOTTOM + DOOR_HEIGHT / 2.0,
            )
        ),
        material="cabinet_inner",
        name="cabinet_divider",
    )

    left_door = model.part("left_door")
    left_door.visual(
        Box((LEFT_DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(LEFT_DOOR_WIDTH / 2.0, 0.0, DOOR_HEIGHT / 2.0)),
        material="stainless",
        name="left_door_panel",
    )
    _add_vertical_handle(
        left_door,
        x=LEFT_DOOR_WIDTH - 0.048,
        y=DOOR_THICKNESS / 2.0 + 0.022,
        z=0.93,
        bar_length=1.12,
        bar_radius=0.011,
        post_length=0.046,
        post_radius=0.0075,
        post_offset=0.29,
        bar_name="left_door_handle_bar",
    )
    left_door.visual(
        Box((LEFT_DOOR_WIDTH - 0.050, 0.008, DOOR_HEIGHT - 0.080)),
        origin=Origin(
            xyz=(
                LEFT_DOOR_WIDTH / 2.0,
                -DOOR_THICKNESS / 2.0 + 0.004,
                DOOR_HEIGHT / 2.0,
            )
        ),
        material="seal",
        name="left_door_rear_seal",
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((RIGHT_DOOR_WIDTH, RIGHT_DOOR_CORE_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(-RIGHT_DOOR_WIDTH / 2.0, RIGHT_DOOR_CORE_Y, DOOR_HEIGHT / 2.0)
        ),
        material="stainless",
        name="right_door_core",
    )
    right_door.visual(
        Box(
            (
                RIGHT_DOOR_WIDTH,
                RIGHT_DOOR_FACE_THICKNESS,
                DOOR_HEIGHT - SERVICE_OPENING_TOP,
            )
        ),
        origin=Origin(
            xyz=(
                -RIGHT_DOOR_WIDTH / 2.0,
                RIGHT_DOOR_FACE_Y,
                SERVICE_OPENING_TOP + (DOOR_HEIGHT - SERVICE_OPENING_TOP) / 2.0,
            )
        ),
        material="stainless",
        name="right_door_face_top",
    )
    right_door.visual(
        Box((RIGHT_DOOR_WIDTH, RIGHT_DOOR_FACE_THICKNESS, SERVICE_OPENING_BOTTOM)),
        origin=Origin(
            xyz=(
                -RIGHT_DOOR_WIDTH / 2.0,
                RIGHT_DOOR_FACE_Y,
                SERVICE_OPENING_BOTTOM / 2.0,
            )
        ),
        material="stainless",
        name="right_door_face_bottom",
    )
    right_door.visual(
        Box(
            (
                SERVICE_OPENING_LEFT + RIGHT_DOOR_WIDTH,
                RIGHT_DOOR_FACE_THICKNESS,
                SERVICE_OPENING_TOP - SERVICE_OPENING_BOTTOM,
            )
        ),
        origin=Origin(
            xyz=(
                (SERVICE_OPENING_LEFT - RIGHT_DOOR_WIDTH) / 2.0,
                RIGHT_DOOR_FACE_Y,
                (SERVICE_OPENING_BOTTOM + SERVICE_OPENING_TOP) / 2.0,
            )
        ),
        material="stainless",
        name="right_door_face_left",
    )
    right_door.visual(
        Box(
            (
                -SERVICE_OPENING_RIGHT,
                RIGHT_DOOR_FACE_THICKNESS,
                SERVICE_OPENING_TOP - SERVICE_OPENING_BOTTOM,
            )
        ),
        origin=Origin(
            xyz=(
                SERVICE_OPENING_RIGHT / 2.0,
                RIGHT_DOOR_FACE_Y,
                (SERVICE_OPENING_BOTTOM + SERVICE_OPENING_TOP) / 2.0,
            )
        ),
        material="stainless",
        name="right_door_face_right",
    )
    right_door.visual(
        Box((0.075, 0.009, DOOR_HEIGHT - 0.060)),
        origin=Origin(
            xyz=(
                -0.075 / 2.0,
                -DOOR_THICKNESS / 2.0 + 0.0045,
                DOOR_HEIGHT / 2.0,
            )
        ),
        material="seal",
        name="right_door_hinge_seal",
    )
    right_door.visual(
        Box((0.075, 0.009, DOOR_HEIGHT - 0.060)),
        origin=Origin(
            xyz=(
                -RIGHT_DOOR_WIDTH + 0.075 / 2.0,
                -DOOR_THICKNESS / 2.0 + 0.0045,
                DOOR_HEIGHT / 2.0,
            )
        ),
        material="seal",
        name="right_door_outer_seal",
    )
    _add_vertical_handle(
        right_door,
        x=-RIGHT_DOOR_WIDTH + 0.050,
        y=DOOR_THICKNESS / 2.0 + 0.022,
        z=0.93,
        bar_length=1.12,
        bar_radius=0.011,
        post_length=0.046,
        post_radius=0.0075,
        post_offset=0.29,
        bar_name="right_door_handle_bar",
    )

    service_flap = model.part("service_flap")
    service_flap.visual(
        Box((SERVICE_FLAP_WIDTH, SERVICE_FLAP_THICKNESS, SERVICE_FLAP_HEIGHT)),
        origin=Origin(
            xyz=(
                SERVICE_FLAP_WIDTH / 2.0,
                0.0,
                SERVICE_FLAP_HEIGHT / 2.0,
            )
        ),
        material="stainless",
        name="service_flap_panel",
    )
    service_flap.visual(
        Box((0.070, 0.012, 0.030)),
        origin=Origin(
            xyz=(SERVICE_FLAP_WIDTH - 0.045, 0.010, SERVICE_FLAP_HEIGHT / 2.0)
        ),
        material="handle",
        name="service_flap_pull",
    )

    model.articulation(
        "cabinet_to_left_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_door,
        origin=Origin(xyz=(-WIDTH / 2.0, DOOR_HINGE_Y, DOOR_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "cabinet_to_right_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_door,
        origin=Origin(xyz=(WIDTH / 2.0, DOOR_HINGE_Y, DOOR_BOTTOM)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "right_door_to_service_flap",
        ArticulationType.REVOLUTE,
        parent=right_door,
        child=service_flap,
        origin=Origin(
            xyz=(SERVICE_FLAP_LEFT, SERVICE_FLAP_Y, SERVICE_FLAP_BOTTOM)
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.20,
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

    cabinet = object_model.get_part("cabinet")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    service_flap = object_model.get_part("service_flap")

    left_hinge = object_model.get_articulation("cabinet_to_left_door")
    right_hinge = object_model.get_articulation("cabinet_to_right_door")
    flap_hinge = object_model.get_articulation("right_door_to_service_flap")

    flap_panel = service_flap.get_visual("service_flap_panel")

    ctx.check(
        "main doors use vertical side hinges",
        left_hinge.axis == (0.0, 0.0, 1.0) and right_hinge.axis == (0.0, 0.0, -1.0),
        details=f"left_axis={left_hinge.axis}, right_axis={right_hinge.axis}",
    )
    ctx.check(
        "service flap uses a vertical face hinge",
        flap_hinge.axis == (0.0, 0.0, 1.0),
        details=f"flap_axis={flap_hinge.axis}",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0, flap_hinge: 0.0}):
        ctx.expect_gap(
            left_door,
            cabinet,
            axis="y",
            max_gap=0.001,
            max_penetration=0.00001,
            name="left door sits just ahead of cabinet face",
        )
        ctx.expect_gap(
            right_door,
            cabinet,
            axis="y",
            max_gap=0.001,
            max_penetration=0.00001,
            name="right door sits just ahead of cabinet face",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.004,
            max_gap=0.008,
            name="closed doors keep a narrow center seam",
        )
        ctx.expect_overlap(
            left_door,
            cabinet,
            axes="xz",
            min_overlap=0.25,
            name="left door covers the left opening",
        )
        ctx.expect_overlap(
            right_door,
            cabinet,
            axes="xz",
            min_overlap=0.25,
            name="right door covers the right opening",
        )
        ctx.expect_within(
            service_flap,
            right_door,
            axes="xz",
            inner_elem=flap_panel,
            margin=0.0,
            name="service flap sits within the larger right door panel",
        )
        ctx.expect_gap(
            service_flap,
            right_door,
            axis="y",
            positive_elem=flap_panel,
            negative_elem="right_door_core",
            min_gap=0.002,
            max_gap=0.010,
            name="service flap closes ahead of the service cavity floor",
        )

        left_closed = ctx.part_world_aabb(left_door)
        right_closed = ctx.part_world_aabb(right_door)
        flap_closed = ctx.part_element_world_aabb(service_flap, elem=flap_panel)

    with ctx.pose({left_hinge: 1.15}):
        left_open = ctx.part_world_aabb(left_door)

    with ctx.pose({right_hinge: 1.15}):
        right_open = ctx.part_world_aabb(right_door)

    with ctx.pose({flap_hinge: 1.0}):
        flap_open = ctx.part_element_world_aabb(service_flap, elem=flap_panel)

    ctx.check(
        "left door swings outward from the left cabinet side",
        left_closed is not None
        and left_open is not None
        and left_open[1][1] > left_closed[1][1] + 0.12
        and left_open[1][0] < left_closed[1][0] - 0.05,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right door swings outward from the right cabinet side",
        right_closed is not None
        and right_open is not None
        and right_open[1][1] > right_closed[1][1] + 0.12
        and right_open[0][0] > right_closed[0][0] + 0.05,
        details=f"closed={right_closed}, open={right_open}",
    )
    ctx.check(
        "service flap opens out from the door face",
        flap_closed is not None
        and flap_open is not None
        and flap_open[1][1] > flap_closed[1][1] + 0.05,
        details=f"closed={flap_closed}, open={flap_open}",
    )
    ctx.check(
        "right door is the larger main door",
        left_closed is not None
        and right_closed is not None
        and (right_closed[1][0] - right_closed[0][0])
        > (left_closed[1][0] - left_closed[0][0]) + 0.10,
        details=f"left={left_closed}, right={right_closed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
