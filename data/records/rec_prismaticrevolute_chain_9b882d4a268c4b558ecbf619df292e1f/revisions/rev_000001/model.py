from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CHANNEL_L = 0.42
CHANNEL_W = 0.152
BASE_T = 0.012
WALL_T = 0.012
WALL_H = 0.020
STOP_T = 0.014
STOP_H = 0.026
FRONT_BUMPER_H = 0.012

SHUTTLE_L = 0.140
SHUTTLE_W = 0.104
SHUTTLE_H = 0.034
SHUTTLE_RETRACT_X = -0.090
SHUTTLE_TRAVEL = 0.180

TOWER_L = 0.052
TOWER_T = 0.014
TOWER_H = 0.034
TOWER_X = 0.026
TOWER_Y = 0.049
TOWER_Z = 0.024
HINGE_X = 0.038
HINGE_Y = TOWER_Y + (TOWER_T / 2.0)
HINGE_Z = 0.048

PADDLE_L = 0.086
PADDLE_T = 0.007
PADDLE_H = 0.052
PADDLE_OPEN = 1.15


def _build_paddle_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PADDLE_L, PADDLE_T, PADDLE_H, centered=(False, False, True))
        .edges("|Y")
        .fillet(0.004)
    )
    slot = (
        cq.Workplane("XY")
        .box(0.034, PADDLE_T + 0.002, 0.022, centered=(True, True, True))
        .translate((0.046, PADDLE_T / 2.0, 0.0))
    )
    return plate.cut(slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drawer_shuttle_paddle")

    model.material("powder_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("machined_alloy", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("signal_yellow", rgba=(0.92, 0.76, 0.12, 1.0))
    model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base_channel")
    base.visual(
        Box((CHANNEL_L, CHANNEL_W, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material="powder_gray",
        name="base_plate",
    )
    base.visual(
        Box((CHANNEL_L - (2.0 * STOP_T), WALL_T, WALL_H)),
        origin=Origin(
            xyz=(0.0, (CHANNEL_W / 2.0) - (WALL_T / 2.0), BASE_T + (WALL_H / 2.0))
        ),
        material="powder_gray",
        name="left_wall",
    )
    base.visual(
        Box((CHANNEL_L - (2.0 * STOP_T), WALL_T, WALL_H)),
        origin=Origin(
            xyz=(0.0, -((CHANNEL_W / 2.0) - (WALL_T / 2.0)), BASE_T + (WALL_H / 2.0))
        ),
        material="powder_gray",
        name="right_wall",
    )
    base.visual(
        Box((STOP_T, CHANNEL_W - (2.0 * WALL_T), STOP_H)),
        origin=Origin(
            xyz=((-CHANNEL_L / 2.0) + (STOP_T / 2.0), 0.0, BASE_T + (STOP_H / 2.0))
        ),
        material="powder_gray",
        name="rear_stop",
    )
    base.visual(
        Box((STOP_T, CHANNEL_W - (2.0 * WALL_T), FRONT_BUMPER_H)),
        origin=Origin(
            xyz=((CHANNEL_L / 2.0) - (STOP_T / 2.0), 0.0, BASE_T + (FRONT_BUMPER_H / 2.0))
        ),
        material="rubber_black",
        name="front_bumper",
    )
    base.inertial = Inertial.from_geometry(
        Box((CHANNEL_L, CHANNEL_W, BASE_T + WALL_H)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, (BASE_T + WALL_H) / 2.0)),
    )

    shuttle = model.part("shuttle")
    shuttle.visual(
        Box((SHUTTLE_L, SHUTTLE_W, SHUTTLE_H)),
        origin=Origin(xyz=(0.0, 0.0, SHUTTLE_H / 2.0)),
        material="machined_alloy",
        name="carriage_body",
    )
    shuttle.visual(
        Box((TOWER_L, TOWER_T, TOWER_H)),
        origin=Origin(xyz=(TOWER_X, TOWER_Y, TOWER_Z + (TOWER_H / 2.0))),
        material="machined_alloy",
        name="hinge_tower",
    )
    shuttle.inertial = Inertial.from_geometry(
        Box((SHUTTLE_L, SHUTTLE_W, SHUTTLE_H)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, SHUTTLE_H / 2.0)),
    )

    paddle = model.part("paddle")
    paddle.visual(
        mesh_from_cadquery(_build_paddle_shape(), "paddle_plate"),
        material="signal_yellow",
        name="paddle_plate",
    )
    paddle.visual(
        Box((0.012, PADDLE_T, 0.024)),
        origin=Origin(xyz=(PADDLE_L - 0.006, PADDLE_T / 2.0, 0.0)),
        material="rubber_black",
        name="paddle_tip",
    )
    paddle.inertial = Inertial.from_geometry(
        Box((PADDLE_L, PADDLE_T, PADDLE_H)),
        mass=0.25,
        origin=Origin(xyz=(PADDLE_L / 2.0, PADDLE_T / 2.0, 0.0)),
    )

    model.articulation(
        "channel_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=shuttle,
        origin=Origin(xyz=(SHUTTLE_RETRACT_X, 0.0, BASE_T)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SHUTTLE_TRAVEL,
            effort=120.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "shuttle_to_paddle",
        ArticulationType.REVOLUTE,
        parent=shuttle,
        child=paddle,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=PADDLE_OPEN,
            effort=12.0,
            velocity=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_channel")
    shuttle = object_model.get_part("shuttle")
    paddle = object_model.get_part("paddle")
    shuttle_slide = object_model.get_articulation("channel_to_shuttle")
    paddle_hinge = object_model.get_articulation("shuttle_to_paddle")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "shuttle_slide_axis",
        tuple(shuttle_slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected prismatic axis (1, 0, 0), got {shuttle_slide.axis}",
    )
    ctx.check(
        "paddle_hinge_axis",
        tuple(paddle_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"expected hinge axis (0, 0, 1), got {paddle_hinge.axis}",
    )
    ctx.expect_contact(shuttle, base, name="shuttle_supported_by_channel")
    ctx.expect_contact(paddle, shuttle, name="paddle_closed_against_shuttle")
    ctx.expect_within(
        shuttle,
        base,
        axes="y",
        margin=0.001,
        name="shuttle_stays_between_channel_walls",
    )

    with ctx.pose({shuttle_slide: SHUTTLE_TRAVEL}):
        ctx.expect_origin_gap(
            shuttle,
            base,
            axis="x",
            min_gap=0.085,
            max_gap=0.095,
            name="shuttle_extends_forward_on_positive_x",
        )
        ctx.expect_contact(
            shuttle,
            base,
            name="shuttle_remains_supported_when_extended",
        )

    with ctx.pose({paddle_hinge: PADDLE_OPEN}):
        ctx.expect_gap(
            paddle,
            shuttle,
            axis="y",
            min_gap=0.045,
            positive_elem="paddle_tip",
            name="paddle_tip_swings_outward_on_positive_angle",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
