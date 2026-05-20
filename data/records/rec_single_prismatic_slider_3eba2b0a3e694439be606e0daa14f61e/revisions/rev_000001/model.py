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


BASE_LENGTH = 0.46
BASE_WIDTH = 0.18
BOTTOM_THICKNESS = 0.008
SIDE_WALL_THICKNESS = 0.010
SIDE_WALL_HEIGHT = 0.044
BASE_HEIGHT = BOTTOM_THICKNESS + SIDE_WALL_HEIGHT
CHANNEL_WIDTH = BASE_WIDTH - 2.0 * SIDE_WALL_THICKNESS

STRIP_RUN_Y = 0.054
FLOOR_STRIP_WIDTH = 0.018
FLOOR_STRIP_HEIGHT = 0.003
FLOOR_STRIP_LENGTH = 0.296
FLOOR_STRIP_CENTER_X = 0.048

SIDE_STRIP_THICKNESS = 0.0025
SIDE_STRIP_HEIGHT = 0.018
SIDE_STRIP_LENGTH = 0.36
SIDE_STRIP_CENTER_X = 0.02
SIDE_STRIP_BOTTOM_Z = 0.012

STOP_BLOCK_LENGTH = 0.012
STOP_BLOCK_WIDTH = 0.024
STOP_BLOCK_HEIGHT = FLOOR_STRIP_HEIGHT
FRONT_STOP_NEGATIVE_X = 0.208
REAR_STOP_POSITIVE_X = -0.112
FRONT_STOP_CENTER_X = FRONT_STOP_NEGATIVE_X + STOP_BLOCK_LENGTH / 2.0
REAR_STOP_CENTER_X = REAR_STOP_POSITIVE_X - STOP_BLOCK_LENGTH / 2.0

REAR_BRIDGE_LENGTH = 0.020
REAR_BRIDGE_HEIGHT = 0.020

CARRIAGE_LENGTH = 0.34
CARRIAGE_WIDTH = 0.153
RUNNER_HEIGHT = 0.004
CARRIAGE_BODY_HEIGHT = 0.038
CARRIAGE_HEIGHT = RUNNER_HEIGHT + CARRIAGE_BODY_HEIGHT
SIDE_WALL_CARRIAGE = 0.008
REAR_WALL_CARRIAGE = 0.012
FRONT_WALL_CARRIAGE = 0.024
TRAY_BOTTOM_THICKNESS = 0.006

RUNNER_LENGTH = 0.18
RUNNER_WIDTH = 0.018
RUNNER_CENTER_X = -0.082
RUNNER_CENTER_Y = STRIP_RUN_Y

FRONT_STOP_FEATURE_LENGTH = 0.026
FRONT_STOP_FEATURE_WIDTH = 0.11
FRONT_STOP_FEATURE_HEIGHT = 0.026
FRONT_STOP_FEATURE_CENTER_X = (
    CARRIAGE_LENGTH / 2.0 - FRONT_STOP_FEATURE_LENGTH / 2.0
)

PRISMATIC_CLOSED_X = 0.06
PRISMATIC_Z = BOTTOM_THICKNESS + FLOOR_STRIP_HEIGHT
TRAVEL = 0.14


def _box_shape(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    x, y, z = xyz
    return cq.Workplane("XY").box(sx, sy, sz, centered=(True, True, False)).translate((x, y, z))


def _set_box_inertial(
    part,
    *,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    mass: float,
) -> None:
    part.inertial = Inertial.from_geometry(
        Box(size),
        mass=mass,
        origin=Origin(xyz=center),
    )


def _base_body_shape() -> cq.Workplane:
    bottom = _box_shape(
        (BASE_LENGTH, BASE_WIDTH, BOTTOM_THICKNESS),
        (0.0, 0.0, 0.0),
    )
    left_wall = _box_shape(
        (BASE_LENGTH, SIDE_WALL_THICKNESS, SIDE_WALL_HEIGHT),
        (0.0, BASE_WIDTH / 2.0 - SIDE_WALL_THICKNESS / 2.0, BOTTOM_THICKNESS),
    )
    right_wall = _box_shape(
        (BASE_LENGTH, SIDE_WALL_THICKNESS, SIDE_WALL_HEIGHT),
        (0.0, -BASE_WIDTH / 2.0 + SIDE_WALL_THICKNESS / 2.0, BOTTOM_THICKNESS),
    )
    rear_bridge = _box_shape(
        (REAR_BRIDGE_LENGTH, CHANNEL_WIDTH, REAR_BRIDGE_HEIGHT),
        (-BASE_LENGTH / 2.0 + REAR_BRIDGE_LENGTH / 2.0, 0.0, BOTTOM_THICKNESS),
    )
    return bottom.union(left_wall).union(right_wall).union(rear_bridge)


def _base_wear_strips_shape() -> cq.Workplane:
    left_floor_strip = _box_shape(
        (FLOOR_STRIP_LENGTH, FLOOR_STRIP_WIDTH, FLOOR_STRIP_HEIGHT),
        (FLOOR_STRIP_CENTER_X, STRIP_RUN_Y, BOTTOM_THICKNESS),
    )
    right_floor_strip = _box_shape(
        (FLOOR_STRIP_LENGTH, FLOOR_STRIP_WIDTH, FLOOR_STRIP_HEIGHT),
        (FLOOR_STRIP_CENTER_X, -STRIP_RUN_Y, BOTTOM_THICKNESS),
    )
    strip_y = CHANNEL_WIDTH / 2.0 - SIDE_STRIP_THICKNESS / 2.0
    left_side_strip = _box_shape(
        (SIDE_STRIP_LENGTH, SIDE_STRIP_THICKNESS, SIDE_STRIP_HEIGHT),
        (SIDE_STRIP_CENTER_X, strip_y, SIDE_STRIP_BOTTOM_Z),
    )
    right_side_strip = _box_shape(
        (SIDE_STRIP_LENGTH, SIDE_STRIP_THICKNESS, SIDE_STRIP_HEIGHT),
        (SIDE_STRIP_CENTER_X, -strip_y, SIDE_STRIP_BOTTOM_Z),
    )
    return (
        left_floor_strip
        .union(right_floor_strip)
        .union(left_side_strip)
        .union(right_side_strip)
    )


def _front_stop_blocks_shape() -> cq.Workplane:
    front_left = _box_shape(
        (STOP_BLOCK_LENGTH, STOP_BLOCK_WIDTH, STOP_BLOCK_HEIGHT),
        (FRONT_STOP_CENTER_X, STRIP_RUN_Y, BOTTOM_THICKNESS),
    )
    front_right = _box_shape(
        (STOP_BLOCK_LENGTH, STOP_BLOCK_WIDTH, STOP_BLOCK_HEIGHT),
        (FRONT_STOP_CENTER_X, -STRIP_RUN_Y, BOTTOM_THICKNESS),
    )
    return front_left.union(front_right)


def _rear_stop_blocks_shape() -> cq.Workplane:
    rear_left = _box_shape(
        (STOP_BLOCK_LENGTH, STOP_BLOCK_WIDTH, STOP_BLOCK_HEIGHT),
        (REAR_STOP_CENTER_X, STRIP_RUN_Y, BOTTOM_THICKNESS),
    )
    rear_right = _box_shape(
        (STOP_BLOCK_LENGTH, STOP_BLOCK_WIDTH, STOP_BLOCK_HEIGHT),
        (REAR_STOP_CENTER_X, -STRIP_RUN_Y, BOTTOM_THICKNESS),
    )
    return rear_left.union(rear_right)


def _carriage_body_shape() -> cq.Workplane:
    outer = _box_shape(
        (CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_BODY_HEIGHT),
        (0.0, 0.0, RUNNER_HEIGHT),
    )
    cavity = _box_shape(
        (
            CARRIAGE_LENGTH - FRONT_WALL_CARRIAGE - REAR_WALL_CARRIAGE,
            CARRIAGE_WIDTH - 2.0 * SIDE_WALL_CARRIAGE,
            CARRIAGE_BODY_HEIGHT - TRAY_BOTTOM_THICKNESS,
        ),
        (
            (REAR_WALL_CARRIAGE - FRONT_WALL_CARRIAGE) / 2.0,
            0.0,
            RUNNER_HEIGHT + TRAY_BOTTOM_THICKNESS,
        ),
    )
    return outer.cut(cavity)


def _carriage_runners_shape() -> cq.Workplane:
    left_runner = _box_shape(
        (RUNNER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT),
        (RUNNER_CENTER_X, RUNNER_CENTER_Y, 0.0),
    )
    right_runner = _box_shape(
        (RUNNER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT),
        (RUNNER_CENTER_X, -RUNNER_CENTER_Y, 0.0),
    )
    return left_runner.union(right_runner)


def _front_stop_feature_shape() -> cq.Workplane:
    return _box_shape(
        (
            FRONT_STOP_FEATURE_LENGTH,
            FRONT_STOP_FEATURE_WIDTH,
            FRONT_STOP_FEATURE_HEIGHT,
        ),
        (FRONT_STOP_FEATURE_CENTER_X, 0.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="instrument_slide")
    model.material("base_aluminum", rgba=(0.75, 0.76, 0.78, 1.0))
    model.material("carriage_gray", rgba=(0.50, 0.52, 0.56, 1.0))
    model.material("wear_polymer", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("stop_black", rgba=(0.10, 0.11, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body_shape(), "base_body"),
        material="base_aluminum",
        name="base_body",
    )
    base.visual(
        mesh_from_cadquery(_base_wear_strips_shape(), "base_wear_strips"),
        material="wear_polymer",
        name="base_wear_strips",
    )
    base.visual(
        mesh_from_cadquery(_front_stop_blocks_shape(), "base_front_stops"),
        material="stop_black",
        name="base_front_stops",
    )
    base.visual(
        mesh_from_cadquery(_rear_stop_blocks_shape(), "base_rear_stops"),
        material="stop_black",
        name="base_rear_stops",
    )
    _set_box_inertial(
        base,
        size=(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT),
        center=(0.0, 0.0, BASE_HEIGHT / 2.0),
        mass=2.6,
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_body_shape(), "carriage_body"),
        material="carriage_gray",
        name="carriage_body",
    )
    carriage.visual(
        mesh_from_cadquery(_carriage_runners_shape(), "carriage_runners"),
        material="wear_polymer",
        name="carriage_runners",
    )
    carriage.visual(
        mesh_from_cadquery(_front_stop_feature_shape(), "carriage_front_stop"),
        material="stop_black",
        name="front_stop_feature",
    )
    _set_box_inertial(
        carriage,
        size=(CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT),
        center=(0.0, 0.0, CARRIAGE_HEIGHT / 2.0),
        mass=1.1,
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(PRISMATIC_CLOSED_X, 0.0, PRISMATIC_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.30,
            lower=0.0,
            upper=TRAVEL,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("base_to_carriage")

    base_wear_strips = base.get_visual("base_wear_strips")
    base_front_stops = base.get_visual("base_front_stops")
    base_rear_stops = base.get_visual("base_rear_stops")
    carriage_runners = carriage.get_visual("carriage_runners")
    front_stop_feature = carriage.get_visual("front_stop_feature")

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

    ctx.check("base_present", base is not None)
    ctx.check("carriage_present", carriage is not None)
    ctx.check("single_prismatic_slide", slide.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("slide_axis_is_channel_x", tuple(slide.axis) == (1.0, 0.0, 0.0))

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            base,
            carriage,
            elem_a=base_wear_strips,
            elem_b=carriage_runners,
            name="closed_pose_runners_supported",
        )
        ctx.expect_gap(
            carriage,
            base,
            axis="x",
            positive_elem=carriage_runners,
            negative_elem=base_rear_stops,
            min_gap=0.0,
            max_gap=0.0005,
            name="closed_pose_rear_stops_catch_runner_pack",
        )

    with ctx.pose({slide: TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_no_overlap")
        ctx.expect_contact(
            base,
            carriage,
            elem_a=base_wear_strips,
            elem_b=carriage_runners,
            name="open_pose_runners_still_supported",
        )
        ctx.expect_overlap(
            base,
            carriage,
            axes="x",
            min_overlap=0.195,
            name="open_pose_keeps_long_guide_overlap",
        )
        ctx.expect_gap(
            base,
            carriage,
            axis="x",
            positive_elem=base_front_stops,
            negative_elem=carriage_runners,
            max_gap=0.0005,
            max_penetration=1.0e-5,
            name="open_pose_front_stops_limit_travel",
        )
        ctx.expect_within(
            carriage,
            base,
            axes="y",
            inner_elem=front_stop_feature,
            margin=0.04,
            name="front_stop_feature_stays_laterally_centered",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
