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


DECK_LENGTH = 1.80
DECK_WIDTH = 0.58
DECK_TOP_THICKNESS = 0.018
SIDE_RAIL_WIDTH = 0.055
SIDE_RAIL_HEIGHT = 0.050
SIDE_RAIL_Y = 0.235
SIDE_RAIL_Z = -(DECK_TOP_THICKNESS + SIDE_RAIL_HEIGHT / 2.0)
MOUNT_PLATE_THICKNESS = 0.008
MOUNT_PLATE_Z = -(DECK_TOP_THICKNESS + SIDE_RAIL_HEIGHT + MOUNT_PLATE_THICKNESS / 2.0)
CASTER_ORIGIN_Z = MOUNT_PLATE_Z - MOUNT_PLATE_THICKNESS / 2.0

WHEEL_RADIUS = 0.055
WHEEL_WIDTH = 0.034
WHEEL_CENTER_Z = -0.092


def _add_caster(
    model: ArticulatedObject,
    deck,
    *,
    prefix: str,
    x: float,
    y: float,
    paint,
    zinc,
    rubber,
) -> tuple[str, str, str, str]:
    fork = model.part(f"{prefix}_fork")
    fork.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=zinc,
        name="swivel_head",
    )
    fork.visual(
        Box((0.062, 0.026, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=zinc,
        name="fork_crown",
    )
    fork.visual(
        Box((0.010, 0.018, 0.064)),
        origin=Origin(xyz=(-0.024, 0.0, -0.068)),
        material=paint,
        name="left_tine",
    )
    fork.visual(
        Box((0.010, 0.018, 0.064)),
        origin=Origin(xyz=(0.024, 0.0, -0.068)),
        material=paint,
        name="right_tine",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.070, 0.030, 0.110)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
    )

    wheel = model.part(f"{prefix}_wheel")
    wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.027, length=0.024),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=zinc,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=paint,
        name="axle_boss",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=1.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    swivel_name = f"{prefix}_swivel"
    spin_name = f"{prefix}_wheel_spin"

    model.articulation(
        swivel_name,
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=fork,
        origin=Origin(xyz=(x, y, CASTER_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0),
    )
    model.articulation(
        spin_name,
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=24.0),
    )
    return fork.name, wheel.name, swivel_name, spin_name


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lumber_cart")

    deck_wood = model.material("deck_wood", rgba=(0.60, 0.48, 0.31, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.19, 0.22, 0.20, 1.0))
    zinc = model.material("zinc", rgba=(0.68, 0.70, 0.73, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((DECK_LENGTH, DECK_WIDTH, DECK_TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -DECK_TOP_THICKNESS / 2.0)),
        material=deck_wood,
        name="deck_panel",
    )
    deck.visual(
        Box((1.68, SIDE_RAIL_WIDTH, SIDE_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, SIDE_RAIL_Y, SIDE_RAIL_Z)),
        material=frame_paint,
        name="right_side_rail",
    )
    deck.visual(
        Box((1.68, SIDE_RAIL_WIDTH, SIDE_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, -SIDE_RAIL_Y, SIDE_RAIL_Z)),
        material=frame_paint,
        name="left_side_rail",
    )
    deck.visual(
        Box((0.070, 0.470, SIDE_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.815, 0.0, SIDE_RAIL_Z)),
        material=frame_paint,
        name="front_cross_rail",
    )
    deck.visual(
        Box((0.070, 0.470, SIDE_RAIL_HEIGHT)),
        origin=Origin(xyz=(-0.825, 0.0, SIDE_RAIL_Z)),
        material=frame_paint,
        name="rear_cross_rail",
    )
    deck.visual(
        Box((1.50, 0.090, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=frame_paint,
        name="center_stiffener",
    )
    for corner_name, cx, cy in (
        ("front_left", 0.775, 0.240),
        ("front_right", 0.775, -0.240),
        ("rear_left", -0.775, 0.240),
        ("rear_right", -0.775, -0.240),
    ):
        deck.visual(
            Box((0.110, 0.090, MOUNT_PLATE_THICKNESS)),
            origin=Origin(xyz=(cx, cy, MOUNT_PLATE_Z)),
            material=zinc,
            name=f"{corner_name}_caster_plate",
        )
    deck.visual(
        Box((0.024, 0.090, 0.020)),
        origin=Origin(xyz=(-0.888, 0.230, -0.010)),
        material=frame_paint,
        name="left_hinge_block",
    )
    deck.visual(
        Box((0.024, 0.090, 0.020)),
        origin=Origin(xyz=(-0.888, -0.230, -0.010)),
        material=frame_paint,
        name="right_hinge_block",
    )
    deck.visual(
        Box((0.046, 0.060, 0.014)),
        origin=Origin(xyz=(-0.922, 0.230, -0.006)),
        material=frame_paint,
        name="left_hinge_shelf",
    )
    deck.visual(
        Box((0.046, 0.060, 0.014)),
        origin=Origin(xyz=(-0.922, -0.230, -0.006)),
        material=frame_paint,
        name="right_hinge_shelf",
    )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.160)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    push_frame = model.part("push_frame")
    push_frame.visual(
        Cylinder(radius=0.014, length=0.070),
        origin=Origin(xyz=(0.0, 0.230, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="left_hinge_sleeve",
    )
    push_frame.visual(
        Cylinder(radius=0.014, length=0.070),
        origin=Origin(xyz=(0.0, -0.230, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="right_hinge_sleeve",
    )
    push_frame.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(-0.015, 0.230, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
        name="left_foot_tube",
    )
    push_frame.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(-0.015, -0.230, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
        name="right_foot_tube",
    )
    push_frame.visual(
        Cylinder(radius=0.016, length=0.920),
        origin=Origin(xyz=(-0.030, 0.230, 0.460)),
        material=frame_paint,
        name="left_upright",
    )
    push_frame.visual(
        Cylinder(radius=0.016, length=0.920),
        origin=Origin(xyz=(-0.030, -0.230, 0.460)),
        material=frame_paint,
        name="right_upright",
    )
    push_frame.visual(
        Cylinder(radius=0.016, length=0.460),
        origin=Origin(xyz=(-0.030, 0.0, 0.920), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="top_grip_bar",
    )
    push_frame.visual(
        Cylinder(radius=0.010, length=0.460),
        origin=Origin(xyz=(-0.030, 0.0, 0.430), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="mid_brace",
    )
    push_frame.inertial = Inertial.from_geometry(
        Box((0.060, 0.500, 0.960)),
        mass=8.0,
        origin=Origin(xyz=(-0.030, 0.0, 0.480)),
    )

    model.articulation(
        "rear_push_frame_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=push_frame,
        origin=Origin(xyz=(-0.915, 0.0, 0.015)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.6,
            lower=0.0,
            upper=1.35,
        ),
    )

    _add_caster(
        model,
        deck,
        prefix="front_left",
        x=0.775,
        y=0.240,
        paint=frame_paint,
        zinc=zinc,
        rubber=rubber,
    )
    _add_caster(
        model,
        deck,
        prefix="front_right",
        x=0.775,
        y=-0.240,
        paint=frame_paint,
        zinc=zinc,
        rubber=rubber,
    )
    _add_caster(
        model,
        deck,
        prefix="rear_left",
        x=-0.775,
        y=0.240,
        paint=frame_paint,
        zinc=zinc,
        rubber=rubber,
    )
    _add_caster(
        model,
        deck,
        prefix="rear_right",
        x=-0.775,
        y=-0.240,
        paint=frame_paint,
        zinc=zinc,
        rubber=rubber,
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

    deck = object_model.get_part("deck")
    push_frame = object_model.get_part("push_frame")
    push_hinge = object_model.get_articulation("rear_push_frame_hinge")

    caster_joint_names = (
        "front_left_swivel",
        "front_right_swivel",
        "rear_left_swivel",
        "rear_right_swivel",
    )
    wheel_joint_names = (
        "front_left_wheel_spin",
        "front_right_wheel_spin",
        "rear_left_wheel_spin",
        "rear_right_wheel_spin",
    )

    ctx.check(
        "push frame hinge uses deck-width axis",
        push_hinge.axis == (0.0, 1.0, 0.0),
        details=f"axis={push_hinge.axis}",
    )
    ctx.check(
        "all caster forks swivel on vertical pivots",
        all(object_model.get_articulation(name).axis == (0.0, 0.0, 1.0) for name in caster_joint_names),
        details=str({name: object_model.get_articulation(name).axis for name in caster_joint_names}),
    )
    ctx.check(
        "all caster wheels spin on horizontal axles",
        all(object_model.get_articulation(name).axis == (1.0, 0.0, 0.0) for name in wheel_joint_names),
        details=str({name: object_model.get_articulation(name).axis for name in wheel_joint_names}),
    )

    for wheel_name in ("front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"):
        ctx.expect_origin_gap(
            deck,
            object_model.get_part(wheel_name),
            axis="z",
            min_gap=0.13,
            max_gap=0.20,
            name=f"{wheel_name} hangs below the platform deck",
        )

    rest_push_aabb = ctx.part_world_aabb(push_frame)
    ctx.check(
        "push frame stands tall in the default pose",
        rest_push_aabb is not None and rest_push_aabb[1][2] > 0.92,
        details=f"aabb={rest_push_aabb}",
    )
    ctx.expect_overlap(
        push_frame,
        deck,
        axes="y",
        min_overlap=0.40,
        name="upright push frame still spans most of the cart width",
    )

    front_left_swivel = object_model.get_articulation("front_left_swivel")
    front_left_wheel = object_model.get_part("front_left_wheel")
    rest_wheel_pos = ctx.part_world_position(front_left_wheel)
    with ctx.pose({front_left_swivel: pi / 2.0}):
        swiveled_pos = ctx.part_world_position(front_left_wheel)
    ctx.check(
        "front left caster swivels about its vertical kingpin",
        rest_wheel_pos is not None
        and swiveled_pos is not None
        and abs(rest_wheel_pos[0] - swiveled_pos[0]) < 1e-6
        and abs(rest_wheel_pos[1] - swiveled_pos[1]) < 1e-6
        and abs(rest_wheel_pos[2] - swiveled_pos[2]) < 1e-6,
        details=f"rest={rest_wheel_pos}, swiveled={swiveled_pos}",
    )

    with ctx.pose({push_hinge: push_hinge.motion_limits.upper}):
        folded_push_aabb = ctx.part_world_aabb(push_frame)
        ctx.check(
            "push frame folds down over the platform",
            folded_push_aabb is not None and folded_push_aabb[1][2] < 0.30,
            details=f"aabb={folded_push_aabb}",
        )
        ctx.expect_overlap(
            push_frame,
            deck,
            axes="xy",
            min_overlap=0.30,
            name="folded push frame lies over the deck footprint",
        )
        ctx.expect_gap(
            push_frame,
            deck,
            axis="z",
            max_gap=0.22,
            max_penetration=0.001,
            name="folded push frame stays above the deck without intersecting it",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
