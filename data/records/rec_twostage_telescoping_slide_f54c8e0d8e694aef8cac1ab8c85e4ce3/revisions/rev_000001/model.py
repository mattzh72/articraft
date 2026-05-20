from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OUTER_LENGTH = 0.34
OUTER_WIDTH = 0.055
OUTER_HEIGHT = 0.022
OUTER_WALL = 0.003
OUTER_BASE = 0.0035

MID_LENGTH = 0.24
MID_WIDTH = 0.043
MID_HEIGHT = 0.014
MID_WALL = 0.0025
MID_BASE = 0.0030

CARRIAGE_LENGTH = 0.16
CARRIAGE_PLATE_WIDTH = 0.038
CARRIAGE_SHOE_WIDTH = 0.031
CARRIAGE_PLATE_THICKNESS = 0.004
CARRIAGE_SHOE_HEIGHT = 0.009

STOWED_MID_OFFSET = OUTER_LENGTH - MID_LENGTH
STOWED_CARRIAGE_OFFSET = MID_LENGTH - CARRIAGE_LENGTH

MID_TRAVEL = 0.12
CARRIAGE_TRAVEL = 0.08


def _box_at(
    length: float,
    width: float,
    height: float,
    *,
    x0: float = 0.0,
    y: float = 0.0,
    z0: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(
        (x0 + length / 2.0, y, z0 + height / 2.0)
    )


def _make_channel_rail(
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    base: float,
    runner_width: float = 0.0,
    runner_height: float = 0.0,
    round_holes: tuple[float, ...] = (),
    slot_centers: tuple[float, ...] = (),
    slot_length: float = 0.028,
    slot_width: float = 0.006,
) -> cq.Workplane:
    base_plate = _box_at(length, width, base)
    left_wall = _box_at(
        length,
        wall,
        height - base,
        y=(width - wall) / 2.0,
        z0=base,
    )
    right_wall = _box_at(
        length,
        wall,
        height - base,
        y=-(width - wall) / 2.0,
        z0=base,
    )
    rail = base_plate.union(left_wall).union(right_wall)

    if runner_width > 0.0 and runner_height > 0.0:
        runner_center_y = width / 2.0 - wall - runner_width / 2.0
        left_runner = _box_at(
            length,
            runner_width,
            runner_height,
            y=runner_center_y,
            z0=height - runner_height,
        )
        right_runner = _box_at(
            length,
            runner_width,
            runner_height,
            y=-runner_center_y,
            z0=height - runner_height,
        )
        rail = rail.union(left_runner).union(right_runner)

    for x in round_holes:
        hole = (
            cq.Workplane("XY")
            .workplane(offset=-0.001)
            .center(x, 0.0)
            .circle(0.0032)
            .extrude(base + 0.003)
        )
        rail = rail.cut(hole)

    for x in slot_centers:
        slot = (
            cq.Workplane("XY")
            .workplane(offset=-0.001)
            .center(x, 0.0)
            .slot2D(slot_length, slot_width, angle=0.0)
            .extrude(base + 0.003)
        )
        rail = rail.cut(slot)

    return rail


def _make_terminal_carriage() -> cq.Workplane:
    shoe = _box_at(
        CARRIAGE_LENGTH,
        CARRIAGE_SHOE_WIDTH,
        CARRIAGE_SHOE_HEIGHT,
    )
    plate = _box_at(
        CARRIAGE_LENGTH,
        CARRIAGE_PLATE_WIDTH,
        CARRIAGE_PLATE_THICKNESS,
        z0=CARRIAGE_SHOE_HEIGHT,
    )
    carriage = shoe.union(plate)

    for x in (0.040, CARRIAGE_LENGTH - 0.040):
        hole = (
            cq.Workplane("XY")
            .workplane(offset=CARRIAGE_SHOE_HEIGHT - 0.001)
            .center(x, 0.0)
            .circle(0.0030)
            .extrude(CARRIAGE_PLATE_THICKNESS + 0.003)
        )
        carriage = carriage.cut(hole)

    return carriage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_slide")

    zinc = model.material("zinc_plated_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_steel = model.material("dark_rolled_steel", rgba=(0.43, 0.45, 0.48, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.17, 0.18, 0.20, 1.0))

    outer_rail = model.part("outer_rail")
    outer_shape = _make_channel_rail(
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        base=OUTER_BASE,
        runner_width=0.004,
        runner_height=0.0015,
        round_holes=(0.055, 0.285),
        slot_centers=(0.17,),
        slot_length=0.040,
        slot_width=0.007,
    )
    outer_rail.visual(
        mesh_from_cadquery(outer_shape, "outer_rail"),
        material=zinc,
        name="outer_shell",
    )

    intermediate_rail = model.part("intermediate_rail")
    intermediate_shape = _make_channel_rail(
        length=MID_LENGTH,
        width=MID_WIDTH,
        height=MID_HEIGHT,
        wall=MID_WALL,
        base=MID_BASE,
        runner_width=0.0035,
        runner_height=0.0012,
        round_holes=(0.050, 0.190),
        slot_centers=(0.12,),
        slot_length=0.032,
        slot_width=0.006,
    )
    intermediate_rail.visual(
        mesh_from_cadquery(intermediate_shape, "intermediate_rail"),
        material=dark_steel,
        name="intermediate_shell",
    )

    terminal_carriage = model.part("terminal_carriage")
    terminal_carriage.visual(
        mesh_from_cadquery(_make_terminal_carriage(), "terminal_carriage"),
        material=black_oxide,
        name="carriage_body",
    )

    model.articulation(
        "outer_to_intermediate",
        ArticulationType.PRISMATIC,
        parent=outer_rail,
        child=intermediate_rail,
        origin=Origin(xyz=(STOWED_MID_OFFSET, 0.0, OUTER_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.40,
            lower=0.0,
            upper=MID_TRAVEL,
        ),
    )
    model.articulation(
        "intermediate_to_carriage",
        ArticulationType.PRISMATIC,
        parent=intermediate_rail,
        child=terminal_carriage,
        origin=Origin(xyz=(STOWED_CARRIAGE_OFFSET, 0.0, MID_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.40,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    outer_rail = object_model.get_part("outer_rail")
    intermediate_rail = object_model.get_part("intermediate_rail")
    terminal_carriage = object_model.get_part("terminal_carriage")
    outer_to_intermediate = object_model.get_articulation("outer_to_intermediate")
    intermediate_to_carriage = object_model.get_articulation("intermediate_to_carriage")

    part_names = {part.name for part in object_model.parts}
    joint_names = {joint.name for joint in object_model.articulations}
    ctx.check(
        "outer rail present",
        "outer_rail" in part_names,
        "Expected fixed outer rail part.",
    )
    ctx.check(
        "intermediate rail present",
        "intermediate_rail" in part_names,
        "Expected intermediate rail part.",
    )
    ctx.check(
        "terminal carriage present",
        "terminal_carriage" in part_names,
        "Expected terminal carriage part.",
    )
    ctx.check(
        "outer to intermediate joint present",
        "outer_to_intermediate" in joint_names,
        "Expected first prismatic stage articulation.",
    )
    ctx.check(
        "intermediate to carriage joint present",
        "intermediate_to_carriage" in joint_names,
        "Expected second prismatic stage articulation.",
    )
    ctx.check(
        "slide stages use aligned prismatic axes",
        outer_to_intermediate.articulation_type == ArticulationType.PRISMATIC
        and intermediate_to_carriage.articulation_type == ArticulationType.PRISMATIC
        and tuple(outer_to_intermediate.axis) == (1.0, 0.0, 0.0)
        and tuple(intermediate_to_carriage.axis) == (1.0, 0.0, 0.0),
        "Both slide stages should translate along the same +X axis.",
    )

    with ctx.pose({outer_to_intermediate: 0.0, intermediate_to_carriage: 0.0}):
        ctx.expect_contact(
            intermediate_rail,
            outer_rail,
            name="intermediate rail is supported by outer rail",
        )
        ctx.expect_contact(
            terminal_carriage,
            intermediate_rail,
            name="carriage is supported by intermediate rail",
        )
        ctx.expect_overlap(
            intermediate_rail,
            outer_rail,
            axes="x",
            min_overlap=0.20,
            name="stowed intermediate overlap is substantial",
        )
        ctx.expect_overlap(
            terminal_carriage,
            intermediate_rail,
            axes="x",
            min_overlap=0.14,
            name="stowed carriage overlap is substantial",
        )
        ctx.expect_within(
            intermediate_rail,
            outer_rail,
            axes="y",
            margin=0.0,
            name="intermediate rail stays laterally within outer rail",
        )
        ctx.expect_within(
            terminal_carriage,
            intermediate_rail,
            axes="y",
            margin=0.0,
            name="carriage stays laterally within intermediate rail",
        )
        ctx.expect_gap(
            intermediate_rail,
            outer_rail,
            axis="z",
            min_gap=0.0,
            max_gap=1e-6,
            max_penetration=0.0,
            name="intermediate rail seats on outer rail without penetration",
        )
        ctx.expect_gap(
            terminal_carriage,
            intermediate_rail,
            axis="z",
            min_gap=0.0,
            max_gap=1e-6,
            max_penetration=0.0,
            name="carriage seats on intermediate rail without penetration",
        )
        ctx.expect_origin_gap(
            intermediate_rail,
            outer_rail,
            axis="x",
            min_gap=STOWED_MID_OFFSET - 1e-6,
            max_gap=STOWED_MID_OFFSET + 1e-6,
            name="intermediate stage stowed offset",
        )
        ctx.expect_origin_gap(
            terminal_carriage,
            intermediate_rail,
            axis="x",
            min_gap=STOWED_CARRIAGE_OFFSET - 1e-6,
            max_gap=STOWED_CARRIAGE_OFFSET + 1e-6,
            name="carriage stage stowed offset",
        )

    with ctx.pose(
        {
            outer_to_intermediate: MID_TRAVEL,
            intermediate_to_carriage: CARRIAGE_TRAVEL,
        }
    ):
        ctx.expect_overlap(
            intermediate_rail,
            outer_rail,
            axes="x",
            min_overlap=0.10,
            name="extended intermediate stage retains rail engagement",
        )
        ctx.expect_overlap(
            terminal_carriage,
            intermediate_rail,
            axes="x",
            min_overlap=0.079,
            name="extended carriage retains rail engagement",
        )
        ctx.expect_origin_gap(
            intermediate_rail,
            outer_rail,
            axis="x",
            min_gap=STOWED_MID_OFFSET + MID_TRAVEL - 1e-6,
            max_gap=STOWED_MID_OFFSET + MID_TRAVEL + 1e-6,
            name="intermediate stage extension travel",
        )
        ctx.expect_origin_gap(
            terminal_carriage,
            intermediate_rail,
            axis="x",
            min_gap=STOWED_CARRIAGE_OFFSET + CARRIAGE_TRAVEL - 1e-6,
            max_gap=STOWED_CARRIAGE_OFFSET + CARRIAGE_TRAVEL + 1e-6,
            name="carriage stage extension travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
