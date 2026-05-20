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


PLATE_LENGTH = 0.48
PLATE_HEIGHT = 0.11
PLATE_THICKNESS = 0.003
PLATE_SLOT_LENGTH = 0.024
PLATE_SLOT_WIDTH = 0.007
PLATE_SLOT_POSITIONS = (
    (0.08, 0.028),
    (0.24, -0.026),
    (0.40, 0.024),
)

CHANNEL_OFFSET_X = 0.03
OUTER_LENGTH = 0.42
OUTER_HEIGHT = 0.042
OUTER_DEPTH = 0.016
OUTER_WEB = 0.002
OUTER_FLANGE = 0.002

INNER_LENGTH = 0.46
INNER_CORE_DEPTH = 0.0056
INNER_CORE_HEIGHT = 0.033
INNER_SIDE_CLEARANCE = 0.0
INNER_PAD_DEPTH = OUTER_DEPTH - OUTER_WEB - 0.002 - 2.0 * INNER_SIDE_CLEARANCE
INNER_PAD_THICKNESS = 0.0032
INNER_PAD_Z = (
    OUTER_HEIGHT / 2.0
    - OUTER_FLANGE
    - INNER_SIDE_CLEARANCE
    - INNER_PAD_THICKNESS / 2.0
)
INNER_TRAVEL = 0.26


def _box(length: float, depth: float, height: float, *, x0: float = 0.0, y0: float = 0.0, z0: float = 0.0):
    return cq.Workplane("XY").box(length, depth, height).translate(
        (x0 + length / 2.0, y0 + depth / 2.0, z0 + height / 2.0)
    )


def _make_side_plate() -> cq.Workplane:
    plate = _box(
        PLATE_LENGTH,
        PLATE_THICKNESS,
        PLATE_HEIGHT,
        x0=0.0,
        y0=-PLATE_THICKNESS / 2.0,
        z0=-PLATE_HEIGHT / 2.0,
    )
    for slot_x, slot_z in PLATE_SLOT_POSITIONS:
        slot = (
            cq.Workplane("XZ")
            .center(slot_x, slot_z)
            .slot2D(PLATE_SLOT_LENGTH, PLATE_SLOT_WIDTH)
            .extrude(PLATE_THICKNESS * 3.0, both=True)
        )
        plate = plate.cut(slot)
    return plate
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_slide_extension")

    model.material("plate_gray", rgba=(0.33, 0.35, 0.38, 1.0))
    model.material("outer_steel", rgba=(0.58, 0.61, 0.66, 1.0))
    model.material("inner_steel", rgba=(0.74, 0.76, 0.79, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        mesh_from_cadquery(_make_side_plate(), "side_plate"),
        material="plate_gray",
        name="plate_shell",
    )

    outer_channel = model.part("outer_channel")
    outer_channel.visual(
        Box((OUTER_LENGTH, OUTER_WEB, OUTER_HEIGHT)),
        material="outer_steel",
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, OUTER_WEB / 2.0, 0.0)),
        name="back_web",
    )
    outer_channel.visual(
        Box((OUTER_LENGTH, OUTER_DEPTH - OUTER_WEB, OUTER_FLANGE)),
        material="outer_steel",
        origin=Origin(
            xyz=(
                OUTER_LENGTH / 2.0,
                OUTER_WEB + (OUTER_DEPTH - OUTER_WEB) / 2.0,
                OUTER_HEIGHT / 2.0 - OUTER_FLANGE / 2.0,
            )
        ),
        name="top_flange",
    )
    outer_channel.visual(
        Box((OUTER_LENGTH, OUTER_DEPTH - OUTER_WEB, OUTER_FLANGE)),
        material="outer_steel",
        origin=Origin(
            xyz=(
                OUTER_LENGTH / 2.0,
                OUTER_WEB + (OUTER_DEPTH - OUTER_WEB) / 2.0,
                -OUTER_HEIGHT / 2.0 + OUTER_FLANGE / 2.0,
            )
        ),
        name="bottom_flange",
    )

    inner_runner = model.part("inner_runner")
    inner_runner.visual(
        Box((INNER_LENGTH, INNER_CORE_DEPTH, INNER_CORE_HEIGHT)),
        material="inner_steel",
        origin=Origin(xyz=(INNER_LENGTH / 2.0, INNER_CORE_DEPTH / 2.0, 0.0)),
        name="runner_core",
    )
    inner_runner.visual(
        Box((INNER_LENGTH, INNER_PAD_DEPTH, INNER_PAD_THICKNESS)),
        material="inner_steel",
        origin=Origin(xyz=(INNER_LENGTH / 2.0, INNER_PAD_DEPTH / 2.0, INNER_PAD_Z)),
        name="top_shoe",
    )
    inner_runner.visual(
        Box((INNER_LENGTH, INNER_PAD_DEPTH, INNER_PAD_THICKNESS)),
        material="inner_steel",
        origin=Origin(xyz=(INNER_LENGTH / 2.0, INNER_PAD_DEPTH / 2.0, -INNER_PAD_Z)),
        name="bottom_shoe",
    )

    model.articulation(
        "plate_to_outer_channel",
        ArticulationType.FIXED,
        parent=side_plate,
        child=outer_channel,
        origin=Origin(xyz=(CHANNEL_OFFSET_X, PLATE_THICKNESS / 2.0, 0.0)),
    )
    model.articulation(
        "outer_channel_to_inner_runner",
        ArticulationType.PRISMATIC,
        parent=outer_channel,
        child=inner_runner,
        origin=Origin(xyz=(0.0, OUTER_WEB + INNER_SIDE_CLEARANCE, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=INNER_TRAVEL,
            effort=120.0,
            velocity=0.45,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    outer_channel = object_model.get_part("outer_channel")
    inner_runner = object_model.get_part("inner_runner")
    slide = object_model.get_articulation("outer_channel_to_inner_runner")

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

    ctx.expect_contact(
        outer_channel,
        side_plate,
        name="outer channel is mounted against side plate",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            inner_runner,
            outer_channel,
            name="inner runner is supported by outer channel when closed",
        )
        ctx.expect_within(
            inner_runner,
            outer_channel,
            axes="yz",
            margin=0.0,
            name="closed runner cross-section stays inside outer channel envelope",
        )
        ctx.expect_overlap(
            inner_runner,
            outer_channel,
            axes="x",
            min_overlap=0.40,
            name="closed runner remains deeply engaged in outer channel",
        )

    with ctx.pose({slide: INNER_TRAVEL}):
        ctx.expect_contact(
            inner_runner,
            outer_channel,
            name="inner runner remains supported at full extension",
        )
        ctx.expect_within(
            inner_runner,
            outer_channel,
            axes="yz",
            margin=0.0,
            name="extended runner cross-section stays inside outer channel envelope",
        )
        ctx.expect_overlap(
            inner_runner,
            outer_channel,
            axes="x",
            min_overlap=0.15,
            name="extended runner keeps meaningful overlap with outer channel",
        )

    with ctx.pose({slide: 0.0}):
        closed_pos = ctx.part_world_position(inner_runner)
    with ctx.pose({slide: INNER_TRAVEL}):
        open_pos = ctx.part_world_position(inner_runner)
    moved_outward = (
        closed_pos is not None
        and open_pos is not None
        and (open_pos[0] - closed_pos[0]) > INNER_TRAVEL * 0.99
    )
    ctx.check(
        "inner runner slides outward along +X",
        moved_outward,
        details=f"closed={closed_pos}, open={open_pos}, expected travel≈{INNER_TRAVEL}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
