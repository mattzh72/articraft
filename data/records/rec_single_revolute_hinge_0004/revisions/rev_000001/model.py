from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

HINGE_LENGTH = 0.120
LEAF_WIDTH = 0.050
PLATE_THICKNESS = 0.006
KNUCKLE_RADIUS = 0.012
PIN_RADIUS = 0.005
PIN_LENGTH = HINGE_LENGTH + 0.008
KNUCKLE_SEGMENT_LENGTH = 0.022
KNUCKLE_GAP = 0.002
BLOCK_THICKNESS = 0.040
BLOCK_WIDTH = 0.050
BLOCK_HEIGHT = 0.136


def _segment_starts(count: int) -> list[float]:
    start = -HINGE_LENGTH / 2.0
    pitch = KNUCKLE_SEGMENT_LENGTH + KNUCKLE_GAP
    return [start + i * pitch for i in range(count)]


def _make_fixed_knuckles() -> cq.Workplane:
    knuckles = None
    for index, start_z in enumerate(_segment_starts(5)):
        if index % 2 == 0:
            bridge = cq.Workplane("XY").box(
                PLATE_THICKNESS,
                KNUCKLE_RADIUS,
                KNUCKLE_SEGMENT_LENGTH,
            ).translate(
                (
                    -PLATE_THICKNESS / 2.0,
                    -KNUCKLE_RADIUS / 2.0,
                    start_z + KNUCKLE_SEGMENT_LENGTH / 2.0,
                )
            )
            outer_barrel = cq.Workplane("XY").circle(KNUCKLE_RADIUS).extrude(KNUCKLE_SEGMENT_LENGTH).translate(
                (0.0, 0.0, start_z)
            )
            inner_bore = cq.Workplane("XY").circle(PIN_RADIUS).extrude(KNUCKLE_SEGMENT_LENGTH + 0.002).translate(
                (0.0, 0.0, start_z - 0.001)
            )
            barrel = outer_barrel.cut(inner_bore)
            knuckle = bridge.union(barrel)
            knuckles = knuckle if knuckles is None else knuckles.union(knuckle)

    return knuckles if knuckles is not None else cq.Workplane("XY")


def _make_swing_knuckles() -> cq.Workplane:
    knuckles = None
    for index, start_z in enumerate(_segment_starts(5)):
        if index % 2 == 1:
            bridge = cq.Workplane("XY").box(
                PLATE_THICKNESS,
                KNUCKLE_RADIUS,
                KNUCKLE_SEGMENT_LENGTH,
            ).translate(
                (
                    PLATE_THICKNESS / 2.0,
                    KNUCKLE_RADIUS / 2.0,
                    start_z + KNUCKLE_SEGMENT_LENGTH / 2.0,
                )
            )
            outer_barrel = cq.Workplane("XY").circle(KNUCKLE_RADIUS).extrude(KNUCKLE_SEGMENT_LENGTH).translate(
                (0.0, 0.0, start_z)
            )
            inner_bore = cq.Workplane("XY").circle(PIN_RADIUS + 0.0012).extrude(KNUCKLE_SEGMENT_LENGTH + 0.002).translate(
                (0.0, 0.0, start_z - 0.001)
            )
            sleeve = outer_barrel.cut(inner_bore)
            knuckle = bridge.union(sleeve)
            knuckles = knuckle if knuckles is None else knuckles.union(knuckle)

    return knuckles if knuckles is not None else cq.Workplane("XY")


def _build_block_origin() -> Origin:
    return Origin(
        xyz=(
            -(BLOCK_THICKNESS / 2.0 + PLATE_THICKNESS),
            -(KNUCKLE_RADIUS + LEAF_WIDTH / 2.0),
            0.0,
        )
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_hinge", assets=ASSETS)

    black_oxide = model.material("black_oxide", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.42, 0.44, 0.47, 1.0))

    mounting_block = model.part("mounting_block")
    mounting_block.visual(
        Box((BLOCK_THICKNESS, BLOCK_WIDTH, BLOCK_HEIGHT)),
        origin=_build_block_origin(),
        material=black_oxide,
        name="block_body",
    )
    mounting_block.inertial = Inertial.from_geometry(
        Box((BLOCK_THICKNESS, BLOCK_WIDTH, BLOCK_HEIGHT)),
        mass=1.8,
        origin=_build_block_origin(),
    )

    fixed_leaf = model.part("fixed_leaf")
    fixed_leaf.visual(
        Box((PLATE_THICKNESS, LEAF_WIDTH, HINGE_LENGTH)),
        origin=Origin(
            xyz=(
                -PLATE_THICKNESS / 2.0,
                -(KNUCKLE_RADIUS + LEAF_WIDTH / 2.0),
                0.0,
            )
        ),
        material=steel,
        name="fixed_plate",
    )
    fixed_leaf.visual(
        mesh_from_cadquery(_make_fixed_knuckles(), "fixed_knuckles.obj", assets=ASSETS),
        material=steel,
        name="fixed_knuckles",
    )
    fixed_leaf.inertial = Inertial.from_geometry(
        Box((0.032, 0.072, PIN_LENGTH)),
        mass=1.0,
        origin=Origin(xyz=(-0.002, -0.020, 0.0)),
    )

    swing_leaf = model.part("swing_leaf")
    swing_leaf.visual(
        Box((PLATE_THICKNESS, LEAF_WIDTH, HINGE_LENGTH)),
        origin=Origin(
            xyz=(
                PLATE_THICKNESS / 2.0,
                KNUCKLE_RADIUS + LEAF_WIDTH / 2.0,
                0.0,
            )
        ),
        material=steel,
        name="swing_plate",
    )
    swing_leaf.visual(
        mesh_from_cadquery(_make_swing_knuckles(), "swing_knuckles.obj", assets=ASSETS),
        material=steel,
        name="swing_knuckles",
    )
    swing_leaf.inertial = Inertial.from_geometry(
        Box((0.028, 0.072, HINGE_LENGTH)),
        mass=0.8,
        origin=Origin(xyz=(0.002, 0.020, 0.0)),
    )

    pin = model.part("pin")
    pin.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        material=black_oxide,
        name="pin_body",
    )
    pin.inertial = Inertial.from_geometry(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        mass=0.22,
    )

    model.articulation(
        "block_to_fixed_leaf",
        ArticulationType.FIXED,
        parent=mounting_block,
        child=fixed_leaf,
        origin=Origin(),
    )
    model.articulation(
        "fixed_leaf_to_pin",
        ArticulationType.FIXED,
        parent=fixed_leaf,
        child=pin,
        origin=Origin(),
    )
    model.articulation(
        "pin_to_swing_leaf",
        ArticulationType.REVOLUTE,
        parent=pin,
        child=swing_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(175.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    mounting_block = object_model.get_part("mounting_block")
    fixed_leaf = object_model.get_part("fixed_leaf")
    pin = object_model.get_part("pin")
    swing_leaf = object_model.get_part("swing_leaf")
    hinge = object_model.get_articulation("pin_to_swing_leaf")

    block_body = mounting_block.get_visual("block_body")
    fixed_plate = fixed_leaf.get_visual("fixed_plate")
    fixed_knuckles = fixed_leaf.get_visual("fixed_knuckles")
    pin_body = pin.get_visual("pin_body")
    swing_plate = swing_leaf.get_visual("swing_plate")
    swing_knuckles = swing_leaf.get_visual("swing_knuckles")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.allow_overlap(
        pin,
        fixed_leaf,
        elem_a=pin_body,
        elem_b=fixed_knuckles,
        reason="Pin is press-fit into the fixed leaf knuckles.",
    )
    ctx.allow_overlap(
        pin,
        swing_leaf,
        elem_a=pin_body,
        elem_b=swing_knuckles,
        reason="Swing leaf rotates around the central pin, so the knuckle sleeves envelop it.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=15)

    ctx.expect_contact(
        fixed_leaf,
        mounting_block,
        elem_a=fixed_plate,
        elem_b=block_body,
        contact_tol=5e-4,
        name="fixed_leaf_contacts_mounting_block",
    )
    ctx.expect_gap(
        fixed_leaf,
        mounting_block,
        axis="x",
        positive_elem=fixed_plate,
        negative_elem=block_body,
        max_gap=5e-4,
        max_penetration=0.0,
        name="fixed_leaf_sits_flush_on_block",
    )
    ctx.expect_overlap(
        fixed_leaf,
        mounting_block,
        axes="yz",
        elem_a=fixed_plate,
        elem_b=block_body,
        min_overlap=0.040,
        name="mounting_block_covers_fixed_leaf_footprint",
    )
    ctx.expect_contact(
        pin,
        fixed_leaf,
        elem_a=pin_body,
        elem_b=fixed_knuckles,
        contact_tol=5e-4,
        name="pin_seats_in_fixed_knuckles",
    )
    ctx.expect_gap(
        swing_leaf,
        pin,
        axis="x",
        positive_elem=swing_knuckles,
        negative_elem=pin_body,
        min_gap=-0.020,
        name="swing_knuckles_wrap_pin_axis",
    )
    ctx.expect_overlap(
        swing_leaf,
        pin,
        axes="z",
        elem_a=swing_knuckles,
        elem_b=pin_body,
        min_overlap=0.068,
        name="pin_runs_through_swing_knuckles",
    )

    ctx.check(
        "hinge_axis_is_pin_axis",
        tuple(round(value, 6) for value in hinge.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical pin axis, got {hinge.axis}",
    )
    ctx.check(
        "hinge_motion_limits_span_half_turn",
        hinge.motion_limits is not None
        and abs(hinge.motion_limits.lower - 0.0) < 1e-6
        and abs(hinge.motion_limits.upper - math.radians(175.0)) < 1e-6,
        details=f"expected 0..pi limits, got {hinge.motion_limits}",
    )

    with ctx.pose({hinge: 0.0}):
        swing_open_aabb = ctx.part_world_aabb(swing_leaf)
        ctx.check(
            "swing_leaf_open_pose_reaches_positive_side",
            swing_open_aabb is not None
            and swing_open_aabb[1][1] > 0.055
            and swing_open_aabb[0][1] > -0.0135,
            details=f"unexpected open-pose bounds: {swing_open_aabb}",
        )
        ctx.expect_overlap(
            swing_leaf,
            pin,
            axes="z",
            elem_a=swing_knuckles,
            elem_b=pin_body,
            min_overlap=0.068,
            name="knuckles_share_pin_span_open",
        )

    with ctx.pose({hinge: math.pi / 2.0}):
        swing_mid_aabb = ctx.part_world_aabb(swing_leaf)
        ctx.check(
            "swing_leaf_mid_pose_swings_around_pin",
            swing_mid_aabb is not None
            and swing_mid_aabb[0][0] < -0.055
            and swing_mid_aabb[1][1] < 0.015,
            details=f"unexpected mid-pose bounds: {swing_mid_aabb}",
        )

    with ctx.pose({hinge: math.radians(175.0)}):
        swing_closed_aabb = ctx.part_world_aabb(swing_leaf)
        ctx.check(
            "swing_leaf_closed_pose_folds_back_over_fixed_side",
            swing_closed_aabb is not None
            and swing_closed_aabb[0][1] < -0.055
            and swing_closed_aabb[1][1] < 0.0135,
            details=f"unexpected closed-pose bounds: {swing_closed_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
