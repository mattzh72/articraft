from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.23
BASE_FOOT_HEIGHT = 0.018
BASE_DECK_HEIGHT = 0.012
BASE_DECK_TOP = BASE_FOOT_HEIGHT + BASE_DECK_HEIGHT

OUTER_LAND_INNER_RADIUS = 0.118
OUTER_LAND_OUTER_RADIUS = 0.188
OUTER_LAND_TOP = 0.050

MID_LAND_INNER_RADIUS = 0.058
MID_LAND_OUTER_RADIUS = 0.088
MID_LAND_TOP = 0.082

CUP_LAND_RADIUS = 0.034
CUP_LAND_TOP = 0.114

PLATTER_INNER_RADIUS = 0.106
PLATTER_OUTER_RADIUS = 0.205
PLATTER_HEIGHT = 0.018

RING_INNER_RADIUS = 0.052
RING_OUTER_RADIUS = 0.099
RING_HEIGHT = 0.056

CUP_OUTER_RADIUS = 0.044
CUP_INNER_RADIUS = 0.032
CUP_HEIGHT = 0.058
CUP_FLOOR = 0.006


def revolved_section(profile: list[tuple[float, float]]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .revolve(360, (0.0, 0.0), (0.0, 1.0))
    )


def make_base_shape() -> cq.Workplane:
    plinth = revolved_section(
        [
            (0.0, 0.0),
            (BASE_RADIUS - 0.010, 0.0),
            (BASE_RADIUS, 0.005),
            (BASE_RADIUS, BASE_FOOT_HEIGHT - 0.003),
            (BASE_RADIUS - 0.012, BASE_FOOT_HEIGHT),
            (0.0, BASE_FOOT_HEIGHT),
        ]
    )
    deck = revolved_section(
        [
            (0.0, BASE_FOOT_HEIGHT),
            (0.208, BASE_FOOT_HEIGHT),
            (0.210, BASE_FOOT_HEIGHT + 0.002),
            (0.210, BASE_DECK_TOP - 0.001),
            (0.205, BASE_DECK_TOP),
            (0.0, BASE_DECK_TOP),
        ]
    )
    outer_land = revolved_section(
        [
            (OUTER_LAND_INNER_RADIUS, BASE_DECK_TOP + 0.002),
            (OUTER_LAND_INNER_RADIUS + 0.004, BASE_DECK_TOP),
            (OUTER_LAND_OUTER_RADIUS - 0.004, BASE_DECK_TOP),
            (OUTER_LAND_OUTER_RADIUS, BASE_DECK_TOP + 0.002),
            (OUTER_LAND_OUTER_RADIUS, OUTER_LAND_TOP - 0.002),
            (OUTER_LAND_OUTER_RADIUS - 0.005, OUTER_LAND_TOP),
            (OUTER_LAND_INNER_RADIUS + 0.005, OUTER_LAND_TOP),
            (OUTER_LAND_INNER_RADIUS, OUTER_LAND_TOP - 0.003),
        ]
    )
    mid_land = revolved_section(
        [
            (MID_LAND_INNER_RADIUS, BASE_DECK_TOP + 0.002),
            (MID_LAND_INNER_RADIUS + 0.004, BASE_DECK_TOP),
            (MID_LAND_OUTER_RADIUS - 0.004, BASE_DECK_TOP),
            (MID_LAND_OUTER_RADIUS, BASE_DECK_TOP + 0.002),
            (MID_LAND_OUTER_RADIUS, MID_LAND_TOP - 0.002),
            (MID_LAND_OUTER_RADIUS - 0.004, MID_LAND_TOP),
            (MID_LAND_INNER_RADIUS + 0.004, MID_LAND_TOP),
            (MID_LAND_INNER_RADIUS, MID_LAND_TOP - 0.003),
        ]
    )
    cup_land = revolved_section(
        [
            (0.0, BASE_DECK_TOP),
            (CUP_LAND_RADIUS, BASE_DECK_TOP),
            (CUP_LAND_RADIUS, CUP_LAND_TOP - 0.002),
            (CUP_LAND_RADIUS - 0.003, CUP_LAND_TOP),
            (0.0, CUP_LAND_TOP),
        ]
    )

    return plinth.union(deck).union(outer_land).union(mid_land).union(cup_land)


def make_platter_shape() -> cq.Workplane:
    return revolved_section(
        [
            (PLATTER_INNER_RADIUS, 0.004),
            (PLATTER_INNER_RADIUS + 0.010, 0.0),
            (PLATTER_OUTER_RADIUS - 0.006, 0.0),
            (PLATTER_OUTER_RADIUS, 0.004),
            (PLATTER_OUTER_RADIUS, PLATTER_HEIGHT - 0.003),
            (PLATTER_OUTER_RADIUS - 0.008, PLATTER_HEIGHT),
            (PLATTER_INNER_RADIUS + 0.016, PLATTER_HEIGHT),
            (PLATTER_INNER_RADIUS, PLATTER_HEIGHT - 0.006),
        ]
    )


def make_intermediate_ring_shape() -> cq.Workplane:
    return revolved_section(
        [
            (RING_INNER_RADIUS, 0.006),
            (RING_INNER_RADIUS + 0.008, 0.0),
            (RING_OUTER_RADIUS - 0.006, 0.0),
            (RING_OUTER_RADIUS, 0.006),
            (RING_OUTER_RADIUS, RING_HEIGHT - 0.006),
            (RING_OUTER_RADIUS - 0.008, RING_HEIGHT),
            (RING_INNER_RADIUS + 0.012, RING_HEIGHT),
            (RING_INNER_RADIUS, RING_HEIGHT - 0.010),
        ]
    )


def make_fixture_cup_shape() -> cq.Workplane:
    return revolved_section(
        [
            (0.0, 0.0),
            (CUP_OUTER_RADIUS - 0.004, 0.0),
            (CUP_OUTER_RADIUS, 0.004),
            (CUP_OUTER_RADIUS, CUP_HEIGHT - 0.004),
            (CUP_OUTER_RADIUS - 0.004, CUP_HEIGHT),
            (CUP_INNER_RADIUS, CUP_HEIGHT),
            (CUP_INNER_RADIUS, CUP_FLOOR),
            (0.0, CUP_FLOOR),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_ring_turntable")

    base_color = model.material("base_graphite", rgba=(0.15, 0.16, 0.18, 1.0))
    platter_color = model.material("platter_aluminum", rgba=(0.71, 0.73, 0.76, 1.0))
    ring_color = model.material("ring_charcoal", rgba=(0.34, 0.36, 0.39, 1.0))
    cup_color = model.material("cup_blackened_steel", rgba=(0.22, 0.22, 0.24, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_shape(), "base_housing"),
        material=base_color,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=CUP_LAND_TOP),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, CUP_LAND_TOP / 2.0)),
    )

    platter = model.part("platter")
    platter.visual(
        mesh_from_cadquery(make_platter_shape(), "platter_stage"),
        material=platter_color,
        name="platter_shell",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=PLATTER_OUTER_RADIUS, length=PLATTER_HEIGHT),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, PLATTER_HEIGHT / 2.0)),
    )

    ring = model.part("intermediate_ring")
    ring.visual(
        mesh_from_cadquery(make_intermediate_ring_shape(), "intermediate_ring_stage"),
        material=ring_color,
        name="ring_shell",
    )
    ring.inertial = Inertial.from_geometry(
        Cylinder(radius=RING_OUTER_RADIUS, length=RING_HEIGHT),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, RING_HEIGHT / 2.0)),
    )

    cup = model.part("fixture_cup")
    cup.visual(
        mesh_from_cadquery(make_fixture_cup_shape(), "fixture_cup_stage"),
        material=cup_color,
        name="cup_shell",
    )
    cup.inertial = Inertial.from_geometry(
        Cylinder(radius=CUP_OUTER_RADIUS, length=CUP_HEIGHT),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, CUP_HEIGHT / 2.0)),
    )

    model.articulation(
        "base_to_platter",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, OUTER_LAND_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=3.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "base_to_intermediate_ring",
        ArticulationType.REVOLUTE,
        parent=base,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, MID_LAND_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "base_to_fixture_cup",
        ArticulationType.REVOLUTE,
        parent=base,
        child=cup,
        origin=Origin(xyz=(0.0, 0.0, CUP_LAND_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platter = object_model.get_part("platter")
    ring = object_model.get_part("intermediate_ring")
    cup = object_model.get_part("fixture_cup")
    platter_joint = object_model.get_articulation("base_to_platter")
    ring_joint = object_model.get_articulation("base_to_intermediate_ring")
    cup_joint = object_model.get_articulation("base_to_fixture_cup")

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
    #
    # For this coaxial stacked turntable, the whole-part overlap backstop is
    # too coarse because the base part contains several vertically separated
    # bearing lands in one mesh-backed housing. The exact mounting and nesting
    # checks below encode the intended clearances more faithfully.

    expected_parts = {"base", "platter", "intermediate_ring", "fixture_cup"}
    expected_joints = {
        "base_to_platter",
        "base_to_intermediate_ring",
        "base_to_fixture_cup",
    }
    present_parts = {part.name for part in object_model.parts}
    present_joints = {joint.name for joint in object_model.articulations}
    ctx.check(
        "all_expected_parts_present",
        present_parts == expected_parts,
        f"expected parts {sorted(expected_parts)}, got {sorted(present_parts)}",
    )
    ctx.check(
        "all_expected_joints_present",
        present_joints == expected_joints,
        f"expected joints {sorted(expected_joints)}, got {sorted(present_joints)}",
    )

    stage_joints = (platter_joint, ring_joint, cup_joint)
    ctx.check(
        "all_stage_axes_vertical",
        all(joint.axis == (0.0, 0.0, 1.0) for joint in stage_joints),
        "every stage joint should rotate about the common +Z axis",
    )
    ctx.check(
        "all_stage_limits_are_full_turntable_swings",
        all(
            joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.lower <= -3.0
            and joint.motion_limits.upper >= 3.0
            for joint in stage_joints
        ),
        "each stage should have generous revolute travel around the upright centerline",
    )

    ctx.expect_contact(base, platter, contact_tol=5e-4, name="platter_supported_on_base_land")
    ctx.expect_contact(base, ring, contact_tol=5e-4, name="ring_supported_on_base_land")
    ctx.expect_contact(base, cup, contact_tol=5e-4, name="cup_supported_on_base_land")

    ctx.expect_origin_distance(
        platter,
        ring,
        axes="xy",
        max_dist=1e-6,
        name="platter_and_ring_share_centerline",
    )
    ctx.expect_origin_distance(
        ring,
        cup,
        axes="xy",
        max_dist=1e-6,
        name="ring_and_cup_share_centerline",
    )
    ctx.expect_gap(
        ring,
        platter,
        axis="z",
        min_gap=0.010,
        name="ring_reads_as_stage_above_platter",
    )
    ctx.expect_within(
        ring,
        platter,
        axes="xy",
        margin=0.0,
        name="ring_is_nested_inside_platter_footprint",
    )
    ctx.expect_within(
        cup,
        ring,
        axes="xy",
        margin=0.0,
        name="cup_is_nested_inside_ring_footprint",
    )

    with ctx.pose(
        {
            platter_joint: 1.10,
            ring_joint: -0.85,
            cup_joint: 0.65,
        }
    ):
        ctx.expect_contact(base, platter, contact_tol=5e-4, name="platter_stays_supported_when_rotated")
        ctx.expect_contact(base, ring, contact_tol=5e-4, name="ring_stays_supported_when_rotated")
        ctx.expect_contact(base, cup, contact_tol=5e-4, name="cup_stays_supported_when_rotated")
        ctx.expect_gap(
            ring,
            platter,
            axis="z",
            min_gap=0.010,
            name="ring_stays_above_platter_when_rotated",
        )
        ctx.expect_within(
            ring,
            platter,
            axes="xy",
            margin=0.0,
            name="ring_stays_nested_inside_platter_when_rotated",
        )
        ctx.expect_within(
            cup,
            ring,
            axes="xy",
            margin=0.0,
            name="cup_stays_nested_inside_ring_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
