from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


LOWER_STAGE_Z = 0.092
MIDDLE_STAGE_OFFSET_Z = 0.050
UPPER_STAGE_OFFSET_Z = 0.046
SHAFT_RADIUS = 0.020
BORE_RADIUS = 0.026
HUB_RADIUS = 0.085


def annulus(outer_radius: float, inner_radius: float, height: float, *, z0: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def cylinder(radius: float, height: float, *, z0: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").workplane(offset=z0).circle(radius).extrude(height)


def export_mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(shape, name, tolerance=0.0006, angular_tolerance=0.05)


def make_support_frame() -> cq.Workplane:
    base = cq.Workplane("XY").box(1.12, 0.48, 0.05, centered=(True, True, False))
    left_upright = (
        cq.Workplane("XY")
        .box(0.10, 0.14, 0.28, centered=(True, True, False))
        .translate((-0.46, 0.0, 0.05))
    )
    right_upright = (
        cq.Workplane("XY")
        .box(0.10, 0.14, 0.28, centered=(True, True, False))
        .translate((0.46, 0.0, 0.05))
    )
    bridge = (
        cq.Workplane("XY")
        .box(1.02, 0.14, 0.07, centered=(True, True, False))
        .translate((0.0, 0.0, 0.30))
    )
    lower_thrust_collar = annulus(HUB_RADIUS, BORE_RADIUS, 0.022, z0=0.05)
    central_shaft = cylinder(SHAFT_RADIUS, 0.29, z0=0.05)
    upper_bearing_boss = annulus(0.055, BORE_RADIUS, 0.045, z0=0.265)

    return (
        base.union(left_upright)
        .union(right_upright)
        .union(bridge)
        .union(lower_thrust_collar)
        .union(central_shaft)
        .union(upper_bearing_boss)
    )


def make_lower_stage() -> cq.Workplane:
    platter = annulus(0.39, BORE_RADIUS, 0.032, z0=-0.016)
    upper_rim = annulus(0.395, 0.325, 0.010, z0=0.016)
    hub = annulus(HUB_RADIUS, BORE_RADIUS, 0.054, z0=-0.020)
    top_recess = annulus(0.30, 0.115, 0.008, z0=0.008)
    return platter.union(upper_rim).union(hub).cut(top_recess)


def make_middle_ring() -> cq.Workplane:
    outer_ring = annulus(0.28, 0.17, 0.028, z0=-0.014)
    upper_rim = annulus(0.285, 0.225, 0.008, z0=0.014)
    hub = annulus(HUB_RADIUS, BORE_RADIUS, 0.048, z0=-0.016)
    spoke_x = cq.Workplane("XY").box(0.44, 0.046, 0.022)
    spoke_y = cq.Workplane("XY").box(0.046, 0.44, 0.022)
    center_bore = cylinder(BORE_RADIUS, 0.060, z0=-0.030)
    return outer_ring.union(upper_rim).union(hub).union(spoke_x).union(spoke_y).cut(center_bore)


def make_top_plate() -> cq.Workplane:
    plate = annulus(0.16, BORE_RADIUS, 0.022, z0=-0.011)
    top_rim = annulus(0.165, 0.108, 0.006, z0=0.011)
    hub = annulus(0.078, BORE_RADIUS, 0.030, z0=-0.014)
    dish_cut = annulus(0.135, 0.082, 0.004, z0=0.007)
    return plate.union(top_rim).union(hub).cut(dish_cut)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_coaxial_turntable")

    support_paint = model.material("support_paint", rgba=(0.17, 0.18, 0.20, 1.0))
    lower_finish = model.material("lower_finish", rgba=(0.28, 0.30, 0.34, 1.0))
    middle_finish = model.material("middle_finish", rgba=(0.60, 0.63, 0.67, 1.0))
    upper_finish = model.material("upper_finish", rgba=(0.73, 0.75, 0.78, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        export_mesh(make_support_frame(), "support_frame"),
        origin=Origin(),
        material=support_paint,
        name="support_frame_shell",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        export_mesh(make_lower_stage(), "lower_stage"),
        origin=Origin(),
        material=lower_finish,
        name="lower_stage_shell",
    )

    middle_ring = model.part("middle_ring")
    middle_ring.visual(
        export_mesh(make_middle_ring(), "middle_ring"),
        origin=Origin(),
        material=middle_finish,
        name="middle_ring_shell",
    )

    top_plate = model.part("top_plate")
    top_plate.visual(
        export_mesh(make_top_plate(), "top_plate"),
        origin=Origin(),
        material=upper_finish,
        name="top_plate_shell",
    )

    spin_limits = MotionLimits(
        effort=35.0,
        velocity=2.5,
        lower=-2.0 * pi,
        upper=2.0 * pi,
    )

    model.articulation(
        "support_to_lower",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_STAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=spin_limits,
    )
    model.articulation(
        "lower_to_middle",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=middle_ring,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_STAGE_OFFSET_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=spin_limits,
    )
    model.articulation(
        "middle_to_upper",
        ArticulationType.REVOLUTE,
        parent=middle_ring,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, UPPER_STAGE_OFFSET_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=spin_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    lower_stage = object_model.get_part("lower_stage")
    middle_ring = object_model.get_part("middle_ring")
    top_plate = object_model.get_part("top_plate")
    support_to_lower = object_model.get_articulation("support_to_lower")
    lower_to_middle = object_model.get_articulation("lower_to_middle")
    middle_to_upper = object_model.get_articulation("middle_to_upper")

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

    joints = (support_to_lower, lower_to_middle, middle_to_upper)
    axes_ok = all(tuple(joint.axis) == (0.0, 0.0, 1.0) for joint in joints)
    origins_on_line_ok = all(
        abs(joint.origin.xyz[0]) < 1e-9 and abs(joint.origin.xyz[1]) < 1e-9 for joint in joints
    )
    all_revolute = all(joint.articulation_type == ArticulationType.REVOLUTE for joint in joints)
    ctx.check(
        "coaxial revolute stage joints",
        all_revolute and axes_ok and origins_on_line_ok,
        details="Expected three revolute joints sharing the same centered vertical axis.",
    )

    ctx.expect_contact(
        lower_stage,
        support_frame,
        contact_tol=0.002,
        name="lower stage thrust face supported by frame",
    )
    ctx.expect_contact(
        middle_ring,
        lower_stage,
        contact_tol=0.002,
        name="middle ring supported by lower stage hub",
    )
    ctx.expect_contact(
        top_plate,
        middle_ring,
        contact_tol=0.002,
        name="upper plate supported by middle ring hub",
    )

    ctx.expect_origin_distance(
        lower_stage,
        support_frame,
        axes="xy",
        max_dist=0.001,
        name="lower stage stays centered on shaft line",
    )
    ctx.expect_origin_distance(
        middle_ring,
        lower_stage,
        axes="xy",
        max_dist=0.001,
        name="middle ring stays centered on lower stage",
    )
    ctx.expect_origin_distance(
        top_plate,
        middle_ring,
        axes="xy",
        max_dist=0.001,
        name="upper plate stays centered on middle ring",
    )

    ctx.expect_within(
        middle_ring,
        lower_stage,
        axes="xy",
        margin=0.001,
        name="middle ring footprint stays inside lower stage",
    )
    ctx.expect_within(
        top_plate,
        middle_ring,
        axes="xy",
        margin=0.001,
        name="top plate footprint stays inside middle ring envelope",
    )

    lower_aabb = ctx.part_world_aabb(lower_stage)
    middle_aabb = ctx.part_world_aabb(middle_ring)
    upper_aabb = ctx.part_world_aabb(top_plate)
    if lower_aabb and middle_aabb and upper_aabb:
        lower_span = lower_aabb[1][0] - lower_aabb[0][0]
        middle_span = middle_aabb[1][0] - middle_aabb[0][0]
        upper_span = upper_aabb[1][0] - upper_aabb[0][0]
        ctx.check(
            "stage diameters step down upward",
            lower_span > middle_span > upper_span,
            details=(
                f"Expected descending stage diameters, got lower={lower_span:.3f}, "
                f"middle={middle_span:.3f}, upper={upper_span:.3f}."
            ),
        )

    with ctx.pose(
        {
            support_to_lower: 0.9,
            lower_to_middle: -0.7,
            middle_to_upper: 1.1,
        }
    ):
        ctx.expect_contact(
            lower_stage,
            support_frame,
            contact_tol=0.002,
            name="lower stage remains supported while rotated",
        )
        ctx.expect_contact(
            middle_ring,
            lower_stage,
            contact_tol=0.002,
            name="middle ring remains supported while rotated",
        )
        ctx.expect_contact(
            top_plate,
            middle_ring,
            contact_tol=0.002,
            name="upper plate remains supported while rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
