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
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="street_single_post_floodlight")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    bracket_dark = model.material("dark_bracket", rgba=(0.30, 0.32, 0.35, 1.0))
    housing_dark = model.material("housing_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.08, 0.09, 0.10, 1.0))
    glass = model.material("lens_glass", rgba=(0.73, 0.81, 0.89, 0.42))

    base_plate_size = 0.38
    base_plate_thickness = 0.03
    pole_height = 6.20
    pole_bottom_radius = 0.11
    pole_top_radius = 0.065
    pole_top_z = base_plate_thickness + pole_height

    hinge_x = 1.12
    hinge_z = pole_top_z - 0.02
    shaft_length = 0.80
    yoke_plate_thickness = 0.03

    support = model.part("support")
    support.visual(
        Box((base_plate_size, base_plate_size, base_plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_plate_thickness * 0.5)),
        material=galvanized,
        name="base_plate",
    )
    support.visual(
        Cylinder(radius=0.135, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, base_plate_thickness + 0.0425)),
        material=galvanized,
        name="base_collar",
    )

    pole_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.0),
                (pole_bottom_radius, 0.0),
                (pole_top_radius, pole_height),
                (0.0, pole_height),
            ],
            segments=48,
            closed=True,
        ),
        "pole_shell",
    )
    support.visual(
        pole_mesh,
        origin=Origin(xyz=(0.0, 0.0, base_plate_thickness)),
        material=galvanized,
        name="pole_shell",
    )

    arm_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, pole_top_z - 0.24),
                (0.07, 0.0, pole_top_z - 0.06),
                (0.42, 0.0, pole_top_z + 0.18),
                (0.82, 0.0, pole_top_z + 0.08),
                (hinge_x - 0.07, 0.0, hinge_z + 0.01),
            ],
            radius=0.047,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        ),
        "outreach_arm",
    )
    support.visual(
        arm_mesh,
        material=galvanized,
        name="outreach_arm",
    )

    support.visual(
        Box((0.11, 0.14, 0.12)),
        origin=Origin(xyz=(hinge_x - 0.08, 0.0, hinge_z)),
        material=bracket_dark,
        name="yoke_neck",
    )
    support.visual(
        Box((0.10, 0.86, 0.05)),
        origin=Origin(xyz=(hinge_x - 0.07, 0.0, hinge_z + 0.045)),
        material=bracket_dark,
        name="yoke_crossbar",
    )
    support.visual(
        Box((0.10, yoke_plate_thickness, 0.18)),
        origin=Origin(
            xyz=(hinge_x, -(shaft_length * 0.5 + yoke_plate_thickness * 0.5), hinge_z - 0.04)
        ),
        material=bracket_dark,
        name="yoke_left",
    )
    support.visual(
        Box((0.10, yoke_plate_thickness, 0.18)),
        origin=Origin(
            xyz=(hinge_x, shaft_length * 0.5 + yoke_plate_thickness * 0.5, hinge_z - 0.04)
        ),
        material=bracket_dark,
        name="yoke_right",
    )

    head = model.part("flood_head")
    head.visual(
        Cylinder(radius=0.022, length=shaft_length),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=bracket_dark,
        name="tilt_shaft",
    )
    head.visual(
        Box((0.06, 0.70, 0.14)),
        origin=Origin(xyz=(0.025, 0.0, -0.07)),
        material=bracket_dark,
        name="hanger_block",
    )

    nominal_pitch = 0.42
    head.visual(
        Box((0.16, 0.76, 0.32)),
        origin=Origin(xyz=(0.12, 0.0, -0.19), rpy=(0.0, nominal_pitch, 0.0)),
        material=housing_dark,
        name="housing",
    )
    head.visual(
        Box((0.09, 0.56, 0.22)),
        origin=Origin(xyz=(0.02, 0.0, -0.18), rpy=(0.0, nominal_pitch, 0.0)),
        material=bracket_dark,
        name="rear_driver_box",
    )
    head.visual(
        Box((0.07, 0.80, 0.035)),
        origin=Origin(xyz=(0.155, 0.0, -0.03), rpy=(0.0, nominal_pitch, 0.0)),
        material=housing_dark,
        name="visor",
    )
    head.visual(
        Box((0.028, 0.78, 0.34)),
        origin=Origin(xyz=(0.197, 0.0, -0.19), rpy=(0.0, nominal_pitch, 0.0)),
        material=bezel_black,
        name="bezel",
    )
    head.visual(
        Box((0.008, 0.72, 0.26)),
        origin=Origin(xyz=(0.214, 0.0, -0.19), rpy=(0.0, nominal_pitch, 0.0)),
        material=glass,
        name="lens",
    )

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=support,
        child=head,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=-0.55,
            upper=0.60,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    head = object_model.get_part("flood_head")
    tilt = object_model.get_articulation("head_tilt")

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
        "head tilt axis is horizontal across the flood head",
        abs(tilt.axis[1]) > 0.99 and abs(tilt.axis[0]) < 1e-6 and abs(tilt.axis[2]) < 1e-6,
        details=f"axis={tilt.axis}",
    )
    ctx.expect_origin_distance(
        head,
        support,
        axes="x",
        min_dist=1.0,
        name="flood head sits outboard on the outreach arm",
    )

    ctx.expect_contact(
        head,
        support,
        elem_a="tilt_shaft",
        elem_b="yoke_left",
        name="left end of the tilt shaft is supported by the left yoke plate",
    )
    ctx.expect_contact(
        head,
        support,
        elem_a="tilt_shaft",
        elem_b="yoke_right",
        name="right end of the tilt shaft is supported by the right yoke plate",
    )

    lower = tilt.motion_limits.lower if tilt.motion_limits and tilt.motion_limits.lower is not None else -0.3
    upper = tilt.motion_limits.upper if tilt.motion_limits and tilt.motion_limits.upper is not None else 0.3

    with ctx.pose({tilt: lower}):
        ctx.expect_contact(
            head,
            support,
            elem_a="tilt_shaft",
            elem_b="yoke_left",
            name="left trunnion support remains engaged at the upward tilt limit",
        )
        lens_up = _aabb_center(ctx.part_element_world_aabb(head, elem="lens"))

    with ctx.pose({tilt: upper}):
        ctx.expect_contact(
            head,
            support,
            elem_a="tilt_shaft",
            elem_b="yoke_left",
            name="left trunnion support remains engaged at the downward tilt limit",
        )
        lens_down = _aabb_center(ctx.part_element_world_aabb(head, elem="lens"))

    ctx.check(
        "tilt joint drives the lens downward through its range",
        lens_up is not None
        and lens_down is not None
        and lens_down[2] < lens_up[2] - 0.12,
        details=f"lens_up={lens_up}, lens_down={lens_down}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
