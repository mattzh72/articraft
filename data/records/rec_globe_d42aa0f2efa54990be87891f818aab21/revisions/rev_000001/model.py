from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="celestial_globe_turntable")

    base_wood = model.material("base_wood", rgba=(0.26, 0.17, 0.10, 1.0))
    brass = model.material("brass", rgba=(0.77, 0.66, 0.28, 1.0))
    globe_blue = model.material("globe_blue", rgba=(0.11, 0.17, 0.29, 1.0))
    dark_brass = model.material("dark_brass", rgba=(0.53, 0.43, 0.18, 1.0))

    base_height = 0.045
    foot_height = 0.008
    base_radius = 0.160
    foot_radius = 0.180

    plate_radius = 0.158
    plate_height = 0.012
    collar_radius = 0.095
    collar_height = 0.008

    globe_radius = 0.140
    globe_center_z_world = 0.245
    globe_center_z_local = globe_center_z_world - base_height

    globe_tilt = math.radians(23.5)
    polar_axis = (math.sin(globe_tilt), 0.0, math.cos(globe_tilt))

    axle_radius = 0.006
    axle_length = 0.014
    pivot_radius = 0.005
    pivot_length = 0.016

    side_post_radius = 0.010
    side_post_y = 0.160
    bridge_radius = 0.012
    bridge_length = 2.0 * side_post_y

    support_contact_s = globe_radius + axle_length
    pivot_center_s = support_contact_s + 0.5 * pivot_length
    bridge_center_s = support_contact_s + pivot_length

    def axis_point(s: float) -> tuple[float, float, float]:
        return (
            polar_axis[0] * s,
            0.0,
            globe_center_z_local + polar_axis[2] * s,
        )

    base = model.part("base")
    base.visual(
        Cylinder(radius=foot_radius, length=foot_height),
        origin=Origin(xyz=(0.0, 0.0, foot_height * 0.5)),
        material=base_wood,
        name="foot_ring",
    )
    base.visual(
        Cylinder(radius=base_radius, length=base_height - foot_height),
        origin=Origin(xyz=(0.0, 0.0, foot_height + (base_height - foot_height) * 0.5)),
        material=base_wood,
        name="base_drum",
    )

    upper = model.part("upper_assembly")
    upper.visual(
        Cylinder(radius=plate_radius, length=plate_height),
        origin=Origin(xyz=(0.0, 0.0, plate_height * 0.5)),
        material=brass,
        name="turntable_plate",
    )
    upper.visual(
        Cylinder(radius=collar_radius, length=collar_height),
        origin=Origin(xyz=(0.0, 0.0, plate_height + collar_height * 0.5)),
        material=dark_brass,
        name="turntable_collar",
    )

    bridge_center = axis_point(bridge_center_s)
    bridge_left = (bridge_center[0], -side_post_y, bridge_center[2])
    bridge_right = (bridge_center[0], side_post_y, bridge_center[2])

    upper.visual(
        Cylinder(radius=bridge_radius, length=bridge_length),
        origin=Origin(xyz=bridge_center, rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=brass,
        name="top_bridge",
    )

    lower_support_center = axis_point(-pivot_center_s)
    upper_support_center = axis_point(pivot_center_s)

    upper.visual(
        Cylinder(radius=pivot_radius, length=pivot_length),
        origin=Origin(xyz=upper_support_center, rpy=(0.0, globe_tilt, 0.0)),
        material=dark_brass,
        name="top_pivot",
    )
    upper.visual(
        Cylinder(radius=pivot_radius, length=pivot_length),
        origin=Origin(xyz=lower_support_center, rpy=(0.0, globe_tilt, 0.0)),
        material=dark_brass,
        name="bottom_pivot",
    )

    pedestal_height = lower_support_center[2]
    upper.visual(
        Cylinder(radius=0.018, length=pedestal_height),
        origin=Origin(xyz=(lower_support_center[0], 0.0, pedestal_height * 0.5)),
        material=brass,
        name="bottom_pedestal",
    )
    upper.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(lower_support_center[0], 0.0, plate_height * 0.5)),
        material=dark_brass,
        name="pedestal_collar",
    )

    left_post = tube_from_spline_points(
        [
            (0.0, -side_post_y, plate_height * 0.5),
            (0.015, -side_post_y, globe_center_z_local * 0.45),
            bridge_left,
        ],
        radius=side_post_radius,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    right_post = tube_from_spline_points(
        [
            (0.0, side_post_y, plate_height * 0.5),
            (0.015, side_post_y, globe_center_z_local * 0.45),
            bridge_right,
        ],
        radius=side_post_radius,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    upper.visual(
        mesh_from_geometry(left_post, "left_post"),
        material=brass,
        name="left_post",
    )
    upper.visual(
        mesh_from_geometry(right_post, "right_post"),
        material=brass,
        name="right_post",
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=globe_radius),
        origin=Origin(),
        material=globe_blue,
        name="globe_shell",
    )
    globe.visual(
        Cylinder(radius=axle_radius, length=axle_length),
        origin=Origin(
            xyz=(
                polar_axis[0] * (globe_radius + axle_length * 0.5),
                0.0,
                polar_axis[2] * (globe_radius + axle_length * 0.5),
            ),
            rpy=(0.0, globe_tilt, 0.0),
        ),
        material=dark_brass,
        name="top_axle",
    )
    globe.visual(
        Cylinder(radius=axle_radius, length=axle_length),
        origin=Origin(
            xyz=(
                -polar_axis[0] * (globe_radius + axle_length * 0.5),
                0.0,
                -polar_axis[2] * (globe_radius + axle_length * 0.5),
            ),
            rpy=(0.0, globe_tilt, 0.0),
        ),
        material=dark_brass,
        name="bottom_axle",
    )

    model.articulation(
        "base_to_upper",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, base_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0),
    )
    model.articulation(
        "upper_to_globe",
        ArticulationType.CONTINUOUS,
        parent=upper,
        child=globe,
        origin=Origin(xyz=(0.0, 0.0, globe_center_z_local)),
        axis=polar_axis,
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper = object_model.get_part("upper_assembly")
    globe = object_model.get_part("globe")
    base_to_upper = object_model.get_articulation("base_to_upper")
    upper_to_globe = object_model.get_articulation("upper_to_globe")

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

    expected_globe_axis = (
        math.sin(math.radians(23.5)),
        0.0,
        math.cos(math.radians(23.5)),
    )
    ctx.check(
        "turntable_axis_is_vertical",
        tuple(base_to_upper.axis) == (0.0, 0.0, 1.0),
        f"expected vertical axis, got {base_to_upper.axis}",
    )
    ctx.check(
        "globe_axis_matches_polar_tilt",
        max(abs(a - b) for a, b in zip(upper_to_globe.axis, expected_globe_axis)) < 1e-6,
        f"expected {expected_globe_axis}, got {upper_to_globe.axis}",
    )

    ctx.expect_contact(
        upper,
        base,
        elem_a="turntable_plate",
        elem_b="base_drum",
        name="turntable_plate_seats_on_base",
    )
    ctx.expect_overlap(
        upper,
        base,
        axes="xy",
        elem_a="turntable_plate",
        elem_b="base_drum",
        min_overlap=0.28,
        name="turntable_centered_over_base",
    )
    ctx.expect_contact(
        globe,
        upper,
        elem_a="top_axle",
        elem_b="top_pivot",
        name="top_polar_pivot_captures_globe",
    )
    ctx.expect_contact(
        globe,
        upper,
        elem_a="bottom_axle",
        elem_b="bottom_pivot",
        name="bottom_polar_pivot_captures_globe",
    )
    ctx.expect_gap(
        globe,
        upper,
        axis="z",
        positive_elem="globe_shell",
        negative_elem="turntable_plate",
        min_gap=0.045,
        max_gap=0.060,
        name="globe_clears_turntable_plate",
    )

    with ctx.pose({base_to_upper: 1.15, upper_to_globe: 2.10}):
        ctx.expect_contact(
            globe,
            upper,
            elem_a="top_axle",
            elem_b="top_pivot",
            name="top_pivot_contact_persists_when_rotated",
        )
        ctx.expect_contact(
            globe,
            upper,
            elem_a="bottom_axle",
            elem_b="bottom_pivot",
            name="bottom_pivot_contact_persists_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
