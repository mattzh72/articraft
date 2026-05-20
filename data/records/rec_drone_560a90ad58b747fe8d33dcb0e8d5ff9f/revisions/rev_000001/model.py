from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _circle_profile(radius: float, segments: int = 28) -> list[tuple[float, float]]:
    return [
        (
            radius * cos((2.0 * pi * index) / segments),
            radius * sin((2.0 * pi * index) / segments),
        )
        for index in range(segments)
    ]


def _blade_profile(hub_radius: float, span_radius: float) -> list[tuple[float, float]]:
    root_x = hub_radius * 0.72
    return [
        (root_x, -0.020),
        (0.085, -0.0175),
        (0.160, -0.0130),
        (span_radius - 0.028, -0.0095),
        (span_radius, -0.0060),
        (span_radius, 0.0060),
        (span_radius - 0.030, 0.0095),
        (0.162, 0.0120),
        (0.086, 0.0155),
        (root_x, 0.0190),
    ]


def _build_rotor_meshes():
    hub_outer_radius = 0.0165
    hub_inner_radius = 0.0090
    hub_thickness = 0.0050
    blade_span_radius = 0.230
    blade_thickness = 0.0030

    hub_geom = ExtrudeWithHolesGeometry(
        _circle_profile(hub_outer_radius, segments=30),
        [_circle_profile(hub_inner_radius, segments=24)],
        height=hub_thickness,
        center=True,
    )
    blade_geom = ExtrudeGeometry(
        _blade_profile(hub_outer_radius, blade_span_radius),
        blade_thickness,
        center=True,
    )
    return (
        mesh_from_geometry(hub_geom, "coaxial_hub_ring"),
        mesh_from_geometry(blade_geom, "coaxial_rotor_blade"),
        hub_thickness,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coaxial_birotor_helicopter_drone")

    body_paint = model.material("body_paint", rgba=(0.86, 0.88, 0.92, 1.0))
    canopy_tint = model.material("canopy_tint", rgba=(0.12, 0.14, 0.18, 0.92))
    rotor_black = model.material("rotor_black", rgba=(0.10, 0.10, 0.11, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.26, 0.28, 0.30, 1.0))
    skid_graphite = model.material("skid_graphite", rgba=(0.18, 0.18, 0.19, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.87, 0.42, 0.10, 1.0))

    hub_mesh, blade_mesh, hub_thickness = _build_rotor_meshes()

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.34, 0.14, 0.18)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    body.visual(
        Cylinder(radius=0.055, length=0.270),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=body_paint,
        name="main_fuselage",
    )
    body.visual(
        Sphere(radius=0.055),
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
        material=body_paint,
        name="nose_cap",
    )
    body.visual(
        Sphere(radius=0.040),
        origin=Origin(xyz=(-0.135, 0.0, 0.0)),
        material=body_paint,
        name="tail_cap",
    )
    body.visual(
        Cylinder(radius=0.041, length=0.095),
        origin=Origin(xyz=(0.070, 0.0, 0.010), rpy=(0.0, pi / 2.0, 0.0)),
        material=canopy_tint,
        name="cockpit_canopy",
    )
    body.visual(
        Box((0.100, 0.018, 0.020)),
        origin=Origin(xyz=(0.015, 0.0, 0.055)),
        material=accent_orange,
        name="top_stripe",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=metal_dark,
        name="mast_base",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.076),
        origin=Origin(xyz=(0.0, 0.0, 0.111)),
        material=metal_dark,
        name="central_shaft",
    )
    body.visual(
        Cylinder(radius=0.0105, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=metal_dark,
        name="lower_bearing_collar",
    )
    body.visual(
        Cylinder(radius=0.0105, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        material=metal_dark,
        name="upper_bearing_collar",
    )
    body.visual(
        Box((0.040, 0.130, 0.012)),
        origin=Origin(xyz=(0.090, 0.0, -0.061)),
        material=metal_dark,
        name="front_frame_pad",
    )
    body.visual(
        Box((0.040, 0.130, 0.012)),
        origin=Origin(xyz=(-0.090, 0.0, -0.061)),
        material=metal_dark,
        name="rear_frame_pad",
    )

    landing_frame = model.part("landing_frame")
    landing_frame.inertial = Inertial.from_geometry(
        Box((0.36, 0.34, 0.12)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
    )
    frame_radius = 0.0060
    landing_frame.visual(
        mesh_from_geometry(
            wire_from_points(
                [
                    (-0.145, 0.155, -0.170),
                    (-0.165, 0.155, -0.155),
                    (0.165, 0.155, -0.155),
                    (0.145, 0.155, -0.170),
                ],
                radius=frame_radius,
                radial_segments=18,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.022,
                corner_segments=8,
            ),
            "left_skid_runner",
        ),
        material=skid_graphite,
        name="left_skid",
    )
    landing_frame.visual(
        mesh_from_geometry(
            wire_from_points(
                [
                    (-0.145, -0.155, -0.170),
                    (-0.165, -0.155, -0.155),
                    (0.165, -0.155, -0.155),
                    (0.145, -0.155, -0.170),
                ],
                radius=frame_radius,
                radial_segments=18,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.022,
                corner_segments=8,
            ),
            "right_skid_runner",
        ),
        material=skid_graphite,
        name="right_skid",
    )
    landing_frame.visual(
        mesh_from_geometry(
            wire_from_points(
                [(0.090, -0.060, -0.073), (0.090, 0.060, -0.073)],
                radius=frame_radius,
                radial_segments=16,
                cap_ends=True,
            ),
            "front_crossbar",
        ),
        material=skid_graphite,
        name="front_crossbar",
    )
    landing_frame.visual(
        mesh_from_geometry(
            wire_from_points(
                [(-0.090, -0.060, -0.073), (-0.090, 0.060, -0.073)],
                radius=frame_radius,
                radial_segments=16,
                cap_ends=True,
            ),
            "rear_crossbar",
        ),
        material=skid_graphite,
        name="rear_crossbar",
    )
    for visual_name, points in [
        ("front_left_leg", [(0.090, 0.060, -0.073), (0.140, 0.155, -0.155)]),
        ("front_right_leg", [(0.090, -0.060, -0.073), (0.140, -0.155, -0.155)]),
        ("rear_left_leg", [(-0.090, 0.060, -0.073), (-0.140, 0.155, -0.155)]),
        ("rear_right_leg", [(-0.090, -0.060, -0.073), (-0.140, -0.155, -0.155)]),
    ]:
        landing_frame.visual(
            mesh_from_geometry(
                wire_from_points(
                    points,
                    radius=frame_radius,
                    radial_segments=16,
                    cap_ends=True,
                ),
                visual_name,
            ),
            material=skid_graphite,
            name=visual_name,
        )

    lower_rotor = model.part("lower_rotor")
    lower_rotor.inertial = Inertial.from_geometry(
        Box((0.460, 0.055, 0.008)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )
    lower_rotor.visual(
        hub_mesh,
        origin=Origin(xyz=(0.0, 0.0, hub_thickness * 0.5)),
        material=metal_dark,
        name="hub_ring",
    )
    lower_rotor.visual(
        blade_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=rotor_black,
        name="blade_a",
    )
    lower_rotor.visual(
        blade_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0025), rpy=(0.0, 0.0, pi)),
        material=rotor_black,
        name="blade_b",
    )

    upper_rotor = model.part("upper_rotor")
    upper_rotor.inertial = Inertial.from_geometry(
        Box((0.460, 0.055, 0.008)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )
    upper_rotor.visual(
        hub_mesh,
        origin=Origin(xyz=(0.0, 0.0, hub_thickness * 0.5)),
        material=metal_dark,
        name="hub_ring",
    )
    upper_rotor.visual(
        blade_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0025), rpy=(0.0, 0.0, pi / 2.0)),
        material=rotor_black,
        name="blade_a",
    )
    upper_rotor.visual(
        blade_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0025), rpy=(0.0, 0.0, 3.0 * pi / 2.0)),
        material=rotor_black,
        name="blade_b",
    )

    model.articulation(
        "body_to_landing_frame",
        ArticulationType.FIXED,
        parent=body,
        child=landing_frame,
        origin=Origin(),
    )
    model.articulation(
        "lower_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lower_rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.103)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=40.0),
    )
    model.articulation(
        "upper_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=upper_rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=40.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    landing_frame = object_model.get_part("landing_frame")
    lower_rotor = object_model.get_part("lower_rotor")
    upper_rotor = object_model.get_part("upper_rotor")
    lower_rotor_spin = object_model.get_articulation("lower_rotor_spin")
    upper_rotor_spin = object_model.get_articulation("upper_rotor_spin")

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

    ctx.expect_contact(landing_frame, body, name="landing_frame_contacts_body")
    ctx.expect_contact(lower_rotor, body, name="lower_rotor_seated_on_axle")
    ctx.expect_contact(upper_rotor, body, name="upper_rotor_seated_on_axle")
    ctx.expect_gap(
        upper_rotor,
        lower_rotor,
        axis="z",
        min_gap=0.024,
        name="rotor_stack_vertical_clearance",
    )
    ctx.expect_origin_distance(
        lower_rotor,
        upper_rotor,
        axes="xy",
        max_dist=0.001,
        name="rotor_axes_are_aligned",
    )
    ctx.expect_overlap(
        lower_rotor,
        upper_rotor,
        axes="xy",
        elem_a="hub_ring",
        elem_b="hub_ring",
        min_overlap=0.018,
        name="rotor_hubs_share_the_same_axle",
    )
    ctx.expect_gap(
        body,
        landing_frame,
        axis="z",
        positive_elem="main_fuselage",
        negative_elem="left_skid",
        min_gap=0.080,
        name="skids_hang_below_fuselage",
    )
    ctx.check(
        "lower_rotor_axis_is_positive_z",
        tuple(lower_rotor_spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={lower_rotor_spin.axis}",
    )
    ctx.check(
        "upper_rotor_axis_is_negative_z",
        tuple(upper_rotor_spin.axis) == (0.0, 0.0, -1.0),
        details=f"axis={upper_rotor_spin.axis}",
    )

    lower_blade_rest = ctx.part_element_world_aabb(lower_rotor, elem="blade_a")
    upper_blade_rest = ctx.part_element_world_aabb(upper_rotor, elem="blade_a")
    if lower_blade_rest is None or upper_blade_rest is None:
        ctx.fail("rotor_blade_aabbs_exist", "Could not resolve rotor blade AABBs in rest pose.")
    else:
        lower_rest_dx = lower_blade_rest[1][0] - lower_blade_rest[0][0]
        lower_rest_dy = lower_blade_rest[1][1] - lower_blade_rest[0][1]
        upper_rest_dx = upper_blade_rest[1][0] - upper_blade_rest[0][0]
        upper_rest_dy = upper_blade_rest[1][1] - upper_blade_rest[0][1]
        ctx.check(
            "rest_pose_rotors_are_staggered",
            lower_rest_dx > lower_rest_dy and upper_rest_dy > upper_rest_dx,
            details=(
                f"lower(dx={lower_rest_dx:.4f}, dy={lower_rest_dy:.4f}), "
                f"upper(dx={upper_rest_dx:.4f}, dy={upper_rest_dy:.4f})"
            ),
        )

    with ctx.pose({lower_rotor_spin: pi / 2.0, upper_rotor_spin: pi / 2.0}):
        lower_blade_turn = ctx.part_element_world_aabb(lower_rotor, elem="blade_a")
        upper_blade_turn = ctx.part_element_world_aabb(upper_rotor, elem="blade_a")
        if lower_blade_turn is None or upper_blade_turn is None:
            ctx.fail("rotor_blade_aabbs_exist_in_pose", "Could not resolve rotor blade AABBs in spun pose.")
        else:
            lower_turn_dx = lower_blade_turn[1][0] - lower_blade_turn[0][0]
            lower_turn_dy = lower_blade_turn[1][1] - lower_blade_turn[0][1]
            upper_turn_dx = upper_blade_turn[1][0] - upper_blade_turn[0][0]
            upper_turn_dy = upper_blade_turn[1][1] - upper_blade_turn[0][1]
            ctx.check(
                "counter_rotating_rotors_turn_opposite_directions",
                lower_turn_dy > lower_turn_dx and upper_turn_dx > upper_turn_dy,
                details=(
                    f"lower(dx={lower_turn_dx:.4f}, dy={lower_turn_dy:.4f}), "
                    f"upper(dx={upper_turn_dx:.4f}, dy={upper_turn_dy:.4f})"
                ),
            )
        ctx.expect_gap(
            upper_rotor,
            lower_rotor,
            axis="z",
            min_gap=0.024,
            name="rotor_stack_clearance_in_spun_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
