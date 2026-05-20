from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _wheel_tire_mesh(name: str, radius: float, width: float):
    half_width = width * 0.5
    profile = [
        (radius * 0.62, -half_width * 0.96),
        (radius * 0.79, -half_width),
        (radius * 0.91, -half_width * 0.86),
        (radius * 0.98, -half_width * 0.36),
        (radius, 0.0),
        (radius * 0.98, half_width * 0.36),
        (radius * 0.91, half_width * 0.86),
        (radius * 0.79, half_width),
        (radius * 0.62, half_width * 0.96),
        (radius * 0.55, half_width * 0.42),
        (radius * 0.53, 0.0),
        (radius * 0.55, -half_width * 0.42),
        (radius * 0.62, -half_width * 0.96),
    ]
    return _mesh(name, LatheGeometry(profile, segments=56).rotate_y(math.pi / 2.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kamado_grill")

    ceramic_green = model.material("ceramic_green", rgba=(0.12, 0.22, 0.17, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.52, 0.54, 0.57, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))
    handle_wood = model.material("handle_wood", rgba=(0.43, 0.25, 0.12, 1.0))

    base = model.part("base")

    lower_bowl = LatheGeometry.from_shell_profiles(
        [
            (0.0, 0.215),
            (0.10, 0.205),
            (0.19, 0.220),
            (0.26, 0.290),
            (0.286, 0.385),
            (0.290, 0.500),
            (0.280, 0.580),
            (0.270, 0.605),
        ],
        [
            (0.0, 0.255),
            (0.11, 0.248),
            (0.18, 0.280),
            (0.225, 0.350),
            (0.238, 0.470),
            (0.235, 0.565),
            (0.225, 0.595),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
    )
    base.visual(_mesh("lower_bowl_shell", lower_bowl), material=ceramic_green, name="lower_bowl")

    base.visual(
        _mesh("lower_band", TorusGeometry(radius=0.281, tube=0.013, radial_segments=16, tubular_segments=72)),
        origin=Origin(xyz=(0.0, 0.0, 0.592)),
        material=steel_dark,
        name="lower_band",
    )
    base.visual(
        _mesh("cradle_ring", TorusGeometry(radius=0.262, tube=0.012, radial_segments=16, tubular_segments=72)),
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        material=steel_dark,
        name="cradle_ring",
    )

    for name, points in (
        (
            "front_left_leg",
            [
                (0.210, 0.190, 0.015),
                (0.214, 0.190, 0.150),
                (0.221, 0.165, 0.275),
                (0.230, 0.125, 0.365),
            ],
        ),
        (
            "front_right_leg",
            _mirror_x(
                [
                    (0.210, 0.190, 0.015),
                    (0.214, 0.190, 0.150),
                    (0.221, 0.165, 0.275),
                    (0.230, 0.125, 0.365),
                ]
            ),
        ),
        (
            "rear_left_leg",
            [
                (0.220, -0.220, 0.015),
                (0.225, -0.220, 0.140),
                (0.231, -0.180, 0.275),
                (0.238, -0.110, 0.365),
            ],
        ),
        (
            "rear_right_leg",
            _mirror_x(
                [
                    (0.220, -0.220, 0.015),
                    (0.225, -0.220, 0.140),
                    (0.231, -0.180, 0.275),
                    (0.238, -0.110, 0.365),
                ]
            ),
        ),
        (
            "left_lower_rail",
            [
                (0.205, 0.190, 0.085),
                (0.214, 0.020, 0.080),
                (0.220, -0.220, 0.080),
            ],
        ),
        (
            "right_lower_rail",
            _mirror_x(
                [
                    (0.205, 0.190, 0.085),
                    (0.214, 0.020, 0.080),
                    (0.220, -0.220, 0.080),
                ]
            ),
        ),
        (
            "front_lower_rail",
            [
                (-0.205, 0.190, 0.085),
                (0.0, 0.195, 0.085),
                (0.205, 0.190, 0.085),
            ],
        ),
        (
            "left_hinge_support",
            [
                (-0.220, -0.220, 0.140),
                (-0.245, -0.245, 0.330),
                (-0.220, -0.338, 0.545),
                (-0.170, -0.345, 0.678),
            ],
        ),
        (
            "right_hinge_support",
            [
                (0.220, -0.220, 0.140),
                (0.245, -0.245, 0.330),
                (0.220, -0.338, 0.545),
                (0.170, -0.345, 0.678),
            ],
        ),
    ):
        base.visual(
            _mesh(
                f"{name}_mesh",
                tube_from_spline_points(
                    points,
                    radius=0.012 if "hinge" not in name else 0.011,
                    samples_per_segment=12,
                    radial_segments=18,
                ),
            ),
            material=steel_dark,
            name=name,
        )

    base.visual(
        Cylinder(radius=0.016, length=0.562),
        origin=Origin(xyz=(0.0, -0.220, 0.140), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="rear_axle",
    )
    base.visual(
        Box((0.128, 0.018, 0.018)),
        origin=Origin(xyz=(-0.138, -0.326, 0.678)),
        material=steel_dark,
        name="left_hinge_link",
    )
    base.visual(
        Box((0.128, 0.018, 0.018)),
        origin=Origin(xyz=(0.138, -0.326, 0.678)),
        material=steel_dark,
        name="right_hinge_link",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.066),
        origin=Origin(xyz=(0.294, -0.220, 0.140), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mid,
        name="left_axle_stub",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.066),
        origin=Origin(xyz=(-0.294, -0.220, 0.140), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mid,
        name="right_axle_stub",
    )

    base.visual(
        Cylinder(radius=0.022, length=0.050),
        origin=Origin(xyz=(-0.106, -0.297, 0.678), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="left_hinge_bracket",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.050),
        origin=Origin(xyz=(0.106, -0.297, 0.678), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="right_hinge_bracket",
    )

    base.visual(
        Box((0.110, 0.028, 0.080)),
        origin=Origin(xyz=(0.0, 0.276, 0.335)),
        material=steel_dark,
        name="draft_door",
    )
    base.visual(
        Box((0.060, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, 0.292, 0.328)),
        material=steel_mid,
        name="draft_slider",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.72, 0.72, 0.98)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
    )

    lid = model.part("lid")

    lid_shell = LatheGeometry.from_shell_profiles(
        [
            (0.268, 0.000),
            (0.278, 0.035),
            (0.282, 0.090),
            (0.252, 0.195),
            (0.182, 0.275),
            (0.090, 0.315),
            (0.0, 0.325),
        ],
        [
            (0.240, 0.000),
            (0.248, 0.030),
            (0.244, 0.085),
            (0.220, 0.178),
            (0.155, 0.248),
            (0.072, 0.287),
            (0.0, 0.296),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
    )
    lid.visual(
        _mesh("lid_shell", lid_shell),
        origin=Origin(xyz=(0.0, 0.290, -0.070)),
        material=ceramic_green,
        name="lid_shell",
    )
    lid.visual(
        _mesh("upper_band", TorusGeometry(radius=0.284, tube=0.015, radial_segments=16, tubular_segments=72)),
        origin=Origin(xyz=(0.0, 0.290, -0.055)),
        material=steel_dark,
        name="upper_band",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.162),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="hinge_knuckle",
    )

    for name, points in (
        (
            "left_handle_support",
            [
                (-0.100, 0.500, -0.022),
                (-0.100, 0.535, 0.010),
                (-0.100, 0.560, 0.045),
            ],
        ),
        (
            "right_handle_support",
            [
                (0.100, 0.500, -0.022),
                (0.100, 0.535, 0.010),
                (0.100, 0.560, 0.045),
            ],
        ),
    ):
        lid.visual(
            _mesh(
                f"{name}_mesh",
                tube_from_spline_points(points, radius=0.012, samples_per_segment=10, radial_segments=16),
            ),
            material=steel_dark,
            name=name,
        )

    lid.visual(
        Box((0.024, 0.024, 0.036)),
        origin=Origin(xyz=(-0.070, 0.0, 0.0)),
        material=steel_dark,
        name="left_lid_strap",
    )
    lid.visual(
        Box((0.024, 0.024, 0.036)),
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        material=steel_dark,
        name="right_lid_strap",
    )

    lid.visual(
        Cylinder(radius=0.016, length=0.220),
        origin=Origin(xyz=(0.0, 0.570, 0.048), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_wood,
        name="front_handle",
    )
    lid.visual(
        Cylinder(radius=0.054, length=0.030),
        origin=Origin(xyz=(0.0, 0.290, 0.238)),
        material=steel_dark,
        name="vent_collar",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.66, 0.72, 0.46)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.290, 0.050)),
    )

    vent_cap = model.part("vent_cap")
    vent_cap.visual(
        Cylinder(radius=0.060, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=steel_dark,
        name="vent_base",
    )
    vent_cap.visual(
        Cylinder(radius=0.042, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=steel_mid,
        name="vent_cover",
    )
    vent_cap.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=steel_mid,
        name="vent_knob",
    )
    vent_cap.visual(
        Box((0.020, 0.010, 0.004)),
        origin=Origin(xyz=(0.034, 0.0, 0.022)),
        material=steel_mid,
        name="vent_tab",
    )
    vent_cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.040),
        mass=0.4,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(_wheel_tire_mesh("left_wheel_tire", radius=0.115, width=0.050), material=rubber_black, name="tire")
    left_wheel.visual(
        Cylinder(radius=0.072, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mid,
        name="rim",
    )
    left_wheel.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="hub",
    )
    left_wheel.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material=steel_mid,
        name="valve_stem",
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.050),
        mass=2.5,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(_wheel_tire_mesh("right_wheel_tire", radius=0.115, width=0.050), material=rubber_black, name="tire")
    right_wheel.visual(
        Cylinder(radius=0.072, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mid,
        name="rim",
    )
    right_wheel.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="hub",
    )
    right_wheel.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material=steel_mid,
        name="valve_stem",
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.050),
        mass=2.5,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, -0.290, 0.680)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=0.0,
            upper=1.50,
        ),
    )
    model.articulation(
        "vent_spin",
        ArticulationType.CONTINUOUS,
        parent=lid,
        child=vent_cap,
        origin=Origin(xyz=(0.0, 0.290, 0.253)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=left_wheel,
        origin=Origin(xyz=(0.335, -0.220, 0.140)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=right_wheel,
        origin=Origin(xyz=(-0.335, -0.220, 0.140)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    vent_cap = object_model.get_part("vent_cap")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    lid_hinge = object_model.get_articulation("lid_hinge")
    vent_spin = object_model.get_articulation("vent_spin")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")

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

    ctx.check("lid_hinge_axis", lid_hinge.axis == (1.0, 0.0, 0.0), f"axis={lid_hinge.axis}")
    ctx.check("vent_spin_axis", vent_spin.axis == (0.0, 0.0, 1.0), f"axis={vent_spin.axis}")
    ctx.check("left_wheel_axis", left_wheel_spin.axis == (1.0, 0.0, 0.0), f"axis={left_wheel_spin.axis}")
    ctx.check("right_wheel_axis", right_wheel_spin.axis == (1.0, 0.0, 0.0), f"axis={right_wheel_spin.axis}")

    ctx.expect_contact(left_wheel, base)
    ctx.expect_contact(right_wheel, base)
    ctx.expect_contact(vent_cap, lid, elem_a="vent_base", elem_b="vent_collar")
    ctx.expect_contact(lid, base, elem_a="hinge_knuckle", elem_b="left_hinge_bracket")
    ctx.expect_contact(lid, base, elem_a="hinge_knuckle", elem_b="right_hinge_bracket")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.46, elem_a="lid_shell", elem_b="lower_bowl")
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            min_gap=0.003,
            max_gap=0.020,
            positive_elem="lid_shell",
            negative_elem="lower_bowl",
            name="closed_lid_seam_gap",
        )

    with ctx.pose({lid_hinge: 1.20}):
        ctx.expect_contact(lid, base, elem_a="hinge_knuckle", elem_b="left_hinge_bracket", name="open_left_hinge_contact")
        ctx.expect_contact(lid, base, elem_a="hinge_knuckle", elem_b="right_hinge_bracket", name="open_right_hinge_contact")

    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    handle_rest = ctx.part_element_world_aabb(lid, elem="front_handle")
    assert handle_rest is not None
    with ctx.pose({lid_hinge: 1.20}):
        handle_open = ctx.part_element_world_aabb(lid, elem="front_handle")
        assert handle_open is not None
        handle_rest_center = aabb_center(handle_rest)
        handle_open_center = aabb_center(handle_open)
        ctx.check(
            "lid_front_handle_lifts",
            handle_open_center[2] > handle_rest_center[2] + 0.17,
            f"rest_z={handle_rest_center[2]:.4f}, open_z={handle_open_center[2]:.4f}",
        )

    vent_tab_rest = ctx.part_element_world_aabb(vent_cap, elem="vent_tab")
    assert vent_tab_rest is not None
    with ctx.pose({vent_spin: math.pi / 2.0}):
        vent_tab_rotated = ctx.part_element_world_aabb(vent_cap, elem="vent_tab")
        assert vent_tab_rotated is not None
        rest_center = aabb_center(vent_tab_rest)
        rotated_center = aabb_center(vent_tab_rotated)
        ctx.check(
            "vent_tab_orbits_crown",
            abs(rotated_center[1]) > abs(rest_center[1]) + 0.020 and abs(rotated_center[0]) < abs(rest_center[0]) - 0.020,
            f"rest={rest_center}, rotated={rotated_center}",
        )

    valve_rest = ctx.part_element_world_aabb(left_wheel, elem="valve_stem")
    assert valve_rest is not None
    with ctx.pose({left_wheel_spin: math.pi / 2.0}):
        valve_rotated = ctx.part_element_world_aabb(left_wheel, elem="valve_stem")
        assert valve_rotated is not None
        valve_rest_center = aabb_center(valve_rest)
        valve_rotated_center = aabb_center(valve_rotated)
        ctx.check(
            "left_wheel_rotates_about_axle",
            valve_rotated_center[2] < valve_rest_center[2] - 0.050 and valve_rotated_center[1] < valve_rest_center[1] - 0.050,
            f"rest={valve_rest_center}, rotated={valve_rotated_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
