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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def _rect_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _aabb_center(aabb):
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_bar_blender")

    housing_body = model.material("housing_body", rgba=(0.16, 0.17, 0.18, 1.0))
    housing_trim = model.material("housing_trim", rgba=(0.08, 0.09, 0.10, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    start_green = model.material("start_green", rgba=(0.14, 0.60, 0.28, 1.0))
    stop_red = model.material("stop_red", rgba=(0.74, 0.14, 0.12, 1.0))
    amber = model.material("amber", rgba=(0.83, 0.58, 0.14, 1.0))
    polycarbonate = model.material("polycarbonate", rgba=(0.80, 0.90, 0.97, 0.28))
    lid_clear = model.material("lid_clear", rgba=(0.84, 0.93, 0.99, 0.24))
    jar_base_dark = model.material("jar_base_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.76, 0.78, 0.80, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _rect_section(0.240, 0.230, 0.026, 0.012),
                    _rect_section(0.236, 0.226, 0.026, 0.110),
                    _rect_section(0.224, 0.208, 0.024, 0.245),
                    _rect_section(0.214, 0.194, 0.022, 0.340),
                ]
            ),
            "housing_shell",
        ),
        material=housing_body,
        name="housing_shell",
    )
    housing.visual(
        Box((0.132, 0.132, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=housing_trim,
        name="top_deck",
    )
    housing.visual(
        Box((0.104, 0.104, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=panel_dark,
        name="jar_pad",
    )
    housing.visual(
        Box((0.110, 0.004, 0.084)),
        origin=Origin(xyz=(0.0, -0.100, 0.158), rpy=(-0.58, 0.0, 0.0)),
        material=panel_dark,
        name="control_panel",
    )
    housing.visual(
        Box((0.020, 0.014, 0.011)),
        origin=Origin(xyz=(-0.030, -0.103, 0.168), rpy=(-0.58, 0.0, 0.0)),
        material=start_green,
        name="start_button",
    )
    housing.visual(
        Box((0.020, 0.003, 0.011)),
        origin=Origin(xyz=(0.000, -0.103, 0.151), rpy=(-0.58, 0.0, 0.0)),
        material=amber,
        name="speed_button",
    )
    housing.visual(
        Box((0.020, 0.014, 0.011)),
        origin=Origin(xyz=(0.030, -0.103, 0.134), rpy=(-0.58, 0.0, 0.0)),
        material=stop_red,
        name="stop_button",
    )
    housing.visual(
        Box((0.072, 0.014, 0.090)),
        origin=Origin(xyz=(0.0, -0.104, 0.263)),
        material=housing_trim,
        name="latch_mount",
    )
    housing.visual(
        Box((0.010, 0.060, 0.040)),
        origin=Origin(xyz=(-0.112, 0.0, 0.180)),
        material=panel_dark,
        name="left_vent",
    )
    housing.visual(
        Box((0.010, 0.060, 0.040)),
        origin=Origin(xyz=(0.112, 0.0, 0.180)),
        material=panel_dark,
        name="right_vent",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            housing.visual(
                Cylinder(radius=0.018, length=0.012),
                origin=Origin(xyz=(0.082 * x_sign, 0.074 * y_sign, 0.006)),
                material=rubber,
                name=f"foot_{'r' if x_sign > 0 else 'l'}_{'rear' if y_sign > 0 else 'front'}",
            )
    housing.inertial = Inertial.from_geometry(
        Box((0.240, 0.230, 0.352)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
    )

    jar = model.part("jar")
    jar.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.122, 0.122, 0.012, corner_segments=10),
                [superellipse_profile(0.022, 0.022, exponent=2.0, segments=30)],
                0.018,
                center=False,
            ),
            "jar_base_block",
        ),
        material=jar_base_dark,
        name="jar_base_block",
    )
    jar.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.130, 0.130, 0.014, corner_segments=10),
                [rounded_rect_profile(0.120, 0.120, 0.011, corner_segments=10)],
                0.210,
                cap=False,
                center=False,
            ),
            "jar_wall_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=polycarbonate,
        name="jar_wall_shell",
    )
    jar.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.136, 0.136, 0.015, corner_segments=10),
                [rounded_rect_profile(0.120, 0.120, 0.011, corner_segments=10)],
                0.010,
                center=False,
            ),
            "jar_upper_rim",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.228)),
        material=polycarbonate,
        name="jar_upper_rim",
    )
    jar.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                superellipse_profile(0.036, 0.036, exponent=2.0, segments=36),
                [superellipse_profile(0.020, 0.020, exponent=2.0, segments=28)],
                0.010,
                center=False,
            ),
            "bearing_collar_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=jar_base_dark,
        name="bearing_collar",
    )
    jar.visual(
        Box((0.016, 0.022, 0.018)),
        origin=Origin(xyz=(0.072, 0.026, 0.182)),
        material=jar_base_dark,
        name="handle_upper_mount",
    )
    jar.visual(
        Box((0.016, 0.022, 0.018)),
        origin=Origin(xyz=(0.072, 0.024, 0.052)),
        material=jar_base_dark,
        name="handle_lower_mount",
    )
    jar.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.078, 0.026, 0.182),
                    (0.098, 0.056, 0.164),
                    (0.106, 0.066, 0.118),
                    (0.104, 0.062, 0.076),
                    (0.096, 0.052, 0.054),
                    (0.078, 0.024, 0.052),
                ],
                radius=0.0075,
                samples_per_segment=14,
                radial_segments=18,
            ),
            "jar_handle",
        ),
        material=jar_base_dark,
        name="handle",
    )
    jar.inertial = Inertial.from_geometry(
        Box((0.160, 0.160, 0.242)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.156, 0.134, 0.004)),
        origin=Origin(xyz=(0.0, -0.067, 0.002)),
        material=lid_clear,
        name="top_panel",
    )
    lid.visual(
        Box((0.148, 0.008, 0.078)),
        origin=Origin(xyz=(0.0, -0.141, -0.035)),
        material=lid_clear,
        name="front_shield",
    )
    lid.visual(
        Box((0.008, 0.074, 0.078)),
        origin=Origin(xyz=(-0.074, -0.113, -0.035)),
        material=lid_clear,
        name="left_shield",
    )
    lid.visual(
        Box((0.008, 0.074, 0.078)),
        origin=Origin(xyz=(0.074, -0.113, -0.035)),
        material=lid_clear,
        name="right_shield",
    )
    lid.visual(
        Box((0.120, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.138, -0.002)),
        material=lid_clear,
        name="front_seal",
    )
    lid.visual(
        Box((0.010, 0.064, 0.004)),
        origin=Origin(xyz=(-0.071, -0.111, -0.002)),
        material=lid_clear,
        name="left_seal",
    )
    lid.visual(
        Box((0.010, 0.064, 0.004)),
        origin=Origin(xyz=(0.071, -0.111, -0.002)),
        material=lid_clear,
        name="right_seal",
    )
    lid.visual(
        Box((0.144, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.005, 0.007)),
        material=lid_clear,
        name="rear_lip",
    )
    lid.visual(
        Box((0.058, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, -0.143, -0.020)),
        material=housing_trim,
        name="front_grip",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.160, 0.140, 0.090)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.067, -0.028)),
    )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=blade_steel,
        name="hub",
    )
    blade_assembly.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=blade_steel,
        name="shaft",
    )
    blade_assembly.visual(
        Box((0.042, 0.010, 0.003)),
        origin=Origin(xyz=(0.015, 0.0, 0.009), rpy=(0.0, 0.30, 0.0)),
        material=blade_steel,
        name="blade_east",
    )
    blade_assembly.visual(
        Box((0.042, 0.010, 0.003)),
        origin=Origin(xyz=(-0.015, 0.0, 0.009), rpy=(0.0, -0.30, 0.0)),
        material=blade_steel,
        name="blade_west",
    )
    blade_assembly.visual(
        Box((0.010, 0.042, 0.003)),
        origin=Origin(xyz=(0.0, 0.015, 0.009), rpy=(-0.30, 0.0, 0.0)),
        material=blade_steel,
        name="blade_north",
    )
    blade_assembly.visual(
        Box((0.010, 0.042, 0.003)),
        origin=Origin(xyz=(0.0, -0.015, 0.009), rpy=(0.30, 0.0, 0.0)),
        material=blade_steel,
        name="blade_south",
    )
    blade_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.035, length=0.026),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    safety_latch = model.part("safety_latch")
    safety_latch.visual(
        Cylinder(radius=0.006, length=0.056),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_trim,
        name="pivot_barrel",
    )
    safety_latch.visual(
        Box((0.052, 0.009, 0.098)),
        origin=Origin(xyz=(0.0, -0.0045, -0.050)),
        material=housing_trim,
        name="latch_body",
    )
    safety_latch.visual(
        Box((0.032, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, -0.014, -0.005)),
        material=housing_trim,
        name="latch_hook",
    )
    safety_latch.visual(
        Box((0.040, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.012, -0.100)),
        material=housing_trim,
        name="release_tab",
    )
    safety_latch.inertial = Inertial.from_geometry(
        Box((0.060, 0.030, 0.120)),
        mass=0.12,
        origin=Origin(xyz=(0.0, -0.010, -0.050)),
    )

    model.articulation(
        "housing_to_jar",
        ArticulationType.FIXED,
        parent=housing,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, 0.352)),
    )
    model.articulation(
        "jar_to_lid",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=lid,
        origin=Origin(xyz=(0.0, 0.067, 0.238)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "jar_to_blade",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=blade_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=30.0,
            lower=0.0,
            upper=math.tau,
        ),
    )
    model.articulation(
        "housing_to_latch",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=safety_latch,
        origin=Origin(xyz=(0.0, -0.111, 0.306)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.0,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    housing = object_model.get_part("housing")
    jar = object_model.get_part("jar")
    lid = object_model.get_part("lid")
    blade_assembly = object_model.get_part("blade_assembly")
    safety_latch = object_model.get_part("safety_latch")

    lid_hinge = object_model.get_articulation("jar_to_lid")
    blade_joint = object_model.get_articulation("jar_to_blade")
    latch_hinge = object_model.get_articulation("housing_to_latch")

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

    part_names = {part.name for part in object_model.parts}
    joint_names = {joint.name for joint in object_model.articulations}
    ctx.check(
        "core_parts_present",
        {"housing", "jar", "lid", "blade_assembly", "safety_latch"}.issubset(part_names),
        details=f"Present parts: {sorted(part_names)}",
    )
    ctx.check(
        "core_joints_present",
        {"housing_to_jar", "jar_to_lid", "jar_to_blade", "housing_to_latch"}.issubset(joint_names),
        details=f"Present joints: {sorted(joint_names)}",
    )

    ctx.check(
        "lid_hinge_axis_is_horizontal",
        tuple(lid_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"Axis was {lid_hinge.axis}",
    )
    ctx.check(
        "blade_axis_is_vertical",
        tuple(blade_joint.axis) == (0.0, 0.0, 1.0),
        details=f"Axis was {blade_joint.axis}",
    )
    ctx.check(
        "latch_hinge_axis_is_horizontal",
        tuple(latch_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"Axis was {latch_hinge.axis}",
    )

    ctx.expect_contact(jar, housing, name="jar_seated_on_housing")
    ctx.expect_overlap(jar, housing, axes="xy", min_overlap=0.08, name="jar_centered_over_housing")
    ctx.expect_contact(lid, jar, name="lid_closed_on_jar")
    ctx.expect_overlap(lid, jar, axes="xy", min_overlap=0.10, name="lid_covers_jar_opening")
    ctx.expect_contact(blade_assembly, jar, name="blade_hub_seated_in_bearing")
    ctx.expect_within(blade_assembly, jar, axes="xy", margin=0.0, name="blade_stays_within_jar")
    ctx.expect_contact(safety_latch, housing, name="latch_seated_on_front_face")
    ctx.expect_overlap(safety_latch, housing, axes="xz", min_overlap=0.04, name="latch_covers_mount")

    lid_closed = ctx.part_element_world_aabb(lid, elem="front_shield")
    assert lid_closed is not None
    lid_closed_center = _aabb_center(lid_closed)
    with ctx.pose({lid_hinge: 1.30}):
        lid_open = ctx.part_element_world_aabb(lid, elem="front_shield")
        assert lid_open is not None
        lid_open_center = _aabb_center(lid_open)
        ctx.check(
            "lid_front_lifts_when_opened",
            lid_open_center[2] > lid_closed_center[2] + 0.070,
            details=f"Closed center z={lid_closed_center[2]:.4f}, open center z={lid_open_center[2]:.4f}",
        )
        ctx.expect_gap(
            lid,
            jar,
            axis="z",
            min_gap=0.040,
            positive_elem="front_shield",
            name="open_lid_front_clears_jar",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_with_lid_open")

    latch_closed = ctx.part_element_world_aabb(safety_latch, elem="release_tab")
    assert latch_closed is not None
    latch_closed_center = _aabb_center(latch_closed)
    with ctx.pose({latch_hinge: 0.75}):
        latch_open = ctx.part_element_world_aabb(safety_latch, elem="release_tab")
        assert latch_open is not None
        latch_open_center = _aabb_center(latch_open)
        ctx.check(
            "latch_swings_forward_when_opened",
            latch_open_center[1] < latch_closed_center[1] - 0.030,
            details=f"Closed center y={latch_closed_center[1]:.4f}, open center y={latch_open_center[1]:.4f}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_with_latch_open")

    blade_closed = ctx.part_element_world_aabb(blade_assembly, elem="blade_east")
    assert blade_closed is not None
    blade_closed_center = _aabb_center(blade_closed)
    with ctx.pose({blade_joint: 0.80}):
        blade_open = ctx.part_element_world_aabb(blade_assembly, elem="blade_east")
        assert blade_open is not None
        blade_open_center = _aabb_center(blade_open)
        ctx.check(
            "blade_rotates_about_spindle",
            blade_open_center[1] > blade_closed_center[1] + 0.010,
            details=(
                f"Closed center={blade_closed_center}, "
                f"rotated center={blade_open_center}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
