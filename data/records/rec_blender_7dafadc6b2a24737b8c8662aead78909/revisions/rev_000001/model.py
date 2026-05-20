from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="immersion_blender_stand_unit")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.77, 0.79, 0.82, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.42, 0.44, 0.47, 1.0))

    housing_height = 0.34
    dock_top = 0.32
    lift_travel = 0.12

    def build_housing_shell():
        outer_profile = [
            (0.078, 0.000),
            (0.084, 0.018),
            (0.080, 0.080),
            (0.078, 0.255),
            (0.074, housing_height),
        ]
        inner_profile = [
            (0.063, 0.008),
            (0.069, 0.022),
            (0.065, 0.080),
            (0.063, 0.292),
            (0.058, housing_height - 0.012),
        ]
        return mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                outer_profile,
                inner_profile,
                segments=72,
            ),
            "housing_shell",
        )

    def build_guard_frame():
        top_center = (-0.012, 0.022, -0.014)
        bottom_center = (-0.014, 0.026, -0.040)
        top_radius = 0.014
        bottom_radius = 0.020

        frame = tube_from_spline_points(
            [
                (0.0, 0.0, 0.0),
                (-0.003, 0.010, -0.004),
                (-0.007, 0.018, -0.010),
                top_center,
            ],
            radius=0.0019,
            samples_per_segment=12,
            radial_segments=14,
        )

        frame.merge(
            TorusGeometry(top_radius, 0.0013, radial_segments=18, tubular_segments=42).translate(
                *top_center
            )
        )
        frame.merge(
            TorusGeometry(
                bottom_radius,
                0.0015,
                radial_segments=18,
                tubular_segments=46,
            ).translate(*bottom_center)
        )

        for angle_deg in (30, 120, 210, 320):
            angle = math.radians(angle_deg)
            c = math.cos(angle)
            s = math.sin(angle)
            rib = tube_from_spline_points(
                [
                    (
                        top_center[0] + top_radius * c,
                        top_center[1] + top_radius * s,
                        top_center[2],
                    ),
                    (
                        -0.010,
                        0.5 * (top_center[1] + bottom_center[1]) + 0.5 * (top_radius + bottom_radius) * s,
                        -0.027,
                    ),
                    (
                        bottom_center[0] + bottom_radius * c,
                        bottom_center[1] + bottom_radius * s,
                        bottom_center[2],
                    ),
                ],
                radius=0.00125,
                samples_per_segment=12,
                radial_segments=12,
            )
            frame.merge(rib)

        return mesh_from_geometry(frame, "blade_guard_frame_v4")

    base_housing = model.part("base_housing")
    base_housing.visual(
        build_housing_shell(),
        material=shell_white,
        name="housing_shell",
    )
    base_housing.visual(
        Cylinder(radius=0.092, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=charcoal,
        name="base_foot",
    )
    base_housing.visual(
        Box((0.032, 0.050, 0.240)),
        origin=Origin(xyz=(0.060, 0.0, 0.180)),
        material=charcoal,
        name="dock_spine",
    )
    base_housing.visual(
        Box((0.020, 0.008, 0.220)),
        origin=Origin(xyz=(0.080, 0.024, 0.190)),
        material=charcoal,
        name="guide_rail_left",
    )
    base_housing.visual(
        Box((0.020, 0.008, 0.220)),
        origin=Origin(xyz=(0.080, -0.024, 0.190)),
        material=charcoal,
        name="guide_rail_right",
    )
    base_housing.visual(
        Box((0.026, 0.050, 0.016)),
        origin=Origin(xyz=(0.056, 0.0, dock_top - 0.008)),
        material=shell_white,
        name="dock_crown",
    )
    base_housing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.092, length=housing_height),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, housing_height / 2.0)),
    )

    wand_arm = model.part("wand_arm")
    wand_arm.visual(
        Box((0.012, 0.022, 0.240)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=charcoal,
        name="guide_spine",
    )
    wand_arm.visual(
        Box((0.010, 0.010, 0.220)),
        origin=Origin(xyz=(0.0, 0.015, -0.020)),
        material=charcoal,
        name="guide_fin_left",
    )
    wand_arm.visual(
        Box((0.010, 0.010, 0.220)),
        origin=Origin(xyz=(0.0, -0.015, -0.020)),
        material=charcoal,
        name="guide_fin_right",
    )
    wand_arm.visual(
        Cylinder(radius=0.028, length=0.140),
        origin=Origin(xyz=(0.030, 0.0, 0.110)),
        material=charcoal,
        name="motor_grip",
    )
    wand_arm.visual(
        Sphere(radius=0.028),
        origin=Origin(xyz=(0.030, 0.0, 0.180)),
        material=charcoal,
        name="grip_cap",
    )
    wand_arm.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.022, 0.0, 0.015)),
        material=dark_steel,
        name="neck_collar",
    )
    wand_arm.visual(
        Cylinder(radius=0.011, length=0.180),
        origin=Origin(xyz=(0.016, 0.0, -0.105)),
        material=steel,
        name="blending_shaft",
    )
    wand_arm.visual(
        Cylinder(radius=0.014, length=0.038),
        origin=Origin(xyz=(0.016, 0.0, -0.152)),
        material=steel,
        name="tip_mount",
    )
    wand_arm.visual(
        Cylinder(radius=0.0045, length=0.028),
        origin=Origin(xyz=(0.016, 0.0, -0.186)),
        material=dark_steel,
        name="blade_spindle",
    )
    wand_arm.visual(
        Box((0.028, 0.005, 0.002)),
        origin=Origin(xyz=(0.016, 0.0, -0.190)),
        material=dark_steel,
        name="blade_1",
    )
    wand_arm.visual(
        Box((0.005, 0.028, 0.002)),
        origin=Origin(xyz=(0.016, 0.0, -0.190)),
        material=dark_steel,
        name="blade_2",
    )
    wand_arm.visual(
        Box((0.024, 0.012, 0.020)),
        origin=Origin(xyz=(0.020, 0.009, -0.148)),
        material=dark_steel,
        name="mount_back",
    )
    wand_arm.visual(
        Box((0.024, 0.012, 0.005)),
        origin=Origin(xyz=(0.036, 0.009, -0.1405)),
        material=dark_steel,
        name="mount_upper_clamp",
    )
    wand_arm.visual(
        Box((0.024, 0.012, 0.005)),
        origin=Origin(xyz=(0.036, 0.009, -0.1555)),
        material=dark_steel,
        name="mount_lower_clamp",
    )
    wand_arm.inertial = Inertial.from_geometry(
        Box((0.090, 0.060, 0.430)),
        mass=0.95,
        origin=Origin(xyz=(0.026, 0.0, -0.020)),
    )

    blade_guard_cage = model.part("blade_guard_cage")
    blade_guard_cage.visual(
        Cylinder(radius=0.005, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    blade_guard_cage.visual(
        build_guard_frame(),
        material=steel,
        name="guard_frame",
    )
    blade_guard_cage.inertial = Inertial.from_geometry(
        Box((0.065, 0.070, 0.055)),
        mass=0.14,
        origin=Origin(xyz=(-0.012, 0.024, -0.025)),
    )

    model.articulation(
        "housing_to_wand",
        ArticulationType.PRISMATIC,
        parent=base_housing,
        child=wand_arm,
        origin=Origin(xyz=(0.090, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.20,
            lower=0.0,
            upper=lift_travel,
        ),
    )
    model.articulation(
        "wand_to_guard_cage",
        ArticulationType.REVOLUTE,
        parent=wand_arm,
        child=blade_guard_cage,
        origin=Origin(xyz=(0.048, 0.015, -0.148)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.45,
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

    base_housing = object_model.get_part("base_housing")
    wand_arm = object_model.get_part("wand_arm")
    blade_guard_cage = object_model.get_part("blade_guard_cage")

    housing_to_wand = object_model.get_articulation("housing_to_wand")
    wand_to_guard_cage = object_model.get_articulation("wand_to_guard_cage")

    base_aabb = ctx.part_world_aabb(base_housing)
    wand_aabb = ctx.part_world_aabb(wand_arm)
    guard_aabb = ctx.part_world_aabb(blade_guard_cage)
    assert base_aabb is not None
    assert wand_aabb is not None
    assert guard_aabb is not None
    assert base_aabb[1][2] - base_aabb[0][2] > 0.33
    assert wand_aabb[1][2] - wand_aabb[0][2] > 0.38
    assert guard_aabb[1][2] - guard_aabb[0][2] > 0.03

    with ctx.pose({housing_to_wand: 0.0, wand_to_guard_cage: 0.0}):
        ctx.expect_contact(
            wand_arm,
            base_housing,
            elem_a="guide_fin_left",
            elem_b="guide_rail_left",
            name="wand_docked_left_rail_contact",
        )
        ctx.expect_contact(
            wand_arm,
            base_housing,
            elem_a="guide_fin_right",
            elem_b="guide_rail_right",
            name="wand_docked_right_rail_contact",
        )
        ctx.expect_contact(
            blade_guard_cage,
            wand_arm,
            elem_a="hinge_barrel",
            elem_b="mount_upper_clamp",
            name="guard_closed_hinge_contact",
        )
        ctx.expect_origin_distance(wand_arm, base_housing, axes="xy", max_dist=0.12, name="wand_stays_near_housing")

    wand_rest_pos = ctx.part_world_position(wand_arm)
    assert wand_rest_pos is not None
    with ctx.pose({housing_to_wand: 0.06, wand_to_guard_cage: 0.0}):
        ctx.expect_contact(
            wand_arm,
            base_housing,
            elem_a="guide_fin_left",
            elem_b="guide_rail_left",
            name="wand_midstroke_left_rail_contact",
        )
        ctx.expect_contact(
            wand_arm,
            base_housing,
            elem_a="guide_fin_right",
            elem_b="guide_rail_right",
            name="wand_midstroke_right_rail_contact",
        )

    lift_limits = housing_to_wand.motion_limits
    assert lift_limits is not None
    assert lift_limits.upper is not None
    with ctx.pose({housing_to_wand: lift_limits.upper, wand_to_guard_cage: 0.0}):
        wand_upper_pos = ctx.part_world_position(wand_arm)
        assert wand_upper_pos is not None
        assert wand_upper_pos[2] > wand_rest_pos[2] + 0.11
        ctx.expect_contact(
            wand_arm,
            base_housing,
            elem_a="guide_fin_left",
            elem_b="guide_rail_left",
            name="wand_upper_left_rail_contact",
        )
        ctx.expect_contact(
            wand_arm,
            base_housing,
            elem_a="guide_fin_right",
            elem_b="guide_rail_right",
            name="wand_upper_right_rail_contact",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="wand_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="wand_upper_no_floating")

    guard_closed_aabb = ctx.part_world_aabb(blade_guard_cage)
    assert guard_closed_aabb is not None
    guard_limits = wand_to_guard_cage.motion_limits
    assert guard_limits is not None
    assert guard_limits.upper is not None
    with ctx.pose({housing_to_wand: 0.0, wand_to_guard_cage: guard_limits.upper}):
        guard_open_aabb = ctx.part_world_aabb(blade_guard_cage)
        assert guard_open_aabb is not None
        assert guard_open_aabb[1][2] > guard_closed_aabb[1][2] + 0.01
        ctx.expect_contact(
            blade_guard_cage,
            wand_arm,
            elem_a="hinge_barrel",
            elem_b="mount_upper_clamp",
            name="guard_open_hinge_contact",
        )
        assert guard_open_aabb[0][0] < guard_closed_aabb[0][0] - 0.005
        ctx.fail_if_parts_overlap_in_current_pose(name="guard_open_no_overlap")
        ctx.fail_if_isolated_parts(name="guard_open_no_floating")

    with ctx.pose({housing_to_wand: lift_limits.upper, wand_to_guard_cage: guard_limits.upper}):
        ctx.expect_contact(
            wand_arm,
            base_housing,
            elem_a="guide_fin_left",
            elem_b="guide_rail_left",
            name="wand_upper_open_guard_left_rail_contact",
        )
        ctx.expect_contact(
            wand_arm,
            base_housing,
            elem_a="guide_fin_right",
            elem_b="guide_rail_right",
            name="wand_upper_open_guard_right_rail_contact",
        )
        ctx.expect_contact(
            blade_guard_cage,
            wand_arm,
            elem_a="hinge_barrel",
            elem_b="mount_upper_clamp",
            name="guard_open_with_wand_raised_hinge_contact",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_open_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_open_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
