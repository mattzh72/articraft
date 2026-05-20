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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_ring_light_boom_stand")

    stand_black = model.material("stand_black", rgba=(0.14, 0.15, 0.16, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.94, 0.95, 0.92, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.20, 0.21, 0.23, 1.0))

    lower_sleeve_bottom = 0.07
    wheel_center_radius = 0.40
    wheel_radius = 0.045
    ring_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.153, -0.025),
                (0.187, -0.025),
                (0.198, -0.018),
                (0.204, 0.0),
                (0.198, 0.018),
                (0.187, 0.025),
                (0.153, 0.025),
            ],
            [
                (0.168, -0.012),
                (0.186, -0.012),
                (0.193, -0.008),
                (0.196, 0.0),
                (0.193, 0.008),
                (0.186, 0.012),
                (0.168, 0.012),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "ring_light_annulus",
    )
    rear_trim_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.194, tube=0.006, radial_segments=12, tubular_segments=64),
        "ring_light_rear_trim",
    )

    rolling_base = model.part("rolling_base")
    rolling_base.visual(
        Cylinder(radius=0.11, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=stand_black,
        name="hub_drum",
    )
    rolling_base.visual(
        Cylinder(radius=0.035, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=satin_steel,
        name="mast_riser",
    )
    rolling_base.visual(
        Box((0.070, 0.055, 0.030)),
        origin=Origin(xyz=(0.0, -0.003, 0.115)),
        material=trim_dark,
        name="lower_column_sleeve",
    )
    rolling_base.visual(
        Box((0.008, 0.020, 0.280)),
        origin=Origin(xyz=(-0.030, 0.0, 0.270)),
        material=satin_steel,
        name="guide_left",
    )
    rolling_base.visual(
        Box((0.008, 0.020, 0.280)),
        origin=Origin(xyz=(0.030, 0.0, 0.270)),
        material=satin_steel,
        name="guide_right",
    )
    rolling_base.visual(
        Box((0.052, 0.008, 0.280)),
        origin=Origin(xyz=(0.0, -0.030, 0.270)),
        material=satin_steel,
        name="guide_back",
    )
    rolling_base.visual(
        Box((0.070, 0.055, 0.028)),
        origin=Origin(xyz=(0.0, -0.003, 0.424)),
        material=trim_dark,
        name="guide_clamp_head",
    )
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        c = math.cos(angle)
        s = math.sin(angle)
        rolling_base.visual(
            Box((0.240, 0.036, 0.028)),
            origin=Origin(xyz=(0.225 * c, 0.225 * s, 0.058), rpy=(0.0, 0.0, angle)),
            material=stand_black,
            name=f"leg_{index}",
        )
        rolling_base.visual(
            Box((0.010, 0.006, 0.120)),
            origin=Origin(
                xyz=(wheel_center_radius * c + 0.016 * s, wheel_center_radius * s - 0.016 * c, 0.060),
                rpy=(0.0, 0.0, angle),
            ),
            material=stand_black,
            name=f"fork_left_{index}",
        )
        rolling_base.visual(
            Box((0.010, 0.006, 0.120)),
            origin=Origin(
                xyz=(wheel_center_radius * c - 0.016 * s, wheel_center_radius * s + 0.016 * c, 0.060),
                rpy=(0.0, 0.0, angle),
            ),
            material=stand_black,
            name=f"fork_right_{index}",
        )
        rolling_base.visual(
            Box((0.022, 0.030, 0.010)),
            origin=Origin(
                xyz=(wheel_center_radius * c, wheel_center_radius * s, 0.115),
                rpy=(0.0, 0.0, angle),
            ),
            material=stand_black,
            name=f"fork_bridge_{index}",
        )
        rolling_base.visual(
            Box((0.110, 0.022, 0.010)),
            origin=Origin(
                xyz=(0.345 * c, 0.345 * s, 0.115),
                rpy=(0.0, 0.0, angle),
            ),
            material=stand_black,
            name=f"caster_neck_{index}",
        )
        rolling_base.visual(
            Box((0.018, 0.022, 0.040)),
            origin=Origin(
                xyz=(0.345 * c, 0.345 * s, 0.091),
                rpy=(0.0, 0.0, angle),
            ),
            material=stand_black,
            name=f"caster_riser_{index}",
        )
        rolling_base.visual(
            Cylinder(radius=0.010, length=0.014),
            origin=Origin(
                xyz=(wheel_center_radius * c, wheel_center_radius * s, wheel_radius),
                rpy=(math.pi / 2.0, 0.0, angle),
            ),
            material=trim_dark,
            name=f"axle_spindle_{index}",
        )
    rolling_base.inertial = Inertial.from_geometry(
        Box((0.84, 0.84, 1.00)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
    )

    for index in range(3):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            Cylinder(radius=wheel_radius, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber_black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=trim_dark,
            name="axle_hub",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=wheel_radius, length=0.018),
            mass=0.25,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
        model.articulation(
            f"base_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=rolling_base,
            child=wheel,
            origin=Origin(
                xyz=(
                    wheel_center_radius * math.cos(index * (2.0 * math.pi / 3.0)),
                    wheel_center_radius * math.sin(index * (2.0 * math.pi / 3.0)),
                    wheel_radius,
                ),
                rpy=(0.0, 0.0, index * (2.0 * math.pi / 3.0)),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=20.0),
        )

    upper_column = model.part("upper_column")
    upper_column.visual(
        Cylinder(radius=0.024, length=0.90),
        origin=Origin(xyz=(0.0, 0.0, 0.834)),
        material=satin_steel,
        name="upper_tube",
    )
    upper_column.visual(
        Cylinder(radius=0.035, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.376)),
        material=trim_dark,
        name="travel_collar",
    )
    upper_column.visual(
        Cylinder(radius=0.045, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 1.244)),
        material=stand_black,
        name="boom_mount_head",
    )
    upper_column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=1.13),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.565)),
    )
    model.articulation(
        "column_extension",
        ArticulationType.PRISMATIC,
        parent=rolling_base,
        child=upper_column,
        origin=Origin(xyz=(0.0, 0.0, lower_sleeve_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.18,
            lower=0.0,
            upper=0.30,
        ),
    )

    boom_arm = model.part("boom_arm")
    boom_arm.visual(
        Cylinder(radius=0.050, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=stand_black,
        name="yaw_housing",
    )
    boom_arm.visual(
        Box((0.78, 0.050, 0.030)),
        origin=Origin(xyz=(0.39, 0.0, 0.045)),
        material=stand_black,
        name="main_beam",
    )
    boom_arm.visual(
        Box((0.24, 0.040, 0.026)),
        origin=Origin(xyz=(-0.13, 0.0, 0.043)),
        material=stand_black,
        name="rear_beam",
    )
    boom_arm.visual(
        Cylinder(radius=0.045, length=0.150),
        origin=Origin(xyz=(-0.27, 0.0, 0.043), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="counterweight",
    )
    boom_arm.visual(
        Box((0.60, 0.028, 0.020)),
        origin=Origin(xyz=(0.33, 0.0, 0.018)),
        material=trim_dark,
        name="lower_brace",
    )
    boom_arm.visual(
        Box((0.040, 0.004, 0.080)),
        origin=Origin(xyz=(0.80, -0.027, 0.040)),
        material=stand_black,
        name="tip_yoke_left",
    )
    boom_arm.visual(
        Box((0.040, 0.004, 0.080)),
        origin=Origin(xyz=(0.80, 0.027, 0.040)),
        material=stand_black,
        name="tip_yoke_right",
    )
    boom_arm.visual(
        Box((0.020, 0.058, 0.012)),
        origin=Origin(xyz=(0.775, 0.0, 0.076)),
        material=stand_black,
        name="tip_yoke_bridge",
    )
    boom_arm.inertial = Inertial.from_geometry(
        Box((1.05, 0.15, 0.14)),
        mass=3.4,
        origin=Origin(xyz=(0.23, 0.0, 0.05)),
    )
    model.articulation(
        "boom_yaw",
        ArticulationType.REVOLUTE,
        parent=upper_column,
        child=boom_arm,
        origin=Origin(xyz=(0.0, 0.0, 1.284)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.0,
            lower=-math.radians(100.0),
            upper=math.radians(100.0),
        ),
    )

    light_head = model.part("light_head")
    light_head.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="tilt_trunnion",
    )
    light_head.visual(
        Box((0.045, 0.045, 0.090)),
        origin=Origin(xyz=(0.0225, 0.0, 0.0)),
        material=stand_black,
        name="rear_hub",
    )
    light_head.visual(
        Box((0.150, 0.008, 0.060)),
        origin=Origin(xyz=(0.090, -0.022, 0.0)),
        material=stand_black,
        name="support_arm_left",
    )
    light_head.visual(
        Box((0.150, 0.008, 0.060)),
        origin=Origin(xyz=(0.090, 0.022, 0.0)),
        material=stand_black,
        name="support_arm_right",
    )
    light_head.visual(
        Box((0.060, 0.045, 0.045)),
        origin=Origin(xyz=(0.040, 0.0, -0.0675)),
        material=trim_dark,
        name="controller_box",
    )
    light_head.visual(
        Box((0.140, 0.028, 0.020)),
        origin=Origin(xyz=(0.095, 0.0, -0.055)),
        material=stand_black,
        name="lower_spine",
    )
    light_head.visual(
        Box((0.022, 0.034, 0.145)),
        origin=Origin(xyz=(0.138, 0.0, -0.1125)),
        material=stand_black,
        name="lower_mast",
    )
    light_head.visual(
        ring_shell_mesh,
        origin=Origin(xyz=(0.140, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=diffuser_white,
        name="ring_shell",
    )
    light_head.visual(
        rear_trim_mesh,
        origin=Origin(xyz=(0.140, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="rear_trim",
    )
    light_head.inertial = Inertial.from_geometry(
        Box((0.38, 0.42, 0.42)),
        mass=2.2,
        origin=Origin(xyz=(0.14, 0.0, 0.0)),
    )
    model.articulation(
        "light_tilt",
        ArticulationType.REVOLUTE,
        parent=boom_arm,
        child=light_head,
        origin=Origin(xyz=(0.81, 0.0, 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=-math.radians(55.0),
            upper=math.radians(30.0),
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
    rolling_base = object_model.get_part("rolling_base")
    upper_column = object_model.get_part("upper_column")
    boom_arm = object_model.get_part("boom_arm")
    light_head = object_model.get_part("light_head")
    wheels = [object_model.get_part(f"wheel_{index}") for index in range(3)]

    for index, wheel in enumerate(wheels):
        ctx.allow_overlap(
            rolling_base,
            wheel,
            elem_a=f"axle_spindle_{index}",
            elem_b="tire",
            reason="The wheel tire surrounds a small spindle at the hub center in this simplified caster model.",
        )
        ctx.allow_overlap(
            rolling_base,
            wheel,
            elem_a=f"axle_spindle_{index}",
            elem_b="axle_hub",
            reason="The wheel hub rotates around a fixed axle spindle, so the simplified solid hub and spindle intentionally occupy the same center volume.",
        )

    ctx.fail_if_parts_overlap_in_current_pose()

    column_extension = object_model.get_articulation("column_extension")
    boom_yaw = object_model.get_articulation("boom_yaw")
    light_tilt = object_model.get_articulation("light_tilt")
    wheel_joints = [object_model.get_articulation(f"base_to_wheel_{index}") for index in range(3)]

    ctx.expect_contact(upper_column, rolling_base, name="upper_column_seats_in_base")
    ctx.expect_contact(boom_arm, upper_column, name="boom_seats_on_column")
    ctx.expect_contact(light_head, boom_arm, name="light_head_seats_in_tip_yoke")
    for index, wheel in enumerate(wheels):
        ctx.expect_contact(wheel, rolling_base, name=f"wheel_{index}_mounted_to_fork")

    ctx.expect_within(
        upper_column,
        rolling_base,
        axes="xy",
        margin=0.02,
        inner_elem="upper_tube",
        outer_elem="lower_column_sleeve",
        name="upper_column_runs_inside_lower_sleeve",
    )
    ctx.expect_gap(
        light_head,
        rolling_base,
        axis="z",
        min_gap=0.45,
        name="light_head_clears_base_height",
    )

    ctx.check(
        "column_extension_axis_vertical",
        column_extension.axis == (0.0, 0.0, 1.0),
        f"Expected vertical prismatic axis, got {column_extension.axis!r}",
    )
    ctx.check(
        "boom_yaw_axis_vertical",
        boom_yaw.axis == (0.0, 0.0, 1.0),
        f"Expected vertical boom yaw axis, got {boom_yaw.axis!r}",
    )
    ctx.check(
        "light_tilt_axis_horizontal",
        light_tilt.axis in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        f"Expected horizontal light tilt axis, got {light_tilt.axis!r}",
    )
    for index, wheel_joint in enumerate(wheel_joints):
        ctx.check(
            f"wheel_{index}_axle_is_horizontal",
            abs(wheel_joint.axis[2]) < 1e-9,
            f"Wheel joint axis should be horizontal, got {wheel_joint.axis!r}",
        )

    rest_column_pos = ctx.part_world_position(upper_column)
    assert rest_column_pos is not None
    with ctx.pose({column_extension: 0.25}):
        extended_column_pos = ctx.part_world_position(upper_column)
        assert extended_column_pos is not None
        ctx.check(
            "column_extension_moves_upward",
            extended_column_pos[2] > rest_column_pos[2] + 0.20,
            f"Expected upper column to rise, got rest={rest_column_pos}, extended={extended_column_pos}",
        )

    rest_light_pos = ctx.part_world_position(light_head)
    assert rest_light_pos is not None
    with ctx.pose({boom_yaw: math.radians(90.0)}):
        yawed_light_pos = ctx.part_world_position(light_head)
        assert yawed_light_pos is not None
        ctx.check(
            "boom_yaw_swings_light_sideways",
            abs(yawed_light_pos[1] - rest_light_pos[1]) > 0.50,
            f"Expected boom yaw to move light in Y, got rest={rest_light_pos}, yawed={yawed_light_pos}",
        )

    rest_ring_aabb = ctx.part_element_world_aabb(light_head, elem="ring_shell")
    assert rest_ring_aabb is not None
    rest_ring_center_z = 0.5 * (rest_ring_aabb[0][2] + rest_ring_aabb[1][2])
    with ctx.pose({light_tilt: -math.radians(35.0)}):
        tilted_ring_aabb = ctx.part_element_world_aabb(light_head, elem="ring_shell")
        assert tilted_ring_aabb is not None
        tilted_ring_center_z = 0.5 * (tilted_ring_aabb[0][2] + tilted_ring_aabb[1][2])
        ctx.check(
            "light_tilt_changes_head_pitch",
            tilted_ring_center_z > rest_ring_center_z + 0.05,
            f"Expected tilted head to lift ring center, got rest_z={rest_ring_center_z}, tilted_z={tilted_ring_center_z}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
