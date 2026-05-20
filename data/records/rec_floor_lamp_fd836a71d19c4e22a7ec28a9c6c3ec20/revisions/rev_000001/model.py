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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_arm_floor_lamp")

    base_metal = model.material("base_metal", rgba=(0.16, 0.16, 0.18, 1.0))
    arm_metal = model.material("arm_metal", rgba=(0.60, 0.52, 0.37, 1.0))
    joint_metal = model.material("joint_metal", rgba=(0.44, 0.44, 0.46, 1.0))
    shade_finish = model.material("shade_finish", rgba=(0.92, 0.90, 0.83, 1.0))
    socket_dark = model.material("socket_dark", rgba=(0.14, 0.14, 0.15, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.16, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=base_metal,
        name="weighted_base",
    )
    base.visual(
        Cylinder(radius=0.155, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=socket_dark,
        name="floor_pad",
    )
    base.visual(
        Cylinder(radius=0.07, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=joint_metal,
        name="column_mount",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.16, length=0.04),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.016, length=1.30),
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
        material=arm_metal,
        name="column_tube",
    )
    column.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=joint_metal,
        name="lower_collar",
    )
    column.visual(
        Box((0.038, 0.006, 0.030)),
        origin=Origin(xyz=(0.025, 0.016, 1.240)),
        material=joint_metal,
        name="shoulder_support_left",
    )
    column.visual(
        Box((0.038, 0.006, 0.030)),
        origin=Origin(xyz=(0.025, -0.016, 1.240)),
        material=joint_metal,
        name="shoulder_support_right",
    )
    column.visual(
        Box((0.024, 0.006, 0.042)),
        origin=Origin(xyz=(0.056, 0.018, 1.270)),
        material=joint_metal,
        name="shoulder_clevis_left",
    )
    column.visual(
        Box((0.024, 0.006, 0.042)),
        origin=Origin(xyz=(0.056, -0.018, 1.270)),
        material=joint_metal,
        name="shoulder_clevis_right",
    )
    column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.03, length=1.30),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_metal,
        name="shoulder_knuckle",
    )
    upper_arm.visual(
        Box((0.38, 0.018, 0.012)),
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        material=arm_metal,
        name="upper_arm_beam",
    )
    upper_arm.visual(
        Box((0.024, 0.006, 0.036)),
        origin=Origin(xyz=(0.407, 0.012, 0.0)),
        material=joint_metal,
        name="elbow_clevis_left",
    )
    upper_arm.visual(
        Box((0.024, 0.006, 0.036)),
        origin=Origin(xyz=(0.407, -0.012, 0.0)),
        material=joint_metal,
        name="elbow_clevis_right",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.43, 0.04, 0.05)),
        mass=0.8,
        origin=Origin(xyz=(0.215, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_metal,
        name="elbow_knuckle",
    )
    forearm.visual(
        Box((0.355, 0.018, 0.012)),
        origin=Origin(xyz=(0.1925, 0.0, 0.0)),
        material=arm_metal,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.026, 0.006, 0.028)),
        origin=Origin(xyz=(0.383, 0.012, 0.0)),
        material=joint_metal,
        name="shade_clevis_left",
    )
    forearm.visual(
        Box((0.026, 0.006, 0.028)),
        origin=Origin(xyz=(0.383, -0.012, 0.0)),
        material=joint_metal,
        name="shade_clevis_right",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.40, 0.04, 0.04)),
        mass=0.7,
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
    )

    shade_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.0, 0.0),
                (0.022, 0.012),
                (0.048, 0.034),
                (0.082, 0.088),
                (0.108, 0.155),
            ],
            [
                (0.0, 0.008),
                (0.016, 0.020),
                (0.040, 0.042),
                (0.072, 0.094),
                (0.095, 0.150),
            ],
            segments=64,
        ),
        "shade_shell",
    )
    shade_pitch = math.pi / 2.0 + 0.33

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_metal,
        name="shade_knuckle",
    )
    shade.visual(
        Box((0.024, 0.014, 0.010)),
        origin=Origin(xyz=(0.012, 0.0, -0.004)),
        material=joint_metal,
        name="shade_yoke",
    )
    shade.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(
            xyz=(0.033, 0.0, -0.012),
            rpy=(0.0, shade_pitch, 0.0),
        ),
        material=joint_metal,
        name="shade_collar",
    )
    shade.visual(
        shade_shell,
        origin=Origin(
            xyz=(0.045, 0.0, -0.016),
            rpy=(0.0, shade_pitch, 0.0),
        ),
        material=shade_finish,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(
            xyz=(0.060, 0.0, -0.021),
            rpy=(0.0, shade_pitch, 0.0),
        ),
        material=socket_dark,
        name="lamp_socket",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.24, 0.22, 0.18)),
        mass=0.9,
        origin=Origin(xyz=(0.10, 0.0, -0.05)),
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )
    model.articulation(
        "column_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=column,
        child=upper_arm,
        origin=Origin(xyz=(0.044, 0.0, 1.270)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.3,
            lower=-0.55,
            upper=1.10,
        ),
    )
    model.articulation(
        "upper_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.410, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.6,
            lower=-0.10,
            upper=2.20,
        ),
    )
    model.articulation(
        "forearm_to_shade",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=shade,
        origin=Origin(xyz=(0.383, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=-0.90,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    shade = object_model.get_part("shade")

    shoulder = object_model.get_articulation("column_to_upper_arm")
    elbow = object_model.get_articulation("upper_to_forearm")
    shade_tilt = object_model.get_articulation("forearm_to_shade")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(column, base, name="column_seated_on_base")
    ctx.expect_contact(upper_arm, column, name="shoulder_joint_connected")
    ctx.expect_contact(forearm, upper_arm, name="elbow_joint_connected")
    ctx.expect_contact(shade, forearm, name="shade_joint_connected")

    ctx.check(
        "shoulder_axis_matches_vertical_plane_motion",
        shoulder.axis == (0.0, -1.0, 0.0),
        details=f"axis={shoulder.axis}",
    )
    ctx.check(
        "elbow_axis_matches_vertical_plane_motion",
        elbow.axis == (0.0, -1.0, 0.0),
        details=f"axis={elbow.axis}",
    )
    ctx.check(
        "shade_axis_matches_vertical_plane_motion",
        shade_tilt.axis == (0.0, -1.0, 0.0),
        details=f"axis={shade_tilt.axis}",
    )

    shade_rest = ctx.part_world_position(shade)
    if shade_rest is None:
        ctx.fail("shade_rest_position_available", "shade part world position unavailable")
    else:
        with ctx.pose({shoulder: 0.70}):
            shade_raised = ctx.part_world_position(shade)
            ctx.check(
                "shoulder_raises_lamp_head",
                shade_raised is not None and shade_raised[2] > shade_rest[2] + 0.18,
                details=f"rest={shade_rest}, raised={shade_raised}",
            )
            ctx.expect_contact(upper_arm, column, name="shoulder_contact_in_raised_pose")

        with ctx.pose({elbow: 1.10}):
            shade_folded = ctx.part_world_position(shade)
            ctx.check(
                "elbow_reduces_forward_reach",
                shade_folded is not None and shade_folded[0] < shade_rest[0] - 0.10,
                details=f"rest={shade_rest}, folded={shade_folded}",
            )
            ctx.expect_contact(forearm, upper_arm, name="elbow_contact_in_folded_pose")

    shade_rest_aabb = ctx.part_world_aabb(shade)
    if shade_rest_aabb is None:
        ctx.fail("shade_rest_aabb_available", "shade world AABB unavailable")
    else:
        with ctx.pose({shade_tilt: -0.60}):
            tilted_aabb = ctx.part_world_aabb(shade)
            ctx.check(
                "shade_tilt_changes_shade_envelope",
                tilted_aabb is not None
                and abs(tilted_aabb[0][2] - shade_rest_aabb[0][2]) > 0.025,
                details=f"rest={shade_rest_aabb}, tilted={tilted_aabb}",
            )
            ctx.expect_contact(shade, forearm, name="shade_contact_in_tilt_pose")

    with ctx.pose({shoulder: 0.55, elbow: 0.95, shade_tilt: -0.35}):
        ctx.expect_gap(
            shade,
            base,
            axis="z",
            min_gap=0.70,
            name="shade_clears_weighted_base_in_task_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
