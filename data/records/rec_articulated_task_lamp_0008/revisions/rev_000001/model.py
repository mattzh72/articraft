from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_task_lamp", assets=ASSETS)

    base_paint = model.material("base_paint", rgba=(0.22, 0.23, 0.25, 1.0))
    arm_paint = model.material("arm_paint", rgba=(0.70, 0.72, 0.74, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    reflector_white = model.material("reflector_white", rgba=(0.93, 0.93, 0.90, 1.0))
    socket_dark = model.material("socket_dark", rgba=(0.20, 0.19, 0.18, 1.0))
    bulb_glass = model.material("bulb_glass", rgba=(0.95, 0.93, 0.82, 0.55))

    base = model.part("base")
    base.visual(
        Box((0.220, 0.160, 0.022)),
        origin=Origin(xyz=(0.000, 0.000, 0.011)),
        material=base_paint,
        name="weight_block",
    )
    base.visual(
        Box((0.170, 0.112, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.028)),
        material=base_paint,
        name="top_plate",
    )
    base.visual(
        Box((0.056, 0.036, 0.010)),
        origin=Origin(xyz=(0.000, -0.048, 0.029)),
        material=base_paint,
        name="rear_pedestal",
    )
    base.visual(
        Box((0.006, 0.020, 0.030)),
        origin=Origin(xyz=(-0.021, -0.048, 0.049)),
        material=hinge_dark,
        name="hinge_left_cheek",
    )
    base.visual(
        Box((0.006, 0.020, 0.030)),
        origin=Origin(xyz=(0.021, -0.048, 0.049)),
        material=hinge_dark,
        name="hinge_right_cheek",
    )
    base.visual(
        Box((0.120, 0.088, 0.003)),
        origin=Origin(xyz=(0.000, 0.000, 0.0015)),
        material=hinge_dark,
        name="bottom_pad",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.220, 0.160, 0.035)),
        mass=3.8,
        origin=Origin(xyz=(0.000, 0.000, 0.0175)),
    )

    lower_arm = model.part("lower_arm")
    lower_span_y = 0.092
    lower_span_z = 0.168
    lower_start_y = 0.010
    lower_start_z = 0.012
    lower_bar_dy = lower_span_y - lower_start_y
    lower_bar_dz = lower_span_z - lower_start_z
    lower_angle = math.atan2(lower_bar_dz, lower_bar_dy)
    lower_length = math.hypot(lower_bar_dy, lower_bar_dz)
    lower_mid = ((lower_span_y + lower_start_y) * 0.5, (lower_span_z + lower_start_z) * 0.5)
    lower_arm.visual(
        Cylinder(radius=0.014, length=0.036),
        origin=Origin(rpy=(0.000, math.pi / 2.0, 0.000)),
        material=hinge_dark,
        name="rear_hub",
    )
    lower_arm.visual(
        Box((0.010, 0.016, 0.016)),
        origin=Origin(xyz=(-0.011, lower_start_y * 0.5, lower_start_z * 0.5)),
        material=hinge_dark,
        name="left_root_block",
    )
    lower_arm.visual(
        Box((0.010, 0.016, 0.016)),
        origin=Origin(xyz=(0.011, lower_start_y * 0.5, lower_start_z * 0.5)),
        material=hinge_dark,
        name="right_root_block",
    )
    lower_arm.visual(
        Box((0.004, lower_length, 0.008)),
        origin=Origin(xyz=(-0.016, lower_mid[0], lower_mid[1]), rpy=(lower_angle, 0.000, 0.000)),
        material=arm_paint,
        name="left_strut",
    )
    lower_arm.visual(
        Box((0.004, lower_length, 0.008)),
        origin=Origin(xyz=(0.016, lower_mid[0], lower_mid[1]), rpy=(lower_angle, 0.000, 0.000)),
        material=arm_paint,
        name="right_strut",
    )
    lower_arm.visual(
        Box((0.006, 0.018, 0.030)),
        origin=Origin(xyz=(-0.021, lower_span_y, lower_span_z)),
        material=hinge_dark,
        name="elbow_left_cheek",
    )
    lower_arm.visual(
        Box((0.006, 0.018, 0.030)),
        origin=Origin(xyz=(0.021, lower_span_y, lower_span_z)),
        material=hinge_dark,
        name="elbow_right_cheek",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.060, 0.105, 0.185)),
        mass=0.95,
        origin=Origin(xyz=(0.000, lower_span_y * 0.45, lower_span_z * 0.45)),
    )

    upper_arm = model.part("upper_arm")
    upper_span_y = 0.098
    upper_span_z = 0.146
    upper_start_y = 0.010
    upper_start_z = 0.012
    upper_bar_dy = upper_span_y - upper_start_y
    upper_bar_dz = upper_span_z - upper_start_z
    upper_angle = math.atan2(upper_bar_dz, upper_bar_dy)
    upper_length = math.hypot(upper_bar_dy, upper_bar_dz)
    upper_mid = ((upper_span_y + upper_start_y) * 0.5, (upper_span_z + upper_start_z) * 0.5)
    upper_arm.visual(
        Cylinder(radius=0.013, length=0.036),
        origin=Origin(rpy=(0.000, math.pi / 2.0, 0.000)),
        material=hinge_dark,
        name="rear_hub",
    )
    upper_arm.visual(
        Box((0.010, 0.016, 0.016)),
        origin=Origin(xyz=(-0.011, upper_start_y * 0.5, upper_start_z * 0.5)),
        material=hinge_dark,
        name="left_root_block",
    )
    upper_arm.visual(
        Box((0.010, 0.016, 0.016)),
        origin=Origin(xyz=(0.011, upper_start_y * 0.5, upper_start_z * 0.5)),
        material=hinge_dark,
        name="right_root_block",
    )
    upper_arm.visual(
        Box((0.004, upper_length, 0.008)),
        origin=Origin(xyz=(-0.016, upper_mid[0], upper_mid[1]), rpy=(upper_angle, 0.000, 0.000)),
        material=arm_paint,
        name="left_strut",
    )
    upper_arm.visual(
        Box((0.004, upper_length, 0.008)),
        origin=Origin(xyz=(0.016, upper_mid[0], upper_mid[1]), rpy=(upper_angle, 0.000, 0.000)),
        material=arm_paint,
        name="right_strut",
    )
    upper_arm.visual(
        Box((0.006, 0.016, 0.028)),
        origin=Origin(xyz=(-0.021, upper_span_y, upper_span_z)),
        material=hinge_dark,
        name="shade_left_cheek",
    )
    upper_arm.visual(
        Box((0.006, 0.016, 0.028)),
        origin=Origin(xyz=(0.021, upper_span_y, upper_span_z)),
        material=hinge_dark,
        name="shade_right_cheek",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.054, 0.110, 0.165)),
        mass=0.78,
        origin=Origin(xyz=(0.000, upper_span_y * 0.45, upper_span_z * 0.45)),
    )

    shade = model.part("shade")
    reflector_geom = LatheGeometry.from_shell_profiles(
        [
            (0.018, 0.000),
            (0.025, 0.014),
            (0.036, 0.040),
            (0.051, 0.082),
            (0.060, 0.106),
        ],
        [
            (0.000, 0.010),
            (0.012, 0.014),
            (0.023, 0.038),
            (0.041, 0.080),
            (0.052, 0.104),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    reflector_mesh = mesh_from_geometry(reflector_geom, ASSETS.mesh_path("desk_task_lamp_reflector.obj"))
    shade_roll = -math.pi / 2.0 - 0.62
    shade.visual(
        Cylinder(radius=0.011, length=0.036),
        origin=Origin(rpy=(0.000, math.pi / 2.0, 0.000)),
        material=hinge_dark,
        name="tilt_hub",
    )
    shade.visual(
        Box((0.032, 0.018, 0.018)),
        origin=Origin(xyz=(0.000, 0.008, -0.008)),
        material=hinge_dark,
        name="mount_block",
    )
    shade.visual(
        Cylinder(radius=0.009, length=0.030),
        origin=Origin(xyz=(0.000, 0.024, -0.016), rpy=(-math.pi / 2.0, 0.000, 0.000)),
        material=hinge_dark,
        name="neck_tube",
    )
    shade.visual(
        reflector_mesh,
        origin=Origin(xyz=(0.000, 0.034, -0.018), rpy=(shade_roll, 0.000, 0.000)),
        material=reflector_white,
        name="reflector_shell",
    )
    shade.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.000, 0.056, -0.032), rpy=(-math.pi / 2.0, 0.000, 0.000)),
        material=socket_dark,
        name="socket",
    )
    shade.visual(
        Sphere(radius=0.015),
        origin=Origin(xyz=(0.000, 0.068, -0.040)),
        material=bulb_glass,
        name="bulb",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.130, 0.130, 0.090)),
        mass=0.55,
        origin=Origin(xyz=(0.000, 0.056, -0.032)),
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.000, -0.048, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-0.55, upper=0.58),
    )
    model.articulation(
        "lower_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.000, lower_span_y, lower_span_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.2, lower=-1.10, upper=1.00),
    )
    model.articulation(
        "upper_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(0.000, upper_span_y, upper_span_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.8, lower=-0.95, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")
    base_to_lower = object_model.get_articulation("base_to_lower_arm")
    lower_to_upper = object_model.get_articulation("lower_to_upper_arm")
    upper_to_shade = object_model.get_articulation("upper_to_shade")
    top_plate = base.get_visual("top_plate")
    hinge_left_cheek = base.get_visual("hinge_left_cheek")
    lower_rear_hub = lower_arm.get_visual("rear_hub")
    lower_elbow_left_cheek = lower_arm.get_visual("elbow_left_cheek")
    upper_rear_hub = upper_arm.get_visual("rear_hub")
    upper_shade_left_cheek = upper_arm.get_visual("shade_left_cheek")
    shade_hub = shade.get_visual("tilt_hub")
    reflector_shell = shade.get_visual("reflector_shell")
    bulb = shade.get_visual("bulb")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=96)

    for articulation in (base_to_lower, lower_to_upper, upper_to_shade):
        limits = articulation.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({articulation: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_lower_no_floating")
        with ctx.pose({articulation: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_upper_no_floating")

    ctx.expect_contact(
        lower_arm,
        base,
        elem_a=lower_rear_hub,
        elem_b=hinge_left_cheek,
        name="lower_hub_contacts_base_cheek",
    )
    ctx.expect_gap(
        lower_arm,
        base,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem=lower_rear_hub,
        negative_elem=top_plate,
        name="lower_hinge_seated_on_base",
    )
    ctx.expect_overlap(
        lower_arm,
        base,
        axes="yz",
        elem_a=lower_rear_hub,
        elem_b=hinge_left_cheek,
        min_overlap=0.018,
        name="lower_hinge_nested_between_base_cheeks",
    )
    ctx.expect_contact(
        upper_arm,
        lower_arm,
        elem_a=upper_rear_hub,
        elem_b=lower_elbow_left_cheek,
        name="upper_hub_contacts_lower_elbow_cheek",
    )
    ctx.expect_overlap(
        upper_arm,
        lower_arm,
        axes="yz",
        elem_a=upper_rear_hub,
        elem_b=lower_elbow_left_cheek,
        min_overlap=0.018,
        name="elbow_hinge_nested_in_lower_arm_cheeks",
    )
    ctx.expect_contact(
        shade,
        upper_arm,
        elem_a=shade_hub,
        elem_b=upper_shade_left_cheek,
        name="shade_hub_contacts_upper_arm_cheek",
    )
    ctx.expect_overlap(
        shade,
        upper_arm,
        axes="yz",
        elem_a=shade_hub,
        elem_b=upper_shade_left_cheek,
        min_overlap=0.016,
        name="shade_hinge_nested_in_upper_arm_cheeks",
    )
    ctx.expect_within(
        shade,
        shade,
        axes="xy",
        inner_elem=bulb,
        outer_elem=reflector_shell,
        name="bulb_sits_within_reflector_mouth",
    )
    ctx.expect_gap(
        shade,
        base,
        axis="z",
        min_gap=0.080,
        positive_elem=reflector_shell,
        negative_elem=top_plate,
        name="shade_clears_weighted_base_in_rest_pose",
    )
    with ctx.pose({base_to_lower: -0.32, lower_to_upper: 0.78, upper_to_shade: -0.38}):
        ctx.expect_gap(
            shade,
            base,
            axis="z",
            min_gap=0.035,
            positive_elem=reflector_shell,
            negative_elem=top_plate,
            name="shade_stays_above_base_when_reaching_forward",
        )
        ctx.expect_overlap(
            shade,
            upper_arm,
            axes="yz",
            elem_a=shade_hub,
            elem_b=upper_shade_left_cheek,
            min_overlap=0.008,
            name="shade_remains_attached_in_forward_pose",
        )
    with ctx.pose({base_to_lower: 0.42, lower_to_upper: -0.55, upper_to_shade: 0.30}):
        ctx.expect_gap(
            shade,
            base,
            axis="z",
            min_gap=0.140,
            positive_elem=reflector_shell,
            negative_elem=top_plate,
            name="shade_lifts_high_when_arm_raised",
        )
        ctx.expect_overlap(
            upper_arm,
            lower_arm,
            axes="yz",
            elem_a=upper_rear_hub,
            elem_b=lower_elbow_left_cheek,
            min_overlap=0.008,
            name="elbow_mount_holds_in_raised_pose",
        )
    with ctx.pose({upper_to_shade: -0.70}):
        ctx.expect_gap(
            upper_arm,
            shade,
            axis="z",
            min_gap=0.040,
            positive_elem=upper_shade_left_cheek,
            negative_elem=bulb,
            name="shade_can_tilt_down_to_hang_below_its_hinge",
        )
    with ctx.pose({upper_to_shade: 0.40}):
        ctx.expect_gap(
            shade,
            upper_arm,
            axis="y",
            min_gap=0.040,
            positive_elem=bulb,
            negative_elem=upper_shade_left_cheek,
            name="shade_can_tilt_forward_to_throw_light_ahead",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
