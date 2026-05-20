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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_wiper_module")

    painted_steel = model.material("painted_steel", rgba=(0.29, 0.31, 0.33, 1.0))
    dark_cast = model.material("dark_cast", rgba=(0.18, 0.19, 0.20, 1.0))
    zinc = model.material("zinc", rgba=(0.73, 0.74, 0.76, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    blade_rubber = model.material("blade_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    service_orange = model.material("service_orange", rgba=(0.84, 0.38, 0.10, 1.0))

    frame = model.part("support_frame")
    frame.visual(
        Box((0.56, 0.36, 0.03)),
        origin=Origin(xyz=(0.0, -0.04, 0.015)),
        material=painted_steel,
        name="tray_base",
    )
    frame.visual(
        Box((0.18, 0.22, 0.03)),
        origin=Origin(xyz=(-0.44, 0.0, 0.015)),
        material=painted_steel,
        name="left_outboard_shelf",
    )
    frame.visual(
        Box((0.18, 0.22, 0.03)),
        origin=Origin(xyz=(0.44, 0.0, 0.015)),
        material=painted_steel,
        name="right_outboard_shelf",
    )
    frame.visual(
        Box((1.06, 0.03, 0.06)),
        origin=Origin(xyz=(0.0, 0.095, 0.045)),
        material=painted_steel,
        name="front_lip",
    )
    frame.visual(
        Box((1.06, 0.04, 0.08)),
        origin=Origin(xyz=(0.0, -0.09, 0.055)),
        material=painted_steel,
        name="rear_lip",
    )
    frame.visual(
        Box((0.06, 0.03, 0.162)),
        origin=Origin(xyz=(-0.34, 0.040, 0.081)),
        material=painted_steel,
        name="left_tower_base",
    )
    frame.visual(
        Box((0.06, 0.03, 0.162)),
        origin=Origin(xyz=(0.34, 0.040, 0.081)),
        material=painted_steel,
        name="right_tower_base",
    )
    frame.visual(
        Box((0.10, 0.02, 0.08)),
        origin=Origin(xyz=(-0.34, 0.012, 0.065)),
        material=painted_steel,
        name="left_tower_front_web",
    )
    frame.visual(
        Box((0.10, 0.02, 0.08)),
        origin=Origin(xyz=(0.34, 0.012, 0.065)),
        material=painted_steel,
        name="right_tower_front_web",
    )
    frame.visual(
        Box((0.56, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=painted_steel,
        name="crossbrace",
    )
    frame.visual(
        Box((0.10, 0.04, 0.03)),
        origin=Origin(xyz=(-0.07, -0.152, 0.015)),
        material=painted_steel,
        name="left_motor_pad",
    )
    frame.visual(
        Box((0.10, 0.04, 0.03)),
        origin=Origin(xyz=(0.07, -0.152, 0.015)),
        material=painted_steel,
        name="right_motor_pad",
    )
    frame.visual(
        Box((0.16, 0.06, 0.04)),
        origin=Origin(xyz=(-0.24, -0.240, 0.045)),
        material=painted_steel,
        name="left_service_rail",
    )
    frame.visual(
        Box((0.16, 0.06, 0.04)),
        origin=Origin(xyz=(0.24, -0.240, 0.045)),
        material=painted_steel,
        name="right_service_rail",
    )
    frame.visual(
        Box((0.060, 0.060, 0.018)),
        origin=Origin(xyz=(-0.34, 0.0, 0.109)),
        material=dark_cast,
        name="left_housing",
    )
    frame.visual(
        Box((0.060, 0.060, 0.018)),
        origin=Origin(xyz=(0.34, 0.0, 0.109)),
        material=dark_cast,
        name="right_housing",
    )
    frame.visual(
        Box((0.010, 0.048, 0.020)),
        origin=Origin(xyz=(-0.375, 0.0, 0.128)),
        material=zinc,
        name="left_housing_cap",
    )
    frame.visual(
        Box((0.010, 0.048, 0.020)),
        origin=Origin(xyz=(0.375, 0.0, 0.128)),
        material=zinc,
        name="right_housing_cap",
    )
    frame.visual(
        Box((0.12, 0.08, 0.04)),
        origin=Origin(xyz=(0.0, 0.045, 0.05)),
        material=painted_steel,
        name="service_bridge",
    )
    frame.inertial = Inertial.from_geometry(
        Box((1.06, 0.22, 0.19)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    motor_housing = model.part("motor_housing")
    motor_housing.visual(
        Box((0.22, 0.14, 0.084)),
        origin=Origin(xyz=(0.0, -0.018, 0.018)),
        material=dark_cast,
        name="gearbox_body",
    )
    motor_housing.visual(
        Cylinder(radius=0.046, length=0.24),
        origin=Origin(xyz=(0.0, -0.11, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_cast,
        name="motor_can",
    )
    motor_housing.visual(
        Cylinder(radius=0.029, length=0.04),
        origin=Origin(xyz=(0.0, -0.17, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_cast,
        name="motor_endcap",
    )
    motor_housing.visual(
        Box((0.18, 0.10, 0.018)),
        origin=Origin(xyz=(0.0, -0.010, 0.062)),
        material=service_orange,
        name="service_cover",
    )
    motor_housing.visual(
        Cylinder(radius=0.016, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=zinc,
        name="output_stub",
    )
    motor_housing.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=dark_cast,
        name="output_boss",
    )
    motor_housing.visual(
        Box((0.07, 0.03, 0.024)),
        origin=Origin(xyz=(-0.07, 0.010, -0.038)),
        material=painted_steel,
        name="mount_foot_left",
    )
    motor_housing.visual(
        Box((0.07, 0.03, 0.024)),
        origin=Origin(xyz=(0.07, 0.010, -0.038)),
        material=painted_steel,
        name="mount_foot_right",
    )
    motor_housing.visual(
        Box((0.030, 0.032, 0.050)),
        origin=Origin(xyz=(-0.070, 0.006, -0.012)),
        material=painted_steel,
        name="mount_leg_left",
    )
    motor_housing.visual(
        Box((0.030, 0.032, 0.050)),
        origin=Origin(xyz=(0.070, 0.006, -0.012)),
        material=painted_steel,
        name="mount_leg_right",
    )
    motor_housing.inertial = Inertial.from_geometry(
        Box((0.26, 0.20, 0.14)),
        mass=6.5,
        origin=Origin(xyz=(0.0, -0.05, 0.02)),
    )

    motor_crank = model.part("motor_crank")
    motor_crank.visual(
        Cylinder(radius=0.026, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=zinc,
        name="hub",
    )
    motor_crank.visual(
        Box((0.076, 0.026, 0.012)),
        origin=Origin(xyz=(0.062, 0.0, 0.000)),
        material=painted_steel,
        name="crank_arm",
    )
    motor_crank.visual(
        Cylinder(radius=0.010, length=0.090),
        origin=Origin(xyz=(0.090, 0.0, 0.045)),
        material=zinc,
        name="crank_pin",
    )
    motor_crank.visual(
        Box((0.024, 0.022, 0.012)),
        origin=Origin(xyz=(-0.030, 0.0, 0.0)),
        material=painted_steel,
        name="counterweight",
    )
    motor_crank.inertial = Inertial.from_geometry(
        Box((0.14, 0.06, 0.10)),
        mass=0.8,
        origin=Origin(xyz=(0.03, 0.0, 0.035)),
    )

    drive_link = model.part("drive_link")
    drive_link.visual(
        Box((0.116, 0.018, 0.012)),
        origin=Origin(xyz=(0.074, 0.0, 0.020)),
        material=painted_steel,
        name="link_bar",
    )
    drive_link.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=zinc,
        name="proximal_eye",
    )
    drive_link.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.148, 0.0, 0.024)),
        material=zinc,
        name="distal_eye",
    )
    drive_link.visual(
        Box((0.028, 0.018, 0.010)),
        origin=Origin(xyz=(0.067, 0.0, 0.026)),
        material=painted_steel,
        name="service_stiffener",
    )
    drive_link.inertial = Inertial.from_geometry(
        Box((0.17, 0.04, 0.04)),
        mass=0.45,
        origin=Origin(xyz=(0.074, 0.0, 0.020)),
    )

    cross_link = model.part("cross_link")
    cross_link.visual(
        Box((0.438, 0.018, 0.012)),
        origin=Origin(xyz=(0.235, 0.0, -0.022)),
        material=painted_steel,
        name="link_bar",
    )
    cross_link.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=zinc,
        name="proximal_eye",
    )
    cross_link.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.470, 0.0, -0.022)),
        material=zinc,
        name="distal_eye",
    )
    cross_link.visual(
        Box((0.040, 0.018, 0.010)),
        origin=Origin(xyz=(0.217, 0.0, -0.028)),
        material=painted_steel,
        name="center_rib",
    )
    cross_link.inertial = Inertial.from_geometry(
        Box((0.49, 0.04, 0.04)),
        mass=0.9,
        origin=Origin(xyz=(0.235, 0.0, -0.020)),
    )

    def build_wiper_part(name: str, sign: float, blade_stack: float, arm_angle: float) -> None:
        wiper = model.part(name)
        arm_dx = math.cos(arm_angle)
        arm_dy = math.sin(arm_angle)
        link_angle = math.atan2(-0.090, sign * 0.105)
        link_dx = math.cos(link_angle)
        link_dy = math.sin(link_angle)
        wiper.visual(
            Cylinder(radius=0.011, length=0.14),
            origin=Origin(xyz=(0.0, 0.0, 0.100)),
            material=zinc,
            name="spindle_shaft",
        )
        wiper.visual(
            Cylinder(radius=0.020, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, 0.036)),
            material=zinc,
            name="service_washer",
        )
        wiper.visual(
            Cylinder(radius=0.021, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.024)),
            material=painted_steel,
            name="drive_collar",
        )
        wiper.visual(
            Box((0.144, 0.024, 0.014)),
            origin=Origin(
                xyz=(link_dx * 0.0525, link_dy * 0.0525, 0.012),
                rpy=(0.0, 0.0, link_angle),
            ),
            material=painted_steel,
            name="drive_arm",
        )
        wiper.visual(
            Box((0.028, 0.028, 0.016)),
            origin=Origin(xyz=(sign * 0.105, -0.090, 0.014)),
            material=painted_steel,
            name="pin_boss",
        )
        wiper.visual(
            Cylinder(radius=0.010, length=0.080),
            origin=Origin(xyz=(sign * 0.105, -0.090, 0.024)),
            material=zinc,
            name="drive_pin",
        )
        wiper.visual(
            Box((0.040, 0.018, 0.040)),
            origin=Origin(
                xyz=(arm_dx * 0.014, arm_dy * 0.014, 0.056 + blade_stack),
                rpy=(0.0, 0.0, arm_angle),
            ),
            material=painted_steel,
            name="arm_riser",
        )
        wiper.visual(
            Box((0.060, 0.036, 0.018)),
            origin=Origin(
                xyz=(arm_dx * 0.034, arm_dy * 0.034, 0.082 + blade_stack),
                rpy=(0.0, 0.0, arm_angle),
            ),
            material=painted_steel,
            name="arm_knuckle",
        )
        wiper.visual(
            Box((0.310, 0.022, 0.016)),
            origin=Origin(
                xyz=(arm_dx * 0.175, arm_dy * 0.175, 0.082 + blade_stack),
                rpy=(0.0, 0.0, arm_angle),
            ),
            material=painted_steel,
            name="arm_beam",
        )
        wiper.visual(
            Box((0.18, 0.012, 0.012)),
            origin=Origin(
                xyz=(arm_dx * 0.245, arm_dy * 0.245, 0.094 + blade_stack),
                rpy=(0.0, 0.0, arm_angle),
            ),
            material=painted_steel,
            name="spring_strut",
        )
        wiper.visual(
            Box((0.34, 0.030, 0.014)),
            origin=Origin(
                xyz=(arm_dx * 0.405, arm_dy * 0.405, 0.064 + blade_stack),
                rpy=(0.0, 0.0, arm_angle),
            ),
            material=dark_cast,
            name="blade_carrier",
        )
        wiper.visual(
            Box((0.32, 0.008, 0.010)),
            origin=Origin(
                xyz=(arm_dx * 0.405, arm_dy * 0.405, 0.052 + blade_stack),
                rpy=(0.0, 0.0, arm_angle),
            ),
            material=blade_rubber,
            name="blade_rubber",
        )
        wiper.visual(
            Box((0.052, 0.018, 0.010)),
            origin=Origin(
                xyz=(arm_dx * 0.285, arm_dy * 0.285, 0.072 + blade_stack),
                rpy=(0.0, 0.0, arm_angle),
            ),
            material=zinc,
            name="carrier_bridge",
        )
        wiper.visual(
            Box((0.040, 0.018, 0.010)),
            origin=Origin(
                xyz=(arm_dx * 0.565, arm_dy * 0.565, 0.064 + blade_stack),
                rpy=(0.0, 0.0, arm_angle),
            ),
            material=zinc,
            name="end_clip",
        )
        wiper.inertial = Inertial.from_geometry(
            Box((0.70, 0.06, 0.25)),
            mass=1.1,
            origin=Origin(xyz=(sign * 0.290, 0.0, 0.060 + blade_stack)),
        )

    build_wiper_part("left_wiper", sign=1.0, blade_stack=0.0, arm_angle=0.88)
    build_wiper_part("right_wiper", sign=-1.0, blade_stack=0.030, arm_angle=math.pi - 0.88)

    model.articulation(
        "frame_to_motor_housing",
        ArticulationType.FIXED,
        parent=frame,
        child=motor_housing,
        origin=Origin(xyz=(0.0, -0.162, 0.080)),
    )
    model.articulation(
        "motor_to_crank",
        ArticulationType.REVOLUTE,
        parent=motor_housing,
        child=motor_crank,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=-0.75,
            upper=0.75,
        ),
    )
    model.articulation(
        "crank_to_drive_link",
        ArticulationType.FIXED,
        parent=motor_crank,
        child=drive_link,
        origin=Origin(xyz=(0.090, 0.0, 0.045), rpy=(0.0, 0.0, 0.466)),
    )
    model.articulation(
        "frame_to_left_wiper",
        ArticulationType.REVOLUTE,
        parent=frame,
        child="left_wiper",
        origin=Origin(xyz=(-0.34, 0.0, 0.125)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.6,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "frame_to_right_wiper",
        ArticulationType.REVOLUTE,
        parent=frame,
        child="right_wiper",
        origin=Origin(xyz=(0.34, 0.0, 0.125)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.6,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "right_wiper_to_cross_link",
        ArticulationType.FIXED,
        parent="right_wiper",
        child=cross_link,
        origin=Origin(xyz=(-0.105, -0.090, 0.024), rpy=(0.0, 0.0, math.pi)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("support_frame")
    motor_housing = object_model.get_part("motor_housing")
    motor_crank = object_model.get_part("motor_crank")
    drive_link = object_model.get_part("drive_link")
    cross_link = object_model.get_part("cross_link")
    left_wiper = object_model.get_part("left_wiper")
    right_wiper = object_model.get_part("right_wiper")

    crank_joint = object_model.get_articulation("motor_to_crank")
    left_joint = object_model.get_articulation("frame_to_left_wiper")
    right_joint = object_model.get_articulation("frame_to_right_wiper")

    ctx.allow_overlap(
        motor_crank,
        motor_housing,
        reason="Keyed crank hub intentionally seats over the gearbox output stub.",
        elem_a="hub",
        elem_b="output_stub",
    )
    ctx.allow_overlap(
        motor_crank,
        drive_link,
        reason="Serviceable crank-pin bushing seats inside the drive-link eye.",
        elem_a="crank_pin",
        elem_b="proximal_eye",
    )
    ctx.allow_overlap(
        drive_link,
        right_wiper,
        reason="Drive link rides on the supported right spindle relay pin.",
        elem_a="distal_eye",
        elem_b="drive_pin",
    )
    ctx.allow_overlap(
        cross_link,
        right_wiper,
        reason="Cross link shares the right spindle relay pin with a stacked service joint.",
        elem_a="proximal_eye",
        elem_b="drive_pin",
    )
    ctx.allow_overlap(
        cross_link,
        left_wiper,
        reason="Cross link rides on the supported left spindle relay pin.",
        elem_a="distal_eye",
        elem_b="drive_pin",
    )

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

    ctx.expect_contact(
        motor_housing,
        frame,
        elem_a="mount_foot_left",
        elem_b="tray_base",
        name="left_motor_foot_supported_by_tray",
    )
    ctx.expect_contact(
        motor_housing,
        frame,
        elem_a="mount_foot_right",
        elem_b="tray_base",
        name="right_motor_foot_supported_by_tray",
    )
    ctx.expect_within(
        left_wiper,
        frame,
        axes="xy",
        inner_elem="spindle_shaft",
        outer_elem="left_housing",
        margin=0.0,
        name="left_spindle_runs_inside_left_housing",
    )
    ctx.expect_within(
        right_wiper,
        frame,
        axes="xy",
        inner_elem="spindle_shaft",
        outer_elem="right_housing",
        margin=0.0,
        name="right_spindle_runs_inside_right_housing",
    )
    ctx.expect_overlap(
        motor_crank,
        drive_link,
        axes="xy",
        elem_a="crank_pin",
        elem_b="proximal_eye",
        min_overlap=0.015,
        name="drive_link_registered_to_crank_pin",
    )
    ctx.expect_overlap(
        drive_link,
        right_wiper,
        axes="xy",
        elem_a="distal_eye",
        elem_b="drive_pin",
        min_overlap=0.015,
        name="drive_link_registered_to_right_spindle_pin",
    )
    ctx.expect_overlap(
        cross_link,
        right_wiper,
        axes="xy",
        elem_a="proximal_eye",
        elem_b="drive_pin",
        min_overlap=0.015,
        name="cross_link_registered_to_right_spindle_pin",
    )
    ctx.expect_overlap(
        cross_link,
        left_wiper,
        axes="xy",
        elem_a="distal_eye",
        elem_b="drive_pin",
        min_overlap=0.015,
        name="cross_link_registered_to_left_spindle_pin",
    )
    ctx.expect_gap(
        left_wiper,
        frame,
        axis="z",
        positive_elem="blade_rubber",
        negative_elem="crossbrace",
        min_gap=0.015,
        name="left_blade_clears_crossbrace",
    )
    ctx.expect_gap(
        right_wiper,
        frame,
        axis="z",
        positive_elem="blade_rubber",
        negative_elem="crossbrace",
        min_gap=0.025,
        name="right_blade_clears_crossbrace",
    )

    left_park = ctx.part_element_world_aabb(left_wiper, elem="blade_rubber")
    right_park = ctx.part_element_world_aabb(right_wiper, elem="blade_rubber")
    with ctx.pose({crank_joint: 0.40, left_joint: 0.85, right_joint: 0.85}):
        left_swept = ctx.part_element_world_aabb(left_wiper, elem="blade_rubber")
        right_swept = ctx.part_element_world_aabb(right_wiper, elem="blade_rubber")

    left_ok = (
        left_park is not None
        and left_swept is not None
        and left_swept[1][1] > left_park[1][1] + 0.10
    )
    right_ok = (
        right_park is not None
        and right_swept is not None
        and right_swept[1][1] > right_park[1][1] + 0.10
    )
    ctx.check(
        "left_blade_moves_forward_on_positive_sweep",
        left_ok,
        details="Positive left-spindle motion should drive the blade carrier forward across the windshield.",
    )
    ctx.check(
        "right_blade_moves_forward_on_positive_sweep",
        right_ok,
        details="Positive right-spindle motion should drive the blade carrier forward across the windshield.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
