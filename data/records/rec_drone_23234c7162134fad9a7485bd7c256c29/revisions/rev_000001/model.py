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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


ARM_LENGTH = 0.118
ARM_INSERTION = 0.012
ARM_ROOT_Z = 0.007
PROP_HUB_HEIGHT = 0.0045
PROP_JOINT_Z = 0.017
CAMERA_BASE_TILT = math.radians(25.0)


def _blade_section(
    span_x: float,
    chord: float,
    thickness: float,
    twist: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        chord,
        thickness,
        radius=min(thickness * 0.45, chord * 0.14),
        corner_segments=6,
    )
    c = math.cos(twist)
    s = math.sin(twist)
    points: list[tuple[float, float, float]] = []
    for y, z in profile:
        yy = c * y - s * z
        zz = s * y + c * z
        points.append((span_x, yy, zz))
    return points


def _make_prop_blade_mesh():
    return mesh_from_geometry(
        section_loft(
            [
                _blade_section(0.006, 0.017, 0.0030, math.radians(16.0)),
                _blade_section(0.028, 0.013, 0.0024, math.radians(10.0)),
                _blade_section(0.048, 0.008, 0.0018, math.radians(4.0)),
                _blade_section(0.061, 0.0045, 0.0011, math.radians(1.0)),
            ]
        ),
        "racing_quad_prop_blade",
    )


def _add_arm(
    model: ArticulatedObject,
    main_frame,
    *,
    part_name: str,
    joint_name: str,
    joint_xyz: tuple[float, float, float],
    yaw: float,
    carbon,
    metal,
):
    arm = model.part(part_name)
    arm.visual(
        Box((ARM_LENGTH + ARM_INSERTION, 0.014, 0.005)),
        origin=Origin(xyz=((ARM_LENGTH - ARM_INSERTION) * 0.5, 0.0, 0.0)),
        material=carbon,
        name="arm_beam",
    )
    arm.visual(
        Box((0.020, 0.018, 0.003)),
        origin=Origin(xyz=(0.014, 0.0, -0.0035)),
        material=carbon,
        name="esc_pad",
    )
    arm.visual(
        Cylinder(radius=0.015, length=0.008),
        origin=Origin(xyz=(ARM_LENGTH, 0.0, 0.004)),
        material=metal,
        name="motor_mount",
    )
    arm.visual(
        Cylinder(radius=0.010, length=0.009),
        origin=Origin(xyz=(ARM_LENGTH, 0.0, 0.0125)),
        material=metal,
        name="motor_bell",
    )
    arm.inertial = Inertial.from_geometry(
        Box((ARM_LENGTH + ARM_INSERTION, 0.018, 0.022)),
        mass=0.06,
        origin=Origin(xyz=(ARM_LENGTH * 0.5, 0.0, 0.006)),
    )
    model.articulation(
        joint_name,
        ArticulationType.FIXED,
        parent=main_frame,
        child=arm,
        origin=Origin(xyz=joint_xyz, rpy=(0.0, 0.0, yaw)),
    )
    return arm


def _add_propeller(
    model: ArticulatedObject,
    arm,
    *,
    part_name: str,
    joint_name: str,
    axis_sign: float,
    blade_mesh,
    prop_material,
    nut_material,
):
    prop = model.part(part_name)
    prop.visual(
        Cylinder(radius=0.0105, length=PROP_HUB_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, PROP_HUB_HEIGHT * 0.5)),
        material=prop_material,
        name="hub",
    )
    prop.visual(
        blade_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0052)),
        material=prop_material,
        name="blade_a",
    )
    prop.visual(
        blade_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0052), rpy=(0.0, 0.0, math.pi)),
        material=prop_material,
        name="blade_b",
    )
    prop.visual(
        Cylinder(radius=0.0042, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=nut_material,
        name="lock_nut",
    )
    prop.inertial = Inertial.from_geometry(
        Cylinder(radius=0.064, length=0.012),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )
    model.articulation(
        joint_name,
        ArticulationType.CONTINUOUS,
        parent=arm,
        child=prop,
        origin=Origin(xyz=(ARM_LENGTH, 0.0, PROP_JOINT_Z)),
        axis=(0.0, 0.0, axis_sign),
        motion_limits=MotionLimits(effort=0.4, velocity=180.0),
    )
    return prop


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="racing_h_frame_quadrotor")

    carbon = model.material("carbon", rgba=(0.08, 0.08, 0.09, 1.0))
    carbon_gloss = model.material("carbon_gloss", rgba=(0.12, 0.12, 0.13, 1.0))
    metal = model.material("metal", rgba=(0.54, 0.56, 0.60, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.73, 0.76, 1.0))
    accent = model.material("accent", rgba=(0.75, 0.15, 0.12, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.18, 0.28, 0.34, 0.45))

    blade_mesh = _make_prop_blade_mesh()

    main_frame = model.part("main_frame")
    main_frame.visual(
        Box((0.108, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, 0.031, 0.0015)),
        material=carbon,
        name="lower_left_rail",
    )
    main_frame.visual(
        Box((0.108, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, -0.031, 0.0015)),
        material=carbon,
        name="lower_right_rail",
    )
    main_frame.visual(
        Box((0.032, 0.092, 0.003)),
        origin=Origin(xyz=(0.038, 0.0, 0.0015)),
        material=carbon,
        name="front_bridge",
    )
    main_frame.visual(
        Box((0.032, 0.092, 0.003)),
        origin=Origin(xyz=(-0.038, 0.0, 0.0015)),
        material=carbon,
        name="rear_bridge",
    )
    main_frame.visual(
        Box((0.050, 0.050, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=carbon,
        name="center_tray",
    )
    main_frame.visual(
        Box((0.108, 0.058, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0285)),
        material=carbon_gloss,
        name="top_plate",
    )
    for name, xyz in (
        ("standoff_fl", (0.031, 0.020, 0.015)),
        ("standoff_fr", (0.031, -0.020, 0.015)),
        ("standoff_rl", (-0.031, 0.020, 0.015)),
        ("standoff_rr", (-0.031, -0.020, 0.015)),
    ):
        main_frame.visual(
            Cylinder(radius=0.004, length=0.027),
            origin=Origin(xyz=xyz),
            material=aluminum,
            name=name,
        )
    for name, xyz in (
        ("clamp_fl", (0.029, 0.029, ARM_ROOT_Z)),
        ("clamp_fr", (0.029, -0.029, ARM_ROOT_Z)),
        ("clamp_rl", (-0.029, 0.029, ARM_ROOT_Z)),
        ("clamp_rr", (-0.029, -0.029, ARM_ROOT_Z)),
    ):
        main_frame.visual(
            Box((0.020, 0.020, 0.010)),
            origin=Origin(xyz=xyz),
            material=carbon_gloss,
            name=name,
        )
    main_frame.visual(
        Box((0.034, 0.034, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=accent,
        name="flight_stack",
    )
    main_frame.visual(
        Box((0.026, 0.030, 0.028)),
        origin=Origin(xyz=(0.062, 0.0, 0.015)),
        material=carbon_gloss,
        name="camera_bracket",
    )
    main_frame.visual(
        Box((0.030, 0.018, 0.006)),
        origin=Origin(xyz=(-0.030, 0.0, 0.031)),
        material=metal,
        name="vtx_cap",
    )
    main_frame.inertial = Inertial.from_geometry(
        Box((0.170, 0.135, 0.040)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    front_left_arm = _add_arm(
        model,
        main_frame,
        part_name="front_left_arm",
        joint_name="frame_to_front_left_arm",
        joint_xyz=(0.034, 0.034, ARM_ROOT_Z),
        yaw=math.radians(45.0),
        carbon=carbon,
        metal=metal,
    )
    front_right_arm = _add_arm(
        model,
        main_frame,
        part_name="front_right_arm",
        joint_name="frame_to_front_right_arm",
        joint_xyz=(0.034, -0.034, ARM_ROOT_Z),
        yaw=math.radians(-45.0),
        carbon=carbon,
        metal=metal,
    )
    rear_left_arm = _add_arm(
        model,
        main_frame,
        part_name="rear_left_arm",
        joint_name="frame_to_rear_left_arm",
        joint_xyz=(-0.034, 0.034, ARM_ROOT_Z),
        yaw=math.radians(135.0),
        carbon=carbon,
        metal=metal,
    )
    rear_right_arm = _add_arm(
        model,
        main_frame,
        part_name="rear_right_arm",
        joint_name="frame_to_rear_right_arm",
        joint_xyz=(-0.034, -0.034, ARM_ROOT_Z),
        yaw=math.radians(-135.0),
        carbon=carbon,
        metal=metal,
    )

    _add_propeller(
        model,
        front_left_arm,
        part_name="front_left_propeller",
        joint_name="front_left_prop_spin",
        axis_sign=1.0,
        blade_mesh=blade_mesh,
        prop_material=carbon_gloss,
        nut_material=aluminum,
    )
    _add_propeller(
        model,
        front_right_arm,
        part_name="front_right_propeller",
        joint_name="front_right_prop_spin",
        axis_sign=-1.0,
        blade_mesh=blade_mesh,
        prop_material=carbon_gloss,
        nut_material=aluminum,
    )
    _add_propeller(
        model,
        rear_left_arm,
        part_name="rear_left_propeller",
        joint_name="rear_left_prop_spin",
        axis_sign=-1.0,
        blade_mesh=blade_mesh,
        prop_material=carbon_gloss,
        nut_material=aluminum,
    )
    _add_propeller(
        model,
        rear_right_arm,
        part_name="rear_right_propeller",
        joint_name="rear_right_prop_spin",
        axis_sign=1.0,
        blade_mesh=blade_mesh,
        prop_material=carbon_gloss,
        nut_material=aluminum,
    )

    camera_plate = model.part("camera_plate")
    camera_plate.visual(
        Box((0.028, 0.026, 0.010)),
        origin=Origin(xyz=(0.002, 0.0, -0.004)),
        material=metal,
        name="hinge_tab",
    )
    camera_plate.visual(
        Box((0.022, 0.028, 0.006)),
        origin=Origin(xyz=(0.012, 0.0, -0.008), rpy=(0.0, CAMERA_BASE_TILT * 0.45, 0.0)),
        material=metal,
        name="mount_spine",
    )
    camera_plate.visual(
        Box((0.0035, 0.044, 0.034)),
        origin=Origin(xyz=(0.016, 0.0, -0.018), rpy=(0.0, CAMERA_BASE_TILT, 0.0)),
        material=carbon,
        name="plate_panel",
    )
    camera_plate.visual(
        Box((0.016, 0.024, 0.016)),
        origin=Origin(xyz=(0.020, 0.0, -0.018), rpy=(0.0, CAMERA_BASE_TILT, 0.0)),
        material=carbon_gloss,
        name="camera_body",
    )
    camera_plate.visual(
        Cylinder(radius=0.007, length=0.008),
        origin=Origin(
            xyz=(0.029, 0.0, -0.018),
            rpy=(0.0, math.pi * 0.5 + CAMERA_BASE_TILT, 0.0),
        ),
        material=lens_glass,
        name="camera_lens",
    )
    camera_plate.inertial = Inertial.from_geometry(
        Box((0.040, 0.050, 0.040)),
        mass=0.03,
        origin=Origin(xyz=(0.014, 0.0, -0.014)),
    )
    model.articulation(
        "camera_tilt",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=camera_plate,
        origin=Origin(xyz=(0.074, 0.0, 0.027)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
            lower=math.radians(-12.0),
            upper=math.radians(30.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    main_frame = object_model.get_part("main_frame")
    camera_plate = object_model.get_part("camera_plate")
    front_left_arm = object_model.get_part("front_left_arm")
    front_right_arm = object_model.get_part("front_right_arm")
    rear_left_arm = object_model.get_part("rear_left_arm")
    rear_right_arm = object_model.get_part("rear_right_arm")
    front_left_propeller = object_model.get_part("front_left_propeller")
    front_right_propeller = object_model.get_part("front_right_propeller")
    rear_left_propeller = object_model.get_part("rear_left_propeller")
    rear_right_propeller = object_model.get_part("rear_right_propeller")

    camera_tilt = object_model.get_articulation("camera_tilt")
    prop_joints = [
        object_model.get_articulation("front_left_prop_spin"),
        object_model.get_articulation("front_right_prop_spin"),
        object_model.get_articulation("rear_left_prop_spin"),
        object_model.get_articulation("rear_right_prop_spin"),
    ]

    clamp_fl = main_frame.get_visual("clamp_fl")
    clamp_fr = main_frame.get_visual("clamp_fr")
    clamp_rl = main_frame.get_visual("clamp_rl")
    clamp_rr = main_frame.get_visual("clamp_rr")
    camera_bracket = main_frame.get_visual("camera_bracket")
    hinge_tab = camera_plate.get_visual("hinge_tab")
    plate_panel = camera_plate.get_visual("plate_panel")
    lens = camera_plate.get_visual("camera_lens")

    fl_beam = front_left_arm.get_visual("arm_beam")
    fr_beam = front_right_arm.get_visual("arm_beam")
    rl_beam = rear_left_arm.get_visual("arm_beam")
    rr_beam = rear_right_arm.get_visual("arm_beam")

    fl_motor = front_left_arm.get_visual("motor_bell")
    fr_motor = front_right_arm.get_visual("motor_bell")
    rl_motor = rear_left_arm.get_visual("motor_bell")
    rr_motor = rear_right_arm.get_visual("motor_bell")

    fl_hub = front_left_propeller.get_visual("hub")
    fr_hub = front_right_propeller.get_visual("hub")
    rl_hub = rear_left_propeller.get_visual("hub")
    rr_hub = rear_right_propeller.get_visual("hub")

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.allow_overlap(
        main_frame,
        front_left_arm,
        elem_a=clamp_fl,
        elem_b=fl_beam,
        reason="Front-left arm root is simplified as a solid beam inserted into a clamp slot.",
    )
    ctx.allow_overlap(
        main_frame,
        front_right_arm,
        elem_a=clamp_fr,
        elem_b=fr_beam,
        reason="Front-right arm root is simplified as a solid beam inserted into a clamp slot.",
    )
    ctx.allow_overlap(
        main_frame,
        rear_left_arm,
        elem_a=clamp_rl,
        elem_b=rl_beam,
        reason="Rear-left arm root is simplified as a solid beam inserted into a clamp slot.",
    )
    ctx.allow_overlap(
        main_frame,
        rear_right_arm,
        elem_a=clamp_rr,
        elem_b=rr_beam,
        reason="Rear-right arm root is simplified as a solid beam inserted into a clamp slot.",
    )
    ctx.allow_overlap(
        main_frame,
        camera_plate,
        elem_a=camera_bracket,
        elem_b=hinge_tab,
        reason="The camera tilt hinge is represented by a compact tab passing through the nose bracket pivot zone.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    for arm in (front_left_arm, front_right_arm, rear_left_arm, rear_right_arm):
        ctx.expect_overlap(arm, main_frame, axes="xy", min_overlap=0.008)

    for prop, arm, hub, motor in (
        (front_left_propeller, front_left_arm, fl_hub, fl_motor),
        (front_right_propeller, front_right_arm, fr_hub, fr_motor),
        (rear_left_propeller, rear_left_arm, rl_hub, rl_motor),
        (rear_right_propeller, rear_right_arm, rr_hub, rr_motor),
    ):
        ctx.expect_gap(
            prop,
            arm,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=hub,
            negative_elem=motor,
            name=f"{prop.name}_hub_seats_on_motor",
        )
        ctx.expect_overlap(
            prop,
            arm,
            axes="xy",
            min_overlap=0.014,
            elem_a=hub,
            elem_b=motor,
            name=f"{prop.name}_hub_centered_on_motor",
        )

    ctx.expect_origin_gap(
        front_left_propeller,
        front_right_propeller,
        axis="y",
        min_gap=0.20,
        max_gap=0.25,
        name="front_motor_span",
    )
    ctx.expect_origin_gap(
        front_left_propeller,
        rear_left_propeller,
        axis="x",
        min_gap=0.20,
        max_gap=0.25,
        name="left_motor_span",
    )
    ctx.expect_origin_gap(
        camera_plate,
        main_frame,
        axis="x",
        min_gap=0.06,
        max_gap=0.09,
        name="camera_hinge_ahead_of_center",
    )
    ctx.expect_origin_distance(
        camera_plate,
        main_frame,
        axes="y",
        max_dist=0.0005,
        name="camera_on_centerline",
    )
    ctx.expect_overlap(
        camera_plate,
        main_frame,
        axes="y",
        min_overlap=0.025,
        elem_a=plate_panel,
        elem_b=camera_bracket,
        name="camera_plate_centered_on_nose",
    )

    def _center_from_aabb(aabb):
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    limits = camera_tilt.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({camera_tilt: limits.lower}):
            lower_lens_aabb = ctx.part_element_world_aabb(camera_plate, elem=lens)
            ctx.fail_if_parts_overlap_in_current_pose(name="camera_tilt_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="camera_tilt_lower_no_floating")
        with ctx.pose({camera_tilt: limits.upper}):
            upper_lens_aabb = ctx.part_element_world_aabb(camera_plate, elem=lens)
            ctx.fail_if_parts_overlap_in_current_pose(name="camera_tilt_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="camera_tilt_upper_no_floating")

        if lower_lens_aabb is not None and upper_lens_aabb is not None:
            lower_center = _center_from_aabb(lower_lens_aabb)
            upper_center = _center_from_aabb(upper_lens_aabb)
            ctx.check(
                "camera_lens_rises_when_tilted",
                upper_center[2] > lower_center[2] + 0.004,
                details=f"lower={lower_center}, upper={upper_center}",
            )
            ctx.check(
                "camera_lens_moves_forward_when_tilted",
                upper_center[0] > lower_center[0] + 0.003,
                details=f"lower={lower_center}, upper={upper_center}",
            )

    axis_expectations = {
        "front_left_prop_spin": 1.0,
        "front_right_prop_spin": -1.0,
        "rear_left_prop_spin": -1.0,
        "rear_right_prop_spin": 1.0,
    }
    for joint in prop_joints:
        axis = joint.axis
        expected_sign = axis_expectations[joint.name]
        ctx.check(
            f"{joint.name}_vertical_axis",
            abs(axis[0]) < 1e-9 and abs(axis[1]) < 1e-9 and abs(axis[2] - expected_sign) < 1e-9,
            details=f"axis={axis}, expected z sign={expected_sign}",
        )
        with ctx.pose({joint: math.pi * 0.5}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_quarter_turn_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_quarter_turn_no_floating")

    ctx.check(
        "camera_tilt_axis_is_lateral",
        camera_tilt.axis == (0.0, -1.0, 0.0),
        details=f"axis={camera_tilt.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
