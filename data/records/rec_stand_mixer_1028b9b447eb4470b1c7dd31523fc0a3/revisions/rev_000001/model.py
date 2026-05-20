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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deluxe_stand_mixer")

    body_red = model.material("body_red", rgba=(0.72, 0.10, 0.08, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.86, 0.87, 0.89, 1.0))

    def yz_section(
        width: float,
        height: float,
        radius: float,
        x_pos: float,
        z_center: float | None = None,
    ) -> list[tuple[float, float, float]]:
        if z_center is None:
            z_center = height / 2.0
        return [(x_pos, y, z + z_center) for z, y in rounded_rect_profile(height, width, radius)]

    body = model.part("body")

    base_geom = ExtrudeGeometry(rounded_rect_profile(0.44, 0.28, 0.055), 0.07)
    body.visual(
        mesh_from_geometry(base_geom, "base_shell"),
        origin=Origin(xyz=(0.02, 0.0, 0.035)),
        material=body_red,
        name="base_shell",
    )

    pedestal_geom = section_loft(
        [
            yz_section(0.15, 0.12, 0.035, -0.15),
            yz_section(0.17, 0.26, 0.045, -0.10),
            yz_section(0.18, 0.34, 0.050, -0.05),
            yz_section(0.14, 0.28, 0.040, 0.00),
        ]
    )
    body.visual(
        mesh_from_geometry(pedestal_geom, "pedestal_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=body_red,
        name="pedestal_shell",
    )

    body.visual(
        Box((0.09, 0.12, 0.18)),
        origin=Origin(xyz=(0.02, 0.0, 0.16)),
        material=body_red,
        name="lift_tower",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.13),
        origin=Origin(xyz=(-0.12, 0.0, 0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="hinge_barrel",
    )
    body.visual(
        Box((0.045, 0.020, 0.090)),
        origin=Origin(xyz=(-0.11, 0.079, 0.35)),
        material=body_red,
        name="right_hinge_cheek",
    )
    body.visual(
        Box((0.045, 0.020, 0.090)),
        origin=Origin(xyz=(-0.11, -0.079, 0.35)),
        material=body_red,
        name="left_hinge_cheek",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.022),
        origin=Origin(xyz=(-0.12, 0.100, 0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="right_hinge_pin",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.022),
        origin=Origin(xyz=(-0.12, -0.100, 0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="left_hinge_pin",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.44, 0.28, 0.40)),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    bowl_stage = model.part("bowl_stage")
    bowl_stage.visual(
        Box((0.035, 0.10, 0.10)),
        origin=Origin(xyz=(0.0175, 0.0, 0.05)),
        material=trim_dark,
        name="carriage_block",
    )
    bowl_stage.visual(
        Box((0.10, 0.018, 0.012)),
        origin=Origin(xyz=(0.08, 0.042, 0.010)),
        material=trim_dark,
        name="right_arm",
    )
    bowl_stage.visual(
        Box((0.10, 0.018, 0.012)),
        origin=Origin(xyz=(0.08, -0.042, 0.010)),
        material=trim_dark,
        name="left_arm",
    )
    bowl_stage.visual(
        Box((0.12, 0.18, 0.012)),
        origin=Origin(xyz=(0.13, 0.0, 0.012)),
        material=trim_dark,
        name="bowl_plate",
    )
    bowl_stage.inertial = Inertial.from_geometry(
        Box((0.20, 0.18, 0.12)),
        mass=1.2,
        origin=Origin(xyz=(0.10, 0.0, 0.05)),
    )

    model.articulation(
        "body_to_bowl_stage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bowl_stage,
        origin=Origin(xyz=(0.065, 0.0, 0.085)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.06, lower=0.0, upper=0.05),
    )

    bowl = model.part("bowl")
    bowl_geom = LatheGeometry.from_shell_profiles(
        [
            (0.025, 0.000),
            (0.080, 0.010),
            (0.125, 0.050),
            (0.145, 0.115),
            (0.148, 0.155),
        ],
        [
            (0.000, 0.003),
            (0.070, 0.012),
            (0.117, 0.050),
            (0.136, 0.113),
            (0.140, 0.152),
        ],
        segments=64,
    )
    bowl.visual(
        mesh_from_geometry(bowl_geom, "mixing_bowl"),
        material=steel,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.145, length=0.155),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
    )

    model.articulation(
        "stage_to_bowl",
        ArticulationType.FIXED,
        parent=bowl_stage,
        child=bowl,
        origin=Origin(xyz=(0.19, 0.0, 0.018)),
    )

    head = model.part("head")
    head_geom = section_loft(
        [
            yz_section(0.09, 0.055, 0.020, 0.04, z_center=0.108),
            yz_section(0.13, 0.095, 0.030, 0.12, z_center=0.113),
            yz_section(0.18, 0.15, 0.042, 0.22, z_center=0.110),
            yz_section(0.17, 0.14, 0.038, 0.31, z_center=0.08),
            yz_section(0.11, 0.09, 0.028, 0.39, z_center=0.035),
        ]
    )
    head.visual(
        mesh_from_geometry(head_geom, "head_shell"),
        material=body_red,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.032, length=0.050),
        origin=Origin(xyz=(0.405, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="nose_hub",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.0, 0.121, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="right_hinge_socket",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.0, -0.121, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="left_hinge_socket",
    )
    head.visual(
        Box((0.22, 0.042, 0.10)),
        origin=Origin(xyz=(0.13, 0.109, 0.060)),
        material=body_red,
        name="right_hinge_rib",
    )
    head.visual(
        Box((0.22, 0.042, 0.10)),
        origin=Origin(xyz=(0.13, -0.109, 0.060)),
        material=body_red,
        name="left_hinge_rib",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.42, 0.18, 0.18)),
        mass=4.4,
        origin=Origin(xyz=(0.21, 0.0, 0.08)),
    )

    model.articulation(
        "body_to_head",
        ArticulationType.REVOLUTE,
        parent=body,
        child=head,
        origin=Origin(xyz=(-0.12, 0.0, 0.34)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.8, lower=0.0, upper=1.0),
    )

    whisk = model.part("whisk")
    whisk_wires = None
    for phase in (0.0, math.pi / 3.0, 2.0 * math.pi / 3.0, math.pi, 4.0 * math.pi / 3.0, 5.0 * math.pi / 3.0):
        wire = tube_from_spline_points(
            [
                (0.014 * math.cos(phase), 0.014 * math.sin(phase), -0.045),
                (0.040 * math.cos(phase + 0.35), 0.040 * math.sin(phase + 0.35), -0.080),
                (0.056 * math.cos(phase + 0.70), 0.056 * math.sin(phase + 0.70), -0.122),
                (0.034 * math.cos(phase + 1.05), 0.034 * math.sin(phase + 1.05), -0.156),
                (0.0, 0.0, -0.178),
            ],
            radius=0.0022,
            samples_per_segment=16,
            radial_segments=10,
            cap_ends=True,
        )
        whisk_wires = wire if whisk_wires is None else whisk_wires.merge(wire)

    whisk.visual(
        Cylinder(radius=0.006, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=trim_dark,
        name="drive_stub",
    )
    whisk.visual(
        Cylinder(radius=0.020, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=steel,
        name="whisk_collar",
    )
    whisk.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.181)),
        material=steel,
        name="whisk_tip",
    )
    whisk.visual(
        mesh_from_geometry(whisk_wires, "whisk_wires"),
        material=steel,
        name="whisk_wires",
    )
    whisk.inertial = Inertial.from_geometry(
        Box((0.12, 0.12, 0.20)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
    )

    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.398, 0.0, -0.022)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=22.0),
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="knob_body",
    )
    speed_knob.visual(
        Box((0.006, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, 0.024, 0.015)),
        material=trim_dark,
        name="knob_indicator",
    )
    speed_knob.inertial = Inertial.from_geometry(
        Box((0.04, 0.04, 0.04)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.02, 0.0)),
    )

    model.articulation(
        "body_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=speed_knob,
        origin=Origin(xyz=(-0.03, 0.088, 0.215)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.2),
    )

    head_lock_button = model.part("head_lock_button")
    head_lock_button.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(-0.008, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=steel,
        name="lock_button",
    )
    head_lock_button.inertial = Inertial.from_geometry(
        Box((0.03, 0.03, 0.02)),
        mass=0.05,
        origin=Origin(xyz=(-0.008, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=body,
        child=head_lock_button,
        origin=Origin(xyz=(-0.150, 0.0, 0.178)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.04, lower=0.0, upper=0.008),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bowl_stage = object_model.get_part("bowl_stage")
    bowl = object_model.get_part("bowl")
    whisk = object_model.get_part("whisk")
    speed_knob = object_model.get_part("speed_knob")
    head_lock_button = object_model.get_part("head_lock_button")
    lift = object_model.get_articulation("body_to_bowl_stage")
    head_tilt = object_model.get_articulation("body_to_head")
    whisk_drive = object_model.get_articulation("head_to_whisk")
    knob_joint = object_model.get_articulation("body_to_speed_knob")
    lock_joint = object_model.get_articulation("body_to_head_lock")

    ctx.expect_contact(
        bowl,
        bowl_stage,
        elem_a="bowl_shell",
        elem_b="bowl_plate",
        name="bowl sits on the front carriage plate",
    )
    ctx.expect_overlap(
        bowl,
        bowl_stage,
        axes="xy",
        elem_a="bowl_shell",
        elem_b="bowl_plate",
        min_overlap=0.10,
        name="bowl remains centered over the carriage platform",
    )
    ctx.expect_overlap(
        whisk,
        bowl,
        axes="xy",
        min_overlap=0.10,
        name="whisk stays centered over the bowl in the mixing pose",
    )

    rest_stage_pos = ctx.part_world_position(bowl_stage)
    raised_stage_pos = None
    closed_whisk_pos = ctx.part_world_position(whisk)
    with ctx.pose({lift: 0.05}):
        ctx.expect_gap(
            bowl_stage,
            body,
            axis="z",
            min_gap=0.05,
            max_gap=0.08,
            negative_elem="base_shell",
            name="raised carriage clears the top of the die-cast base",
        )
        raised_stage_pos = ctx.part_world_position(bowl_stage)
    opened_whisk_pos = None
    knob_turned_pos = None
    pressed_button_pos = None
    with ctx.pose({head_tilt: 1.0}):
        ctx.expect_gap(
            whisk,
            bowl,
            axis="z",
            min_gap=0.28,
            name="tilted head lifts the whisk well clear of the bowl",
        )
        opened_whisk_pos = ctx.part_world_position(whisk)
    with ctx.pose({knob_joint: 1.2}):
        knob_turned_pos = ctx.part_world_position(speed_knob)
    with ctx.pose({lock_joint: 0.008}):
        pressed_button_pos = ctx.part_world_position(head_lock_button)
    rest_knob_pos = ctx.part_world_position(speed_knob)
    rest_button_pos = ctx.part_world_position(head_lock_button)

    ctx.check(
        "bowl stage raises upward",
        rest_stage_pos is not None
        and raised_stage_pos is not None
        and raised_stage_pos[2] > rest_stage_pos[2] + 0.045,
        details=f"rest={rest_stage_pos}, raised={raised_stage_pos}",
    )
    ctx.check(
        "head tilt lifts the whisk upward",
        closed_whisk_pos is not None
        and opened_whisk_pos is not None
        and opened_whisk_pos[2] > closed_whisk_pos[2] + 0.30,
        details=f"closed={closed_whisk_pos}, opened={opened_whisk_pos}",
    )
    ctx.check(
        "speed knob rotates in place on the base",
        rest_knob_pos is not None
        and knob_turned_pos is not None
        and max(abs(a - b) for a, b in zip(rest_knob_pos, knob_turned_pos)) < 1e-6,
        details=f"rest={rest_knob_pos}, turned={knob_turned_pos}",
    )
    ctx.check(
        "head-lock button has short push travel",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[0] > rest_button_pos[0] + 0.007,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )
    ctx.check(
        "primary mechanisms use the requested articulation types",
        lift.joint_type == ArticulationType.PRISMATIC
        and head_tilt.joint_type == ArticulationType.REVOLUTE
        and whisk_drive.joint_type == ArticulationType.CONTINUOUS
        and knob_joint.joint_type == ArticulationType.REVOLUTE
        and lock_joint.joint_type == ArticulationType.PRISMATIC,
        details=(
            f"lift={lift.joint_type}, head={head_tilt.joint_type}, whisk={whisk_drive.joint_type}, "
            f"knob={knob_joint.joint_type}, lock={lock_joint.joint_type}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
