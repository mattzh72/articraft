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
    repair_loft,
    rounded_rect_profile,
    section_loft,
)


def _yz_section(
    *,
    x: float,
    center_z: float,
    width_y: float,
    height_z: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(height_z, width_y, radius)
    return [(x, y, center_z + z) for z, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_head_stand_mixer")

    body_dark = model.material("body_dark", rgba=(0.16, 0.16, 0.18, 1.0))
    body_trim = model.material("body_trim", rgba=(0.22, 0.22, 0.24, 1.0))
    steel = model.material("steel", rgba=(0.82, 0.84, 0.86, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.55, 0.57, 0.60, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.36, 0.24, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=body_dark,
        name="base_plate",
    )
    base.visual(
        Box((0.19, 0.14, 0.02)),
        origin=Origin(xyz=(0.065, 0.0, 0.040)),
        material=body_trim,
        name="carriage_guide",
    )
    neck_geom = section_loft(
        [
            _yz_section(x=-0.120, center_z=0.110, width_y=0.140, height_z=0.180, radius=0.035),
            _yz_section(x=-0.095, center_z=0.190, width_y=0.132, height_z=0.200, radius=0.035),
            _yz_section(x=-0.060, center_z=0.285, width_y=0.118, height_z=0.128, radius=0.026),
        ]
    )
    base.visual(
        mesh_from_geometry(neck_geom, "mixer_rear_neck"),
        material=body_dark,
        name="rear_neck",
    )
    base.visual(
        Box((0.085, 0.155, 0.034)),
        origin=Origin(xyz=(-0.076, 0.0, 0.307)),
        material=body_dark,
        name="pivot_block",
    )
    base.visual(
        Box((0.040, 0.024, 0.008)),
        origin=Origin(xyz=(-0.151, 0.0, 0.034)),
        material=body_trim,
        name="lock_track",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.24, 0.34)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.16, 0.11, 0.010)),
        origin=Origin(xyz=(0.050, 0.0, 0.005)),
        material=steel_dark,
        name="slide_deck",
    )
    carriage.visual(
        Box((0.050, 0.070, 0.008)),
        origin=Origin(xyz=(-0.015, 0.0, 0.004)),
        material=steel_dark,
        name="retained_tongue",
    )
    carriage.visual(
        Box((0.082, 0.062, 0.032)),
        origin=Origin(xyz=(0.060, 0.0, 0.026)),
        material=body_trim,
        name="bowl_pedestal",
    )
    carriage.visual(
        Box((0.102, 0.084, 0.008)),
        origin=Origin(xyz=(0.060, 0.0, 0.046)),
        material=steel_dark,
        name="bowl_seat",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.18, 0.12, 0.06)),
        mass=0.9,
        origin=Origin(xyz=(0.045, 0.0, 0.030)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.035, 0.0, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.10, lower=0.0, upper=0.050),
    )

    bowl = model.part("bowl")
    bowl_geom = LatheGeometry.from_shell_profiles(
        [
            (0.040, 0.000),
            (0.055, 0.010),
            (0.090, 0.028),
            (0.108, 0.075),
            (0.114, 0.122),
            (0.116, 0.135),
        ],
        [
            (0.020, 0.004),
            (0.046, 0.012),
            (0.082, 0.030),
            (0.100, 0.075),
            (0.106, 0.126),
        ],
        segments=56,
        end_cap="round",
        lip_samples=8,
    )
    bowl.visual(
        mesh_from_geometry(bowl_geom, "mixer_bowl_shell"),
        material=steel,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.116, length=0.135),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
    )

    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.060, 0.0, 0.048)),
    )

    head = model.part("head")
    head_geom = repair_loft(
        section_loft(
            [
                _yz_section(x=0.080, center_z=-0.002, width_y=0.128, height_z=0.114, radius=0.032),
                _yz_section(x=0.165, center_z=-0.010, width_y=0.170, height_z=0.138, radius=0.042),
                _yz_section(x=0.255, center_z=-0.022, width_y=0.110, height_z=0.094, radius=0.028),
            ]
        ),
        repair="mesh",
    )
    head.visual(
        mesh_from_geometry(head_geom, "mixer_head_shell"),
        material=body_dark,
        name="head_shell",
    )
    head.visual(
        Box((0.017, 0.146, 0.030)),
        origin=Origin(xyz=(0.080, 0.0, 0.000)),
        material=body_trim,
        name="rear_hinge_saddle",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.072),
        origin=Origin(xyz=(0.208, 0.0, -0.047)),
        material=body_trim,
        name="drive_nose",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.25, 0.19, 0.18)),
        mass=4.1,
        origin=Origin(xyz=(0.120, 0.0, -0.020)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.105, 0.0, 0.325)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    beater = model.part("beater")
    beater.visual(
        Cylinder(radius=0.0065, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.0275)),
        material=steel,
        name="drive_shaft",
    )
    beater.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.061)),
        material=steel_dark,
        name="shaft_collar",
    )
    beater.visual(
        Box((0.054, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.068)),
        material=steel,
        name="upper_bridge",
    )
    beater.visual(
        Box((0.010, 0.007, 0.068)),
        origin=Origin(xyz=(-0.020, 0.0, -0.101)),
        material=steel,
        name="left_blade",
    )
    beater.visual(
        Box((0.010, 0.007, 0.070)),
        origin=Origin(xyz=(0.000, 0.0, -0.102)),
        material=steel,
        name="center_blade",
    )
    beater.visual(
        Box((0.010, 0.007, 0.060)),
        origin=Origin(xyz=(0.020, 0.0, -0.097)),
        material=steel,
        name="right_blade",
    )
    beater.visual(
        Box((0.042, 0.007, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.132)),
        material=steel,
        name="lower_bridge",
    )
    beater.inertial = Inertial.from_geometry(
        Box((0.060, 0.020, 0.180)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
    )

    model.articulation(
        "head_to_beater",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0.202, 0.0, -0.082)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="knob_stem",
    )
    speed_knob.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    speed_knob.visual(
        Box((0.004, 0.006, 0.014)),
        origin=Origin(xyz=(0.010, 0.030, 0.0)),
        material=steel,
        name="knob_pointer",
    )
    speed_knob.inertial = Inertial.from_geometry(
        Box((0.038, 0.030, 0.038)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
    )

    model.articulation(
        "base_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_knob,
        origin=Origin(xyz=(-0.060, 0.0775, 0.304)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=-2.4, upper=2.4),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.022, 0.014, 0.010)),
        origin=Origin(xyz=(0.011, 0.0, 0.005)),
        material=knob_black,
        name="lock_slider",
    )
    head_lock.visual(
        Box((0.010, 0.014, 0.008)),
        origin=Origin(xyz=(0.007, 0.0, 0.014)),
        material=body_trim,
        name="lock_rib",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.024, 0.016, 0.018)),
        mass=0.03,
        origin=Origin(xyz=(0.011, 0.0, 0.009)),
    )

    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.171, 0.0, 0.038)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.04, lower=0.0, upper=0.010),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    beater = object_model.get_part("beater")
    speed_knob = object_model.get_part("speed_knob")
    head_lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    beater_drive = object_model.get_articulation("head_to_beater")
    knob_joint = object_model.get_articulation("base_to_speed_knob")
    lock_joint = object_model.get_articulation("base_to_head_lock")

    ctx.check(
        "canonical articulation types are present",
        (
            bowl_slide.joint_type == ArticulationType.PRISMATIC
            and head_tilt.joint_type == ArticulationType.REVOLUTE
            and beater_drive.joint_type == ArticulationType.CONTINUOUS
            and knob_joint.joint_type == ArticulationType.REVOLUTE
            and lock_joint.joint_type == ArticulationType.PRISMATIC
        ),
        details=(
            f"bowl={bowl_slide.joint_type}, head={head_tilt.joint_type}, "
            f"beater={beater_drive.joint_type}, knob={knob_joint.joint_type}, "
            f"lock={lock_joint.joint_type}"
        ),
    )
    ctx.check(
        "canonical joint axes match the mixer mechanisms",
        bowl_slide.axis == (1.0, 0.0, 0.0)
        and head_tilt.axis == (0.0, -1.0, 0.0)
        and beater_drive.axis == (0.0, 0.0, 1.0)
        and knob_joint.axis == (0.0, 1.0, 0.0)
        and lock_joint.axis == (1.0, 0.0, 0.0),
        details=(
            f"bowl={bowl_slide.axis}, head={head_tilt.axis}, beater={beater_drive.axis}, "
            f"knob={knob_joint.axis}, lock={lock_joint.axis}"
        ),
    )

    ctx.expect_contact(
        carriage,
        base,
        elem_a="slide_deck",
        elem_b="carriage_guide",
        name="carriage rides on the guide deck",
    )
    ctx.expect_contact(
        head_lock,
        base,
        elem_a="lock_slider",
        elem_b="lock_track",
        name="head lock sits on its track",
    )
    ctx.expect_overlap(
        bowl,
        carriage,
        axes="xy",
        elem_a="bowl_shell",
        elem_b="bowl_seat",
        min_overlap=0.070,
        name="bowl stays seated over the carriage",
    )
    ctx.expect_overlap(
        head,
        base,
        axes="y",
        elem_a="head_shell",
        elem_b="pivot_block",
        min_overlap=0.080,
        name="head lines up with the rear pivot block",
    )
    ctx.expect_contact(
        head,
        base,
        elem_a="rear_hinge_saddle",
        elem_b="pivot_block",
        name="head rests on the rear tilt pivot",
    )

    bowl_rest = ctx.part_world_position(bowl)
    beater_rest = ctx.part_world_position(beater)
    lock_rest = ctx.part_world_position(head_lock)

    bowl_upper = bowl_slide.motion_limits.upper if bowl_slide.motion_limits else None
    head_upper = head_tilt.motion_limits.upper if head_tilt.motion_limits else None
    lock_upper = lock_joint.motion_limits.upper if lock_joint.motion_limits else None

    with ctx.pose({bowl_slide: bowl_upper if bowl_upper is not None else 0.0}):
        bowl_extended = ctx.part_world_position(bowl)
        ctx.expect_contact(
            carriage,
            base,
            elem_a="slide_deck",
            elem_b="carriage_guide",
            name="carriage remains supported when extended",
        )
        ctx.check(
            "bowl carriage moves forward",
            bowl_rest is not None
            and bowl_extended is not None
            and bowl_extended[0] > bowl_rest[0] + 0.040,
            details=f"rest={bowl_rest}, extended={bowl_extended}",
        )

    with ctx.pose({head_tilt: head_upper if head_upper is not None else 0.0}):
        beater_raised = ctx.part_world_position(beater)
        ctx.check(
            "tilt head lifts the beater clear",
            beater_rest is not None
            and beater_raised is not None
            and beater_raised[2] > beater_rest[2] + 0.090,
            details=f"rest={beater_rest}, raised={beater_raised}",
        )

    with ctx.pose({lock_joint: lock_upper if lock_upper is not None else 0.0}):
        lock_extended = ctx.part_world_position(head_lock)
        ctx.check(
            "head lock slider moves forward",
            lock_rest is not None
            and lock_extended is not None
            and lock_extended[0] > lock_rest[0] + 0.007,
            details=f"rest={lock_rest}, extended={lock_extended}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
