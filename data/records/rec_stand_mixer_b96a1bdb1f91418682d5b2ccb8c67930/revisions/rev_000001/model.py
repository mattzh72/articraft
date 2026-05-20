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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workhorse_stand_mixer")

    painted_metal = model.material("painted_metal", rgba=(0.24, 0.25, 0.28, 1.0))
    stainless = model.material("stainless", rgba=(0.84, 0.86, 0.89, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.66, 0.68, 0.72, 1.0))
    dark_knob = model.material("dark_knob", rgba=(0.10, 0.10, 0.11, 1.0))

    def yz_section(
        x: float,
        width_y: float,
        height_z: float,
        radius: float,
        z_center: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y, z + z_center)
            for z, y in rounded_rect_profile(height_z, width_y, radius)
        ]

    base = model.part("base")

    foot_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.40, 0.25, 0.055), 0.05),
        "base_foot",
    )
    base.visual(
        foot_mesh,
        origin=Origin(xyz=(0.02, 0.0, 0.025)),
        material=painted_metal,
        name="base_foot",
    )
    base.visual(
        Box((0.24, 0.026, 0.032)),
        origin=Origin(xyz=(0.09, 0.053, 0.066)),
        material=painted_metal,
        name="carriage_rail_right",
    )
    base.visual(
        Box((0.24, 0.026, 0.032)),
        origin=Origin(xyz=(0.09, -0.053, 0.066)),
        material=painted_metal,
        name="carriage_rail_left",
    )
    base.visual(
        Box((0.022, 0.132, 0.030)),
        origin=Origin(xyz=(-0.020, 0.0, 0.065)),
        material=painted_metal,
        name="carriage_stop_bridge",
    )

    pedestal_mesh = mesh_from_geometry(
        section_loft(
            [
                yz_section(-0.095, 0.13, 0.12, 0.030, 0.110),
                yz_section(-0.078, 0.12, 0.18, 0.032, 0.155),
                yz_section(-0.062, 0.11, 0.23, 0.030, 0.205),
                yz_section(-0.048, 0.10, 0.13, 0.026, 0.278),
            ]
        ),
        "pedestal_shell",
    )
    base.visual(
        pedestal_mesh,
        material=painted_metal,
        name="pedestal_shell",
    )
    base.visual(
        Box((0.040, 0.018, 0.040)),
        origin=Origin(xyz=(-0.072, 0.056, 0.304)),
        material=painted_metal,
        name="hinge_cheek_right",
    )
    base.visual(
        Box((0.040, 0.018, 0.040)),
        origin=Origin(xyz=(-0.072, -0.056, 0.304)),
        material=painted_metal,
        name="hinge_cheek_left",
    )
    base.visual(
        Box((0.048, 0.094, 0.022)),
        origin=Origin(xyz=(-0.078, 0.0, 0.282)),
        material=painted_metal,
        name="hinge_bridge",
    )
    base.visual(
        Box((0.056, 0.036, 0.060)),
        origin=Origin(xyz=(0.058, 0.108, 0.080)),
        material=painted_metal,
        name="knob_mount_boss",
    )
    base.visual(
        Box((0.054, 0.056, 0.020)),
        origin=Origin(xyz=(-0.105, -0.068, 0.252)),
        material=painted_metal,
        name="lock_guide_boss",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.40, 0.25, 0.34)),
        mass=9.0,
        origin=Origin(xyz=(0.00, 0.0, 0.17)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.19, 0.076, 0.020)),
        origin=Origin(xyz=(0.095, 0.0, 0.010)),
        material=satin_steel,
        name="slider_block",
    )
    carriage.visual(
        Box((0.155, 0.118, 0.018)),
        origin=Origin(xyz=(0.110, 0.0, 0.029)),
        material=painted_metal,
        name="saddle_plate",
    )
    carriage.visual(
        Cylinder(radius=0.080, length=0.012),
        origin=Origin(xyz=(0.112, 0.0, 0.044)),
        material=satin_steel,
        name="bowl_saddle",
    )
    carriage.visual(
        Box((0.030, 0.070, 0.026)),
        origin=Origin(xyz=(0.030, 0.0, 0.023)),
        material=painted_metal,
        name="rear_web",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.19, 0.118, 0.056)),
        mass=1.2,
        origin=Origin(xyz=(0.095, 0.0, 0.028)),
    )

    bowl = model.part("bowl")
    bowl_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.030, 0.000),
                (0.060, 0.018),
                (0.094, 0.066),
                (0.116, 0.132),
                (0.123, 0.164),
                (0.126, 0.171),
            ],
            [
                (0.000, 0.003),
                (0.050, 0.013),
                (0.087, 0.063),
                (0.109, 0.132),
                (0.118, 0.163),
            ],
            segments=72,
            end_cap="round",
            lip_samples=10,
        ),
        "mixing_bowl",
    )
    bowl.visual(bowl_mesh, material=stainless, name="bowl_shell")
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.171),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.0855)),
    )

    head = model.part("head")
    head_mesh = mesh_from_geometry(
        section_loft(
            [
                yz_section(0.010, 0.086, 0.082, 0.022, 0.054),
                yz_section(0.082, 0.138, 0.148, 0.040, 0.050),
                yz_section(0.188, 0.162, 0.176, 0.050, 0.048),
                yz_section(0.276, 0.126, 0.122, 0.036, 0.044),
                yz_section(0.316, 0.094, 0.094, 0.024, 0.048),
            ]
        ),
        "tilt_head_shell",
    )
    head.visual(
        head_mesh,
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=painted_metal,
        name="head_shell",
    )
    head.visual(
        Box((0.040, 0.020, 0.038)),
        origin=Origin(xyz=(0.040, 0.034, 0.018)),
        material=painted_metal,
        name="hinge_lug_right",
    )
    head.visual(
        Box((0.040, 0.020, 0.038)),
        origin=Origin(xyz=(0.040, -0.034, 0.018)),
        material=painted_metal,
        name="hinge_lug_left",
    )
    head.visual(
        Cylinder(radius=0.036, length=0.048),
        origin=Origin(xyz=(0.198, 0.0, -0.020)),
        material=painted_metal,
        name="planetary_housing",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.33, 0.17, 0.19)),
        mass=4.8,
        origin=Origin(xyz=(0.165, 0.0, -0.005)),
    )

    beater_drive = model.part("beater_drive")
    beater_drive.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=satin_steel,
        name="drive_shaft",
    )
    beater_drive.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=satin_steel,
        name="beater_coupler",
    )
    beater_drive.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.044),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
    )

    beater = model.part("beater")
    beater.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=stainless,
        name="beater_shaft",
    )
    beater.visual(
        Box((0.022, 0.010, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=stainless,
        name="beater_yoke",
    )
    beater.visual(
        Box((0.014, 0.008, 0.076)),
        origin=Origin(xyz=(-0.024, 0.0, -0.070)),
        material=stainless,
        name="beater_spine",
    )
    beater.visual(
        Box((0.068, 0.008, 0.014)),
        origin=Origin(xyz=(0.005, 0.0, -0.046)),
        material=stainless,
        name="beater_top_bar",
    )
    beater.visual(
        Box((0.014, 0.008, 0.070)),
        origin=Origin(xyz=(0.034, 0.0, -0.077)),
        material=stainless,
        name="beater_outer_leg",
    )
    beater.visual(
        Box((0.072, 0.008, 0.014)),
        origin=Origin(xyz=(0.005, 0.0, -0.108)),
        material=stainless,
        name="beater_bottom_bar",
    )
    beater.visual(
        Box((0.012, 0.008, 0.052)),
        origin=Origin(xyz=(0.002, 0.0, -0.084)),
        material=stainless,
        name="beater_inner_leg",
    )
    beater.inertial = Inertial.from_geometry(
        Box((0.082, 0.012, 0.122)),
        mass=0.28,
        origin=Origin(xyz=(0.005, 0.0, -0.062)),
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_knob,
        name="knob_body",
    )
    speed_knob.visual(
        Cylinder(radius=0.010, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="knob_core",
    )
    speed_knob.visual(
        Box((0.007, 0.008, 0.018)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=dark_knob,
        name="knob_pointer",
    )
    speed_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.026),
        mass=0.08,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    lock_control = model.part("lock_control")
    lock_control.visual(
        Box((0.030, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_knob,
        name="lock_slider_body",
    )
    lock_control.visual(
        Box((0.016, 0.022, 0.014)),
        origin=Origin(xyz=(0.010, 0.0, 0.008)),
        material=dark_knob,
        name="lock_thumb_pad",
    )
    lock_control.inertial = Inertial.from_geometry(
        Box((0.036, 0.022, 0.018)),
        mass=0.05,
        origin=Origin(xyz=(0.006, 0.0, 0.004)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.000, 0.0, 0.057)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.12,
            lower=0.0,
            upper=0.055,
        ),
    )
    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.112, 0.0, 0.050)),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.072, 0.0, 0.325)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "head_to_beater_drive",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater_drive,
        origin=Origin(xyz=(0.197, 0.0, -0.038)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=18.0),
    )
    model.articulation(
        "drive_to_beater",
        ArticulationType.FIXED,
        parent=beater_drive,
        child=beater,
        origin=Origin(xyz=(0.0, 0.0, -0.041)),
    )
    model.articulation(
        "base_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_knob,
        origin=Origin(xyz=(0.058, 0.138, 0.094)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=-math.radians(35.0),
            upper=math.radians(160.0),
        ),
    )
    model.articulation(
        "base_to_lock_control",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_control,
        origin=Origin(xyz=(-0.112, -0.104, 0.252)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.04,
            lower=0.0,
            upper=0.014,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    beater = object_model.get_part("beater")
    beater_drive = object_model.get_part("beater_drive")
    speed_knob = object_model.get_part("speed_knob")
    lock_control = object_model.get_part("lock_control")

    carriage_slide = object_model.get_articulation("base_to_carriage")
    head_hinge = object_model.get_articulation("base_to_head")
    beater_spin = object_model.get_articulation("head_to_beater_drive")
    knob_joint = object_model.get_articulation("base_to_speed_knob")
    lock_joint = object_model.get_articulation("base_to_lock_control")

    ctx.check(
        "primary mechanisms use the requested joint types",
        carriage_slide.articulation_type == ArticulationType.PRISMATIC
        and head_hinge.articulation_type == ArticulationType.REVOLUTE
        and beater_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.articulation_type == ArticulationType.REVOLUTE
        and lock_joint.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"carriage={carriage_slide.articulation_type}, "
            f"head={head_hinge.articulation_type}, "
            f"beater={beater_spin.articulation_type}, "
            f"knob={knob_joint.articulation_type}, "
            f"lock={lock_joint.articulation_type}"
        ),
    )

    head_limits = head_hinge.motion_limits
    knob_limits = knob_joint.motion_limits
    lock_limits = lock_joint.motion_limits
    ctx.check(
        "joint ranges match mixer behavior",
        head_limits is not None
        and head_limits.upper is not None
        and head_limits.upper >= math.radians(55.0)
        and knob_limits is not None
        and knob_limits.lower is not None
        and knob_limits.upper is not None
        and knob_limits.lower < 0.0 < knob_limits.upper
        and (knob_limits.upper - knob_limits.lower) >= math.radians(150.0)
        and lock_limits is not None
        and lock_limits.upper is not None
        and 0.006 <= lock_limits.upper <= 0.025,
        details=(
            f"head_limits={head_limits}, knob_limits={knob_limits}, "
            f"lock_limits={lock_limits}"
        ),
    )

    ctx.expect_contact(
        speed_knob,
        base,
        name="speed knob is mounted on the base",
    )
    ctx.expect_contact(
        lock_control,
        base,
        name="lock slider is mounted on the base",
    )
    ctx.expect_within(
        beater,
        bowl,
        axes="xy",
        margin=0.0,
        name="flat beater sits within the bowl footprint at rest",
    )

    rest_bowl_pos = ctx.part_world_position(bowl)
    rest_lock_pos = ctx.part_world_position(lock_control)
    rest_beater_pos = ctx.part_world_position(beater)
    rest_drive_pos = ctx.part_world_position(beater_drive)

    with ctx.pose({carriage_slide: carriage_slide.motion_limits.upper}):
        extended_bowl_pos = ctx.part_world_position(bowl)
    ctx.check(
        "bowl carriage translates forward",
        rest_bowl_pos is not None
        and extended_bowl_pos is not None
        and extended_bowl_pos[0] > rest_bowl_pos[0] + 0.045,
        details=f"rest={rest_bowl_pos}, extended={extended_bowl_pos}",
    )

    with ctx.pose({head_hinge: head_hinge.motion_limits.upper}):
        tilted_beater_pos = ctx.part_world_position(beater)
    ctx.check(
        "head tilts the beater upward",
        rest_beater_pos is not None
        and tilted_beater_pos is not None
        and tilted_beater_pos[2] > rest_beater_pos[2] + 0.10,
        details=f"rest={rest_beater_pos}, tilted={tilted_beater_pos}",
    )

    with ctx.pose({lock_joint: lock_joint.motion_limits.upper}):
        extended_lock_pos = ctx.part_world_position(lock_control)
        ctx.expect_contact(
            lock_control,
            base,
            name="lock slider stays captured by the base guide",
        )
    ctx.check(
        "lock control slides a short distance",
        rest_lock_pos is not None
        and extended_lock_pos is not None
        and extended_lock_pos[0] > rest_lock_pos[0] + 0.010
        and extended_lock_pos[0] < rest_lock_pos[0] + 0.020,
        details=f"rest={rest_lock_pos}, extended={extended_lock_pos}",
    )

    with ctx.pose({beater_spin: math.pi / 2.0}):
        spun_beater_pos = ctx.part_world_position(beater)
        spun_drive_pos = ctx.part_world_position(beater_drive)
    ctx.check(
        "beater spins in place beneath the head",
        rest_beater_pos is not None
        and spun_beater_pos is not None
        and rest_drive_pos is not None
        and spun_drive_pos is not None
        and abs(spun_beater_pos[0] - rest_beater_pos[0]) < 1e-9
        and abs(spun_beater_pos[1] - rest_beater_pos[1]) < 1e-9
        and abs(spun_beater_pos[2] - rest_beater_pos[2]) < 1e-9
        and abs(spun_drive_pos[0] - rest_drive_pos[0]) < 1e-9
        and abs(spun_drive_pos[1] - rest_drive_pos[1]) < 1e-9
        and abs(spun_drive_pos[2] - rest_drive_pos[2]) < 1e-9,
        details=(
            f"rest_beater={rest_beater_pos}, spun_beater={spun_beater_pos}, "
            f"rest_drive={rest_drive_pos}, spun_drive={spun_drive_pos}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
