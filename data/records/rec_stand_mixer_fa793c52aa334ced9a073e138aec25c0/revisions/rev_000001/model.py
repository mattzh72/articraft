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
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_stand_mixer")

    body_enamel = model.material("body_enamel", rgba=(0.80, 0.16, 0.18, 1.0))
    bowl_metal = model.material("bowl_metal", rgba=(0.92, 0.93, 0.96, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.82, 0.84, 0.86, 1.0))

    def xy_section(
        cx: float,
        cy: float,
        width_x: float,
        width_y: float,
        radius: float,
        z: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (cx + px, cy + py, z)
            for px, py in rounded_rect_profile(width_x, width_y, radius)
        ]

    def yz_section(
        x: float,
        width_y: float,
        height_z: float,
        radius: float,
        z_center: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, py, z_center + pz)
            for py, pz in rounded_rect_profile(width_y, height_z, radius)
        ]

    base = model.part("base")

    base.visual(
        Box((0.33, 0.21, 0.048)),
        origin=Origin(xyz=(0.040, 0.0, 0.024)),
        material=body_enamel,
        name="base_plate",
    )
    base.visual(
        Box((0.16, 0.13, 0.012)),
        origin=Origin(xyz=(0.090, 0.0, 0.054)),
        material=body_enamel,
        name="carriage_deck",
    )
    base.visual(
        Box((0.022, 0.13, 0.020)),
        origin=Origin(xyz=(0.165, 0.0, 0.058)),
        material=body_enamel,
        name="deck_front_stop",
    )

    neck_mesh = mesh_from_geometry(
        section_loft(
            [
                xy_section(-0.060, 0.0, 0.130, 0.138, 0.034, 0.034),
                xy_section(-0.070, 0.0, 0.108, 0.118, 0.030, 0.145),
                xy_section(-0.082, 0.0, 0.076, 0.092, 0.022, 0.230),
                xy_section(-0.090, 0.0, 0.048, 0.062, 0.016, 0.272),
            ]
        ),
        "mixer_neck",
    )
    base.visual(neck_mesh, material=body_enamel, name="neck_shell")
    base.visual(
        Box((0.016, 0.018, 0.012)),
        origin=Origin(xyz=(-0.046, -0.030, 0.264)),
        material=trim_dark,
        name="speed_mount_boss",
    )
    base.visual(
        Box((0.020, 0.018, 0.016)),
        origin=Origin(xyz=(-0.060, -0.030, 0.264)),
        material=trim_dark,
        name="speed_mount_rib",
    )
    base.visual(
        Box((0.038, 0.032, 0.008)),
        origin=Origin(xyz=(-0.105, 0.0, 0.230)),
        material=trim_dark,
        name="head_lock_guide",
    )
    base.visual(
        Box((0.030, 0.040, 0.060)),
        origin=Origin(xyz=(-0.095, 0.046, 0.286)),
        material=body_enamel,
        name="left_hinge_cheek",
    )
    base.visual(
        Box((0.030, 0.040, 0.060)),
        origin=Origin(xyz=(-0.095, -0.046, 0.286)),
        material=body_enamel,
        name="right_hinge_cheek",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.22, 0.33)),
        mass=8.5,
        origin=Origin(xyz=(0.025, 0.0, 0.165)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.100, 0.118, 0.010)),
        origin=Origin(xyz=(0.050, 0.0, 0.005)),
        material=body_enamel,
        name="carriage_plate",
    )
    carriage.visual(
        Box((0.018, 0.118, 0.018)),
        origin=Origin(xyz=(0.009, 0.0, 0.009)),
        material=body_enamel,
        name="carriage_back_web",
    )
    carriage.visual(
        Cylinder(radius=0.045, length=0.008),
        origin=Origin(xyz=(0.073, 0.0, 0.014)),
        material=trim_dark,
        name="carriage_pad",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.10, 0.12, 0.02)),
        mass=0.5,
        origin=Origin(xyz=(0.050, 0.0, 0.010)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.020, 0.0, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.10,
            lower=0.0,
            upper=0.038,
        ),
    )

    bowl = model.part("bowl")
    bowl_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.020, 0.000),
                (0.060, 0.006),
                (0.096, 0.042),
                (0.108, 0.108),
                (0.112, 0.152),
                (0.111, 0.160),
            ],
            [
                (0.006, 0.004),
                (0.054, 0.012),
                (0.090, 0.045),
                (0.101, 0.110),
                (0.105, 0.154),
            ],
            segments=40,
            start_cap="flat",
            end_cap="flat",
            lip_samples=10,
        ),
        "mixer_bowl",
    )
    bowl.visual(bowl_mesh, material=bowl_metal, name="bowl_shell")
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.112, length=0.160),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.073, 0.0, 0.018)),
    )

    head = model.part("head")
    head_shell_mesh = mesh_from_geometry(
        section_loft(
            [
                yz_section(0.098, 0.086, 0.090, 0.024, 0.016),
                yz_section(0.170, 0.132, 0.142, 0.040, 0.018),
                yz_section(0.250, 0.148, 0.154, 0.044, 0.010),
                yz_section(0.315, 0.114, 0.122, 0.034, -0.006),
                yz_section(0.350, 0.078, 0.082, 0.018, -0.014),
            ]
        ),
        "mixer_head_shell",
    )
    head.visual(head_shell_mesh, material=body_enamel, name="head_shell")
    head.visual(
        Box((0.096, 0.052, 0.042)),
        origin=Origin(xyz=(0.060, 0.0, 0.042)),
        material=body_enamel,
        name="rear_bridge",
    )
    head.visual(
        Cylinder(radius=0.028, length=0.052),
        origin=Origin(xyz=(-0.006, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="pivot_barrel",
    )
    head.visual(
        Box((0.060, 0.060, 0.026)),
        origin=Origin(xyz=(0.220, 0.0, -0.016)),
        material=trim_dark,
        name="front_nose",
    )
    head.visual(
        Cylinder(radius=0.040, length=0.054),
        origin=Origin(xyz=(0.208, 0.0, -0.030)),
        material=trim_dark,
        name="drive_collar",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.205, 0.0, -0.066)),
        material=steel,
        name="drive_spindle",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.34, 0.16, 0.18)),
        mass=4.3,
        origin=Origin(xyz=(0.155, 0.0, -0.005)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.082, 0.0, 0.315)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(60.0),
        ),
    )

    whisk = model.part("whisk")
    whisk_geom = CylinderGeometry(radius=0.0054, height=0.034).translate(0.0, 0.0, -0.017)
    whisk_geom.merge(
        CylinderGeometry(radius=0.0080, height=0.020).translate(0.0, 0.0, -0.042)
    )
    whisk_geom.merge(
        CylinderGeometry(radius=0.0110, height=0.018).translate(0.0, 0.0, -0.060)
    )
    for i in range(6):
        angle = i * math.pi / 3.0
        c = math.cos(angle)
        s = math.sin(angle)
        whisk_geom.merge(
            tube_from_spline_points(
                [
                    (0.010 * c, 0.010 * s, -0.050),
                    (0.024 * c, 0.024 * s, -0.070),
                    (0.040 * c, 0.040 * s, -0.098),
                    (0.050 * c, 0.050 * s, -0.118),
                    (0.000, 0.000, -0.142),
                    (-0.050 * c, -0.050 * s, -0.118),
                    (-0.040 * c, -0.040 * s, -0.098),
                    (-0.024 * c, -0.024 * s, -0.070),
                    (-0.010 * c, -0.010 * s, -0.050),
                ],
                radius=0.0015,
                samples_per_segment=10,
                radial_segments=8,
            )
        )
    whisk.visual(
        mesh_from_geometry(whisk_geom, "mixer_balloon_whisk"),
        material=steel,
        name="whisk_shell",
    )
    whisk.inertial = Inertial.from_geometry(
        Cylinder(radius=0.052, length=0.142),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -0.071)),
    )

    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.205, 0.0, -0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=22.0),
    )

    speed_control = model.part("speed_control")
    speed_control.visual(
        Box((0.010, 0.014, 0.006)),
        origin=Origin(xyz=(-0.003, 0.0, 0.003)),
        material=trim_dark,
        name="speed_mount",
    )
    speed_control.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="speed_pivot",
    )
    speed_control.visual(
        Box((0.034, 0.010, 0.010)),
        origin=Origin(xyz=(0.018, 0.0, 0.010)),
        material=trim_dark,
        name="speed_arm",
    )
    speed_control.visual(
        Box((0.014, 0.018, 0.014)),
        origin=Origin(xyz=(0.040, 0.0, 0.013)),
        material=trim_dark,
        name="speed_grip",
    )
    speed_control.inertial = Inertial.from_geometry(
        Box((0.055, 0.020, 0.025)),
        mass=0.08,
        origin=Origin(xyz=(0.020, 0.0, 0.010)),
    )

    model.articulation(
        "base_to_speed_control",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_control,
        origin=Origin(xyz=(-0.030, -0.030, 0.259)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=-0.12,
            upper=0.55,
        ),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.022, 0.024, 0.010)),
        origin=Origin(xyz=(0.011, 0.0, 0.005)),
        material=trim_dark,
        name="lock_slider",
    )
    head_lock.visual(
        Box((0.010, 0.026, 0.010)),
        origin=Origin(xyz=(0.011, 0.0, 0.013)),
        material=trim_dark,
        name="lock_thumb_ridge",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.022, 0.026, 0.018)),
        mass=0.05,
        origin=Origin(xyz=(0.011, 0.0, 0.009)),
    )

    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.118, 0.0, 0.234)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=0.08,
            lower=0.0,
            upper=0.018,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bowl = object_model.get_part("bowl")
    carriage = object_model.get_part("carriage")
    whisk = object_model.get_part("whisk")
    speed_control = object_model.get_part("speed_control")
    head_lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    whisk_drive = object_model.get_articulation("head_to_whisk")
    speed_joint = object_model.get_articulation("base_to_speed_control")
    lock_joint = object_model.get_articulation("base_to_head_lock")

    ctx.check(
        "prompt kinematic structure is present",
        bowl_slide.articulation_type == ArticulationType.PRISMATIC
        and head_tilt.articulation_type == ArticulationType.REVOLUTE
        and whisk_drive.articulation_type == ArticulationType.CONTINUOUS
        and speed_joint.articulation_type == ArticulationType.REVOLUTE
        and lock_joint.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"types={[bowl_slide.articulation_type, head_tilt.articulation_type, whisk_drive.articulation_type, speed_joint.articulation_type, lock_joint.articulation_type]}"
        ),
    )
    ctx.check(
        "primary joint axes match the mixer layout",
        bowl_slide.axis == (1.0, 0.0, 0.0)
        and head_tilt.axis == (0.0, -1.0, 0.0)
        and whisk_drive.axis == (0.0, 0.0, 1.0)
        and speed_joint.axis == (0.0, -1.0, 0.0)
        and lock_joint.axis == (1.0, 0.0, 0.0),
        details=(
            f"axes={[bowl_slide.axis, head_tilt.axis, whisk_drive.axis, speed_joint.axis, lock_joint.axis]}"
        ),
    )

    ctx.expect_overlap(
        whisk,
        bowl,
        axes="xy",
        min_overlap=0.08,
        name="whisk stays centered over the bowl at rest",
    )
    ctx.expect_gap(
        bowl,
        carriage,
        axis="z",
        max_gap=0.004,
        max_penetration=0.0,
        negative_elem="carriage_pad",
        name="bowl sits directly on the carriage pad",
    )

    bowl_slide_upper = bowl_slide.motion_limits.upper
    head_tilt_upper = head_tilt.motion_limits.upper
    speed_upper = speed_joint.motion_limits.upper
    lock_upper = lock_joint.motion_limits.upper

    rest_bowl_pos = ctx.part_world_position(bowl)
    with ctx.pose({bowl_slide: bowl_slide_upper}):
        slid_bowl_pos = ctx.part_world_position(bowl)
    ctx.check(
        "bowl carriage slides forward",
        rest_bowl_pos is not None
        and slid_bowl_pos is not None
        and slid_bowl_pos[0] > rest_bowl_pos[0] + 0.025,
        details=f"rest={rest_bowl_pos}, slid={slid_bowl_pos}",
    )

    rest_whisk_pos = ctx.part_world_position(whisk)
    with ctx.pose({head_tilt: head_tilt_upper}):
        raised_whisk_pos = ctx.part_world_position(whisk)
        ctx.expect_gap(
            whisk,
            bowl,
            axis="z",
            min_gap=0.020,
            name="tilted head lifts whisk clear of the bowl rim",
        )
    ctx.check(
        "head tilt raises the whisk",
        rest_whisk_pos is not None
        and raised_whisk_pos is not None
        and raised_whisk_pos[2] > rest_whisk_pos[2] + 0.070,
        details=f"rest={rest_whisk_pos}, raised={raised_whisk_pos}",
    )

    rest_speed_grip = ctx.part_element_world_aabb(speed_control, elem="speed_grip")
    with ctx.pose({speed_joint: speed_upper}):
        moved_speed_grip = ctx.part_element_world_aabb(speed_control, elem="speed_grip")
    ctx.check(
        "speed control lever swings through an arc",
        rest_speed_grip is not None
        and moved_speed_grip is not None
        and moved_speed_grip[1][2] > rest_speed_grip[1][2] + 0.008,
        details=f"rest={rest_speed_grip}, moved={moved_speed_grip}",
    )

    rest_lock_pos = ctx.part_world_position(head_lock)
    with ctx.pose({lock_joint: lock_upper}):
        slid_lock_pos = ctx.part_world_position(head_lock)
    ctx.check(
        "head lock slider travels forward",
        rest_lock_pos is not None
        and slid_lock_pos is not None
        and slid_lock_pos[0] > rest_lock_pos[0] + 0.010,
        details=f"rest={rest_lock_pos}, moved={slid_lock_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
