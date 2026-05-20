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


ARM_BARREL_RADIUS = 0.018
ARM_BARREL_LENGTH = 0.030
ARM_BEAM_WIDTH = 0.028
ARM_BEAM_THICKNESS = 0.016
ARM_FORK_DEPTH = 0.026
ARM_FORK_THICKNESS = 0.008
ARM_FORK_GAP = 0.030
ARM_FORK_HEIGHT = 0.050
ARM_FORK_BRIDGE_LENGTH = 0.024


def _y_axis_cylinder(
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[Cylinder, Origin]:
    return (
        Cylinder(radius=radius, length=length),
        Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
    )


def _add_clevis(
    part,
    *,
    name_prefix: str,
    joint_xyz: tuple[float, float, float],
    pitch: float,
    material,
) -> None:
    joint_x, _, joint_z = joint_xyz
    unit_x = math.cos(pitch)
    unit_z = math.sin(pitch)
    cheek_center_y = ARM_FORK_GAP * 0.5 + ARM_FORK_THICKNESS * 0.5
    cheek_center = (
        joint_x - unit_x * (ARM_FORK_DEPTH * 0.5),
        0.0,
        joint_z - unit_z * (ARM_FORK_DEPTH * 0.5),
    )
    bridge_center = (
        joint_x - unit_x * (ARM_FORK_DEPTH + ARM_FORK_BRIDGE_LENGTH * 0.5),
        0.0,
        joint_z - unit_z * (ARM_FORK_DEPTH + ARM_FORK_BRIDGE_LENGTH * 0.5),
    )
    cheek_origin = Origin(xyz=cheek_center, rpy=(0.0, -pitch, 0.0))
    bridge_origin = Origin(xyz=bridge_center, rpy=(0.0, -pitch, 0.0))

    part.visual(
        Box((ARM_FORK_DEPTH, ARM_FORK_THICKNESS, ARM_FORK_HEIGHT)),
        origin=Origin(
            xyz=(cheek_center[0], cheek_center_y, cheek_center[2]),
            rpy=cheek_origin.rpy,
        ),
        material=material,
        name=f"{name_prefix}_fork_upper",
    )
    part.visual(
        Box((ARM_FORK_DEPTH, ARM_FORK_THICKNESS, ARM_FORK_HEIGHT)),
        origin=Origin(
            xyz=(cheek_center[0], -cheek_center_y, cheek_center[2]),
            rpy=cheek_origin.rpy,
        ),
        material=material,
        name=f"{name_prefix}_fork_lower",
    )
    part.visual(
        Box(
            (
                ARM_FORK_BRIDGE_LENGTH,
                ARM_FORK_GAP + ARM_FORK_THICKNESS * 2.0,
                ARM_FORK_HEIGHT * 0.46,
            )
        ),
        origin=bridge_origin,
        material=material,
        name=f"{name_prefix}_fork_bridge",
    )


def _add_arm_link(
    part,
    *,
    name_prefix: str,
    dx: float,
    dz: float,
    beam_material,
    joint_material,
) -> None:
    pitch = math.atan2(dz, dx)
    span = math.hypot(dx, dz)
    beam_length = span - (ARM_FORK_DEPTH + ARM_FORK_BRIDGE_LENGTH)
    if beam_length <= 0.0:
        raise ValueError("Arm span is too short for the fork geometry.")

    beam_center = (
        math.cos(pitch) * beam_length * 0.5,
        0.0,
        math.sin(pitch) * beam_length * 0.5,
    )
    barrel_geometry, barrel_origin = _y_axis_cylinder(
        radius=ARM_BARREL_RADIUS,
        length=ARM_BARREL_LENGTH,
    )
    part.visual(
        barrel_geometry,
        origin=barrel_origin,
        material=joint_material,
        name=f"{name_prefix}_prox_barrel",
    )
    part.visual(
        Box((beam_length, ARM_BEAM_WIDTH, ARM_BEAM_THICKNESS)),
        origin=Origin(xyz=beam_center, rpy=(0.0, -pitch, 0.0)),
        material=beam_material,
        name=f"{name_prefix}_beam",
    )
    _add_clevis(
        part,
        name_prefix=f"{name_prefix}_distal",
        joint_xyz=(dx, 0.0, dz),
        pitch=pitch,
        material=joint_material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_painting_floor_lamp")

    base_paint = model.material("base_paint", rgba=(0.14, 0.14, 0.15, 1.0))
    arm_paint = model.material("arm_paint", rgba=(0.18, 0.18, 0.19, 1.0))
    joint_metal = model.material("joint_metal", rgba=(0.52, 0.53, 0.55, 1.0))
    panel_frame = model.material("panel_frame", rgba=(0.17, 0.17, 0.18, 1.0))
    panel_diffuser = model.material("panel_diffuser", rgba=(0.95, 0.94, 0.89, 0.78))

    shoulder_xyz = (0.060, 0.0, 1.335)
    arm_1_vector = (0.340, 0.200)
    arm_2_vector = (0.310, 0.150)
    arm_3_vector = (0.240, 0.080)

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.190, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=base_paint,
        name="base_disc",
    )
    stand.visual(
        Cylinder(radius=0.145, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=base_paint,
        name="base_cap",
    )
    stand.visual(
        Cylinder(radius=0.018, length=1.260),
        origin=Origin(xyz=(0.0, 0.0, 0.658)),
        material=arm_paint,
        name="main_post",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.318)),
        material=joint_metal,
        name="post_collar",
    )
    stand.visual(
        Box((0.032, 0.040, 0.028)),
        origin=Origin(xyz=(0.016, 0.0, shoulder_xyz[2])),
        material=arm_paint,
        name="shoulder_neck",
    )
    stand.visual(
        Box((0.020, 0.046, 0.022)),
        origin=Origin(xyz=(0.036, 0.0, shoulder_xyz[2])),
        material=joint_metal,
        name="shoulder_bridge",
    )
    stand.visual(
        Box((0.028, 0.008, 0.050)),
        origin=Origin(xyz=(0.046, 0.019, shoulder_xyz[2])),
        material=joint_metal,
        name="shoulder_fork_upper",
    )
    stand.visual(
        Box((0.028, 0.008, 0.050)),
        origin=Origin(xyz=(0.046, -0.019, shoulder_xyz[2])),
        material=joint_metal,
        name="shoulder_fork_lower",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.380, 0.380, 1.360)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.680)),
    )

    arm_1 = model.part("arm_1")
    _add_arm_link(
        arm_1,
        name_prefix="arm_1",
        dx=arm_1_vector[0],
        dz=arm_1_vector[1],
        beam_material=arm_paint,
        joint_material=joint_metal,
    )
    arm_1.inertial = Inertial.from_geometry(
        Box((0.360, 0.060, 0.090)),
        mass=1.1,
        origin=Origin(xyz=(0.180, 0.0, 0.100)),
    )

    arm_2 = model.part("arm_2")
    _add_arm_link(
        arm_2,
        name_prefix="arm_2",
        dx=arm_2_vector[0],
        dz=arm_2_vector[1],
        beam_material=arm_paint,
        joint_material=joint_metal,
    )
    arm_2.inertial = Inertial.from_geometry(
        Box((0.330, 0.060, 0.080)),
        mass=0.9,
        origin=Origin(xyz=(0.165, 0.0, 0.075)),
    )

    arm_3 = model.part("arm_3")
    _add_arm_link(
        arm_3,
        name_prefix="arm_3",
        dx=arm_3_vector[0],
        dz=arm_3_vector[1],
        beam_material=arm_paint,
        joint_material=joint_metal,
    )
    arm_3.inertial = Inertial.from_geometry(
        Box((0.260, 0.060, 0.070)),
        mass=0.75,
        origin=Origin(xyz=(0.120, 0.0, 0.040)),
    )

    lamp_panel = model.part("lamp_panel")
    panel_barrel_geometry, panel_barrel_origin = _y_axis_cylinder(
        radius=0.017,
        length=ARM_BARREL_LENGTH,
    )
    lamp_panel.visual(
        panel_barrel_geometry,
        origin=panel_barrel_origin,
        material=joint_metal,
        name="panel_hinge_barrel",
    )
    lamp_panel.visual(
        Box((0.040, 0.070, 0.060)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=panel_frame,
        name="panel_mount_block",
    )
    lamp_panel.visual(
        Box((0.022, 0.340, 0.140)),
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material=panel_frame,
        name="panel_shell",
    )
    lamp_panel.visual(
        Box((0.004, 0.300, 0.108)),
        origin=Origin(xyz=(0.041, 0.0, 0.0)),
        material=panel_diffuser,
        name="panel_diffuser",
    )
    lamp_panel.inertial = Inertial.from_geometry(
        Box((0.064, 0.340, 0.140)),
        mass=0.9,
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
    )

    model.articulation(
        "stand_to_arm_1",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=arm_1,
        origin=Origin(xyz=shoulder_xyz),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-0.95,
            upper=1.10,
        ),
    )
    model.articulation(
        "arm_1_to_arm_2",
        ArticulationType.REVOLUTE,
        parent=arm_1,
        child=arm_2,
        origin=Origin(xyz=(arm_1_vector[0], 0.0, arm_1_vector[1])),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.5,
            lower=-1.15,
            upper=1.15,
        ),
    )
    model.articulation(
        "arm_2_to_arm_3",
        ArticulationType.REVOLUTE,
        parent=arm_2,
        child=arm_3,
        origin=Origin(xyz=(arm_2_vector[0], 0.0, arm_2_vector[1])),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.7,
            lower=-1.10,
            upper=1.05,
        ),
    )
    model.articulation(
        "arm_3_to_panel",
        ArticulationType.REVOLUTE,
        parent=arm_3,
        child=lamp_panel,
        origin=Origin(xyz=(arm_3_vector[0], 0.0, arm_3_vector[1])),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-0.85,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    stand = object_model.get_part("stand")
    arm_1 = object_model.get_part("arm_1")
    arm_2 = object_model.get_part("arm_2")
    arm_3 = object_model.get_part("arm_3")
    lamp_panel = object_model.get_part("lamp_panel")

    stand_to_arm_1 = object_model.get_articulation("stand_to_arm_1")
    arm_1_to_arm_2 = object_model.get_articulation("arm_1_to_arm_2")
    arm_2_to_arm_3 = object_model.get_articulation("arm_2_to_arm_3")
    arm_3_to_panel = object_model.get_articulation("arm_3_to_panel")

    for joint in (
        stand_to_arm_1,
        arm_1_to_arm_2,
        arm_2_to_arm_3,
        arm_3_to_panel,
    ):
        axis = tuple(round(value, 6) for value in joint.axis)
        ctx.check(
            f"{joint.name} is a pitch hinge",
            joint.articulation_type == ArticulationType.REVOLUTE and axis == (0.0, -1.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    ctx.expect_origin_gap(
        lamp_panel,
        stand,
        axis="x",
        min_gap=0.75,
        name="lamp panel reaches well forward of the stand",
    )
    ctx.expect_origin_gap(
        lamp_panel,
        stand,
        axis="z",
        min_gap=1.30,
        name="lamp panel is carried high above the weighted base",
    )

    rest_panel_pos = ctx.part_world_position(lamp_panel)
    with ctx.pose({stand_to_arm_1: 0.35}):
        shoulder_raised_panel_pos = ctx.part_world_position(lamp_panel)
    ctx.check(
        "shoulder hinge raises the lamp head",
        rest_panel_pos is not None
        and shoulder_raised_panel_pos is not None
        and shoulder_raised_panel_pos[2] > rest_panel_pos[2] + 0.08,
        details=f"rest={rest_panel_pos}, raised={shoulder_raised_panel_pos}",
    )

    with ctx.pose({arm_1_to_arm_2: 0.40, arm_2_to_arm_3: 0.25}):
        elbow_lift_panel_pos = ctx.part_world_position(lamp_panel)
    ctx.check(
        "elbow joints can lift the distal lamp assembly",
        rest_panel_pos is not None
        and elbow_lift_panel_pos is not None
        and elbow_lift_panel_pos[2] > rest_panel_pos[2] + 0.10,
        details=f"rest={rest_panel_pos}, elbow_lift={elbow_lift_panel_pos}",
    )

    rest_panel_aabb = ctx.part_element_world_aabb(lamp_panel, elem="panel_shell")
    with ctx.pose({arm_3_to_panel: -0.65}):
        tilted_panel_aabb = ctx.part_element_world_aabb(lamp_panel, elem="panel_shell")
    rest_x_extent = (
        rest_panel_aabb[1][0] - rest_panel_aabb[0][0] if rest_panel_aabb is not None else None
    )
    tilted_x_extent = (
        tilted_panel_aabb[1][0] - tilted_panel_aabb[0][0]
        if tilted_panel_aabb is not None
        else None
    )
    ctx.check(
        "lamp panel tilt changes the panel pitch",
        rest_x_extent is not None
        and tilted_x_extent is not None
        and tilted_x_extent > rest_x_extent + 0.04,
        details=f"rest_x_extent={rest_x_extent}, tilted_x_extent={tilted_x_extent}",
    )

    ctx.check(
        "all articulated lamp parts are present",
        all(part is not None for part in (stand, arm_1, arm_2, arm_3, lamp_panel)),
        details="One or more required articulated lamp parts could not be resolved.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
