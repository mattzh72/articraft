from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BARREL_RADIUS = 0.009
BARREL_LENGTH = 0.032
EAR_Y_OFFSET = 0.019
EAR_THICKNESS = 0.006
EAR_WIDTH = 0.024
EAR_HEIGHT = 0.052


def add_clevis(part, *, joint_z: float, material: str, prefix: str) -> None:
    part.visual(
        Box((0.028, 0.030, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, joint_z - 0.034)),
        material=material,
        name=f"{prefix}_head_block",
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        part.visual(
            Box((EAR_WIDTH, EAR_THICKNESS, EAR_HEIGHT)),
            origin=Origin(xyz=(0.0, sign * EAR_Y_OFFSET, joint_z)),
            material=material,
            name=f"{prefix}_{side}_ear",
        )


def add_arm_link(
    part,
    *,
    length: float,
    material: str,
    prefix: str,
    tube_radius: float = 0.012,
) -> None:
    main_len = length - 0.120
    part.visual(
        Cylinder(radius=BARREL_RADIUS, length=BARREL_LENGTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=f"{prefix}_lower_barrel",
    )
    part.visual(
        Cylinder(radius=tube_radius + 0.003, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=material,
        name=f"{prefix}_lower_collar",
    )
    part.visual(
        Cylinder(radius=tube_radius, length=main_len),
        origin=Origin(xyz=(0.0, 0.0, 0.055 + main_len / 2.0)),
        material=material,
        name=f"{prefix}_tube",
    )
    part.visual(
        Cylinder(radius=tube_radius + 0.004, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, length - 0.048)),
        material=material,
        name=f"{prefix}_upper_collar",
    )
    add_clevis(part, joint_z=length, material=material, prefix=f"{prefix}_upper")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_floor_lamp")

    base_finish = model.material("base_finish", color=(0.16, 0.17, 0.19, 1.0))
    arm_finish = model.material("arm_finish", color=(0.71, 0.63, 0.47, 1.0))
    shade_finish = model.material("shade_finish", color=(0.90, 0.89, 0.84, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.160, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=base_finish,
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=base_finish,
        name="base_hub",
    )
    base.visual(
        Cylinder(radius=0.016, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.388)),
        material=arm_finish,
        name="center_stem",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        material=arm_finish,
        name="shoulder_collar",
    )
    add_clevis(base, joint_z=0.770, material=arm_finish, prefix="shoulder")

    lower_arm = model.part("lower_arm")
    add_arm_link(lower_arm, length=0.380, material=arm_finish, prefix="lower_arm")

    middle_arm = model.part("middle_arm")
    add_arm_link(middle_arm, length=0.340, material=arm_finish, prefix="middle_arm")

    upper_arm = model.part("upper_arm")
    add_arm_link(upper_arm, length=0.270, material=arm_finish, prefix="upper_arm")

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=BARREL_RADIUS, length=BARREL_LENGTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=arm_finish,
        name="shade_barrel",
    )
    shade.visual(
        Cylinder(radius=0.013, length=0.070),
        origin=Origin(xyz=(0.035, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=arm_finish,
        name="shade_neck",
    )
    shade.visual(
        Cylinder(radius=0.032, length=0.045),
        origin=Origin(xyz=(0.078, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=arm_finish,
        name="shade_socket",
    )
    shade.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=Origin(xyz=(0.074, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=shade_finish,
        name="shade_rear_bezel",
    )
    shade.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.040, -0.050),
                    (0.052, -0.028),
                    (0.071, 0.010),
                    (0.088, 0.065),
                ],
                [
                    (0.033, -0.044),
                    (0.044, -0.024),
                    (0.061, 0.010),
                    (0.080, 0.058),
                ],
                segments=48,
                start_cap="flat",
                end_cap="flat",
            ),
            "reading_lamp_shade_shell",
        ),
        origin=Origin(xyz=(0.124, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=shade_finish,
        name="shade_shell",
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.770)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=-1.10,
            upper=1.10,
        ),
    )
    model.articulation(
        "lower_to_middle_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=middle_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.4,
            lower=-1.55,
            upper=1.55,
        ),
    )
    model.articulation(
        "middle_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=middle_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=-1.45,
            upper=1.45,
        ),
    )
    model.articulation(
        "upper_arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-0.65,
            upper=1.10,
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

    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    middle_arm = object_model.get_part("middle_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")

    shoulder = object_model.get_articulation("base_to_lower_arm")
    elbow_1 = object_model.get_articulation("lower_to_middle_arm")
    elbow_2 = object_model.get_articulation("middle_to_upper_arm")
    shade_tilt = object_model.get_articulation("upper_arm_to_shade")

    ctx.expect_origin_gap(
        lower_arm,
        base,
        axis="z",
        min_gap=0.73,
        name="lower arm shoulder sits high above the floor base",
    )
    ctx.expect_origin_gap(
        middle_arm,
        lower_arm,
        axis="z",
        min_gap=0.34,
        name="middle arm stacks above the lower arm at rest",
    )
    ctx.expect_origin_gap(
        upper_arm,
        middle_arm,
        axis="z",
        min_gap=0.30,
        name="upper arm stacks above the middle arm at rest",
    )
    ctx.expect_origin_gap(
        shade,
        upper_arm,
        axis="z",
        min_gap=0.22,
        name="shade joint sits at the tip of the upper arm",
    )

    rest_shade_pos = ctx.part_world_position(shade)
    with ctx.pose({shoulder: 0.55}):
        shoulder_pose_shade = ctx.part_world_position(shade)
    ctx.check(
        "shoulder joint pitches the lamp forward",
        rest_shade_pos is not None
        and shoulder_pose_shade is not None
        and shoulder_pose_shade[0] > rest_shade_pos[0] + 0.18,
        details=f"rest={rest_shade_pos}, shoulder_pose={shoulder_pose_shade}",
    )

    with ctx.pose({elbow_1: 0.70}):
        elbow_1_pose_shade = ctx.part_world_position(shade)
    ctx.check(
        "first elbow advances the upper chain",
        rest_shade_pos is not None
        and elbow_1_pose_shade is not None
        and elbow_1_pose_shade[0] > rest_shade_pos[0] + 0.12,
        details=f"rest={rest_shade_pos}, elbow_pose={elbow_1_pose_shade}",
    )

    with ctx.pose({elbow_2: 0.65}):
        elbow_2_pose_shade = ctx.part_world_position(shade)
    ctx.check(
        "second elbow advances the lamp head",
        rest_shade_pos is not None
        and elbow_2_pose_shade is not None
        and elbow_2_pose_shade[0] > rest_shade_pos[0] + 0.08,
        details=f"rest={rest_shade_pos}, elbow_pose={elbow_2_pose_shade}",
    )

    rest_shade_shell = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({shade_tilt: 0.80}):
        tilted_shade_shell = ctx.part_element_world_aabb(shade, elem="shade_shell")
    ctx.check(
        "shade joint tilts the head downward",
        rest_shade_shell is not None
        and tilted_shade_shell is not None
        and tilted_shade_shell[0][2] < rest_shade_shell[0][2] - 0.03,
        details=f"rest={rest_shade_shell}, tilted={tilted_shade_shell}",
    )

    with ctx.pose(
        {
            shoulder: 0.48,
            elbow_1: 0.58,
            elbow_2: 0.42,
            shade_tilt: 0.78,
        }
    ):
        reading_pose_shade = ctx.part_world_position(shade)
    ctx.check(
        "reading pose places the shade forward of the base",
        reading_pose_shade is not None
        and reading_pose_shade[0] > 0.35
        and reading_pose_shade[2] > 1.00,
        details=f"reading_pose={reading_pose_shade}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
