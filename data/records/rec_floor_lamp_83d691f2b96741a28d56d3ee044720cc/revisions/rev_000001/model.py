from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BASE_RADIUS = 0.180
BASE_THICKNESS = 0.036
BASE_PLINTH_RADIUS = 0.102
BASE_PLINTH_THICKNESS = 0.010
POST_RADIUS = 0.016
POST_HEIGHT = 1.530
POST_COLLAR_RADIUS = 0.034
POST_COLLAR_HEIGHT = 0.030
ARM_THRUST_RING_RADIUS = 0.027
ARM_THRUST_RING_HEIGHT = 0.006

SLEEVE_OUTER_RADIUS = 0.030
SLEEVE_INNER_RADIUS = POST_RADIUS + 0.003
SLEEVE_HEIGHT = 0.060
ARM_RADIUS = 0.010
ARM_ROOT_BLOCK_X = 0.030
ARM_ROOT_BLOCK_Y = 0.024
ARM_ROOT_BLOCK_Z = 0.020
ARM_ROOT_OFFSET_X = SLEEVE_OUTER_RADIUS + 0.014
ARM_ROOT_OFFSET_Z = 0.008
YOKE_GAP_CENTER = 0.013
YOKE_CHEEK_THICKNESS = 0.004
YOKE_CHEEK_X = 0.018
YOKE_CHEEK_Z = 0.022

SHADE_TRUNNION_RADIUS = 0.0055
SHADE_TRUNNION_LENGTH = 0.022
SHADE_NECK_RADIUS = 0.0085
SHADE_NECK_LENGTH = 0.030
SHADE_COLLAR_RADIUS = 0.026
SHADE_COLLAR_LENGTH = 0.014
SHADE_SHELL_LENGTH = 0.158
SHADE_SHELL_NECK_RADIUS = 0.023
SHADE_SHELL_MOUTH_RADIUS = 0.066
SHADE_SHELL_WALL = 0.0018
SHADE_SHELL_OFFSET_X = 0.036

ARM_SPECS = (
    {"height": 0.980, "length": 0.320, "arm_pitch_deg": 8.0, "yaw_deg": 32.0, "shade_pitch_deg": 18.0},
    {"height": 1.115, "length": 0.365, "arm_pitch_deg": 12.0, "yaw_deg": 118.0, "shade_pitch_deg": 12.0},
    {"height": 1.275, "length": 0.405, "arm_pitch_deg": 16.0, "yaw_deg": -108.0, "shade_pitch_deg": 20.0},
    {"height": 1.420, "length": 0.445, "arm_pitch_deg": 20.0, "yaw_deg": -24.0, "shade_pitch_deg": 15.0},
)


def _make_sleeve_mesh() -> object:
    outer = cq.Workplane("XY").circle(SLEEVE_OUTER_RADIUS).extrude(SLEEVE_HEIGHT)
    inner = (
        cq.Workplane("XY")
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(SLEEVE_HEIGHT + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    return outer.cut(inner).translate((0.0, 0.0, -SLEEVE_HEIGHT / 2.0))


def _make_shade_shell() -> object:
    outer = (
        cq.Workplane("XY")
        .circle(SHADE_SHELL_NECK_RADIUS)
        .workplane(offset=SHADE_SHELL_LENGTH)
        .circle(SHADE_SHELL_MOUTH_RADIUS)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .circle(SHADE_SHELL_NECK_RADIUS - SHADE_SHELL_WALL)
        .workplane(offset=SHADE_SHELL_LENGTH)
        .circle(SHADE_SHELL_MOUTH_RADIUS - SHADE_SHELL_WALL)
        .loft(combine=True)
    )
    return outer.cut(inner)


def _add_branch(
    model: ArticulatedObject,
    *,
    index: int,
    spec: dict[str, float],
    stand,
    sleeve_mesh,
    shade_shell_mesh,
) -> None:
    arm = model.part(f"arm_{index}")
    shade = model.part(f"shade_{index}")

    arm.visual(sleeve_mesh, material="arm_brass", name="sleeve")
    arm.visual(
        Box((ARM_ROOT_BLOCK_X, ARM_ROOT_BLOCK_Y, ARM_ROOT_BLOCK_Z)),
        origin=Origin(xyz=(ARM_ROOT_OFFSET_X, 0.0, ARM_ROOT_OFFSET_Z)),
        material="arm_brass",
        name="root_block",
    )

    arm_pitch = math.radians(spec["arm_pitch_deg"])
    arm_vec_x = math.cos(arm_pitch)
    arm_vec_z = math.sin(arm_pitch)
    tube_root_x = SLEEVE_OUTER_RADIUS + 0.026
    tube_root_z = 0.010
    arm.visual(
        Cylinder(radius=ARM_RADIUS, length=spec["length"]),
        origin=Origin(
            xyz=(
                tube_root_x + arm_vec_x * spec["length"] / 2.0,
                0.0,
                tube_root_z + arm_vec_z * spec["length"] / 2.0,
            ),
            rpy=(0.0, math.pi / 2.0 - arm_pitch, 0.0),
        ),
        material="arm_brass",
        name="tube",
    )

    hinge_x = tube_root_x + arm_vec_x * spec["length"] + 0.012
    hinge_z = tube_root_z + arm_vec_z * spec["length"]
    pivot_x = hinge_x + 0.003
    arm.visual(
        Box((0.026, 0.026, 0.018)),
        origin=Origin(xyz=(hinge_x - 0.018, 0.0, hinge_z)),
        material="arm_brass",
        name="tip_block",
    )
    for cheek_index, cheek_y in enumerate((-YOKE_GAP_CENTER, YOKE_GAP_CENTER)):
        arm.visual(
            Box((YOKE_CHEEK_X, YOKE_CHEEK_THICKNESS, YOKE_CHEEK_Z)),
            origin=Origin(xyz=(pivot_x, cheek_y, hinge_z)),
            material="arm_brass",
            name=f"cheek_{cheek_index}",
        )

    shade.visual(
        Cylinder(radius=SHADE_TRUNNION_RADIUS, length=SHADE_TRUNNION_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="arm_brass",
        name="trunnion",
    )
    shade.visual(
        Box((0.014, 0.012, 0.012)),
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
        material="arm_brass",
        name="hinge_block",
    )
    shade.visual(
        Cylinder(radius=SHADE_NECK_RADIUS, length=SHADE_NECK_LENGTH),
        origin=Origin(xyz=(0.021, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="arm_brass",
        name="neck",
    )
    shade.visual(
        Cylinder(radius=SHADE_COLLAR_RADIUS, length=SHADE_COLLAR_LENGTH),
        origin=Origin(xyz=(0.042, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="socket_black",
        name="collar",
    )
    shade.visual(
        shade_shell_mesh,
        origin=Origin(xyz=(SHADE_SHELL_OFFSET_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="shade_linen",
        name="shell",
    )

    model.articulation(
        f"stand_to_arm_{index}",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, spec["height"]), rpy=(0.0, 0.0, math.radians(spec["yaw_deg"]))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-1.20, upper=1.20),
    )
    model.articulation(
        f"arm_{index}_to_shade_{index}",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=shade,
        origin=Origin(xyz=(pivot_x, 0.0, hinge_z), rpy=(0.0, math.radians(spec["shade_pitch_deg"]), 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.4, lower=-0.55, upper=0.85),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tree_floor_lamp")

    model.material("stand_black", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("arm_brass", rgba=(0.69, 0.60, 0.34, 1.0))
    model.material("shade_linen", rgba=(0.93, 0.90, 0.82, 1.0))
    model.material("socket_black", rgba=(0.17, 0.15, 0.14, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="stand_black",
        name="base_disk",
    )
    stand.visual(
        Cylinder(radius=BASE_PLINTH_RADIUS, length=BASE_PLINTH_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + BASE_PLINTH_THICKNESS / 2.0)),
        material="stand_black",
        name="base_plinth",
    )
    stand.visual(
        Cylinder(radius=POST_COLLAR_RADIUS, length=POST_COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + POST_COLLAR_HEIGHT / 2.0)),
        material="stand_black",
        name="post_collar",
    )
    stand.visual(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + POST_HEIGHT / 2.0)),
        material="stand_black",
        name="post",
    )
    stand.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + POST_HEIGHT + 0.010)),
        material="arm_brass",
        name="top_cap",
    )
    for index, spec in enumerate(ARM_SPECS):
        stand.visual(
            Cylinder(radius=ARM_THRUST_RING_RADIUS, length=ARM_THRUST_RING_HEIGHT),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    spec["height"] - SLEEVE_HEIGHT / 2.0 - ARM_THRUST_RING_HEIGHT / 2.0,
                )
            ),
            material="stand_black",
            name=f"thrust_ring_{index}",
        )

    sleeve_mesh = mesh_from_cadquery(_make_sleeve_mesh(), "branch_sleeve")
    shade_shell_mesh = mesh_from_cadquery(_make_shade_shell(), "cone_shade_shell")

    for index, spec in enumerate(ARM_SPECS):
        _add_branch(
            model,
            index=index,
            spec=spec,
            stand=stand,
            sleeve_mesh=sleeve_mesh,
            shade_shell_mesh=shade_shell_mesh,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    for index in range(len(ARM_SPECS)):
        arm_joint = object_model.get_articulation(f"stand_to_arm_{index}")
        shade_joint = object_model.get_articulation(f"arm_{index}_to_shade_{index}")
        shade = object_model.get_part(f"shade_{index}")

        rest_pos = ctx.part_world_position(shade)
        with ctx.pose({arm_joint: arm_joint.motion_limits.upper * 0.75}):
            swept_pos = ctx.part_world_position(shade)
        ctx.check(
            f"arm_{index} swings its shade around the post",
            rest_pos is not None
            and swept_pos is not None
            and math.hypot(swept_pos[0] - rest_pos[0], swept_pos[1] - rest_pos[1]) > 0.12,
            details=f"rest={rest_pos}, swept={swept_pos}",
        )

        with ctx.pose({shade_joint: shade_joint.motion_limits.lower}):
            raised_aabb = ctx.part_element_world_aabb(shade, elem="shell")
        with ctx.pose({shade_joint: shade_joint.motion_limits.upper}):
            lowered_aabb = ctx.part_element_world_aabb(shade, elem="shell")
        ctx.check(
            f"shade_{index} tilts downward at its hinge",
            raised_aabb is not None
            and lowered_aabb is not None
            and lowered_aabb[0][2] < raised_aabb[0][2] - 0.03,
            details=f"raised={raised_aabb}, lowered={lowered_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
