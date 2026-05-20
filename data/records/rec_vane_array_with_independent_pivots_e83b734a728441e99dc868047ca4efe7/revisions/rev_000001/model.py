from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


VANE_COUNT = 6
VANE_AXIS_Z = tuple(0.10 + 0.072 * i for i in range(VANE_COUNT))
FRAME_Y = 0.02
FRAME_OUTER_WIDTH = 0.74
FRAME_HEIGHT = 0.56
SIDE_RAIL_THICKNESS = 0.07
RAIL_DEPTH = 0.08
BLADE_SPAN = 0.54
BLADE_CHORD = 0.075
BLADE_THICKNESS = 0.012
BLADE_REST_ROLL = 0.35
BOSS_RADIUS = 0.014
BOSS_LENGTH = 0.050
BOSS_CENTER_X = BLADE_SPAN / 2.0 + BOSS_LENGTH / 2.0 - 0.012


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_louver_module")

    frame_mat = model.material("powder_coated_frame", rgba=(0.12, 0.13, 0.13, 1.0))
    flange_mat = model.material("matte_wall_flange", rgba=(0.18, 0.19, 0.18, 1.0))
    screw_mat = model.material("dark_screw_heads", rgba=(0.05, 0.05, 0.045, 1.0))
    vane_mat = model.material("satin_aluminum_vanes", rgba=(0.72, 0.74, 0.72, 1.0))
    boss_mat = model.material("dark_pivot_bosses", rgba=(0.08, 0.085, 0.08, 1.0))

    frame = model.part("frame")

    side_center_x = FRAME_OUTER_WIDTH / 2.0 - SIDE_RAIL_THICKNESS / 2.0
    frame.visual(
        Box((SIDE_RAIL_THICKNESS, RAIL_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(-side_center_x, FRAME_Y, FRAME_HEIGHT / 2.0)),
        material=frame_mat,
        name="side_rail_0",
    )
    frame.visual(
        Box((SIDE_RAIL_THICKNESS, RAIL_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(side_center_x, FRAME_Y, FRAME_HEIGHT / 2.0)),
        material=frame_mat,
        name="side_rail_1",
    )
    frame.visual(
        Box((FRAME_OUTER_WIDTH, RAIL_DEPTH, 0.055)),
        origin=Origin(xyz=(0.0, FRAME_Y, 0.0275)),
        material=frame_mat,
        name="bottom_rail",
    )
    frame.visual(
        Box((FRAME_OUTER_WIDTH, RAIL_DEPTH, 0.055)),
        origin=Origin(xyz=(0.0, FRAME_Y, FRAME_HEIGHT - 0.0275)),
        material=frame_mat,
        name="top_rail",
    )

    # A slightly larger rear flange reads as the grounded side-wall mounting
    # frame while leaving the louver opening physically open.
    flange_depth = 0.030
    flange_y = -0.033
    flange_width = 0.81
    flange_height = 0.63
    frame.visual(
        Box((0.065, flange_depth, flange_height)),
        origin=Origin(xyz=(-flange_width / 2.0 + 0.0325, flange_y, FRAME_HEIGHT / 2.0)),
        material=flange_mat,
        name="rear_flange_0",
    )
    frame.visual(
        Box((0.065, flange_depth, flange_height)),
        origin=Origin(xyz=(flange_width / 2.0 - 0.0325, flange_y, FRAME_HEIGHT / 2.0)),
        material=flange_mat,
        name="rear_flange_1",
    )
    frame.visual(
        Box((flange_width, flange_depth, 0.065)),
        origin=Origin(xyz=(0.0, flange_y, -0.035 + 0.0325)),
        material=flange_mat,
        name="rear_flange_2",
    )
    frame.visual(
        Box((flange_width, flange_depth, 0.065)),
        origin=Origin(xyz=(0.0, flange_y, FRAME_HEIGHT + 0.035 - 0.0325)),
        material=flange_mat,
        name="rear_flange_3",
    )

    screw_positions = (
        (-0.325, FRAME_Y + RAIL_DEPTH / 2.0 + 0.002, 0.070),
        (0.325, FRAME_Y + RAIL_DEPTH / 2.0 + 0.002, 0.070),
        (-0.325, FRAME_Y + RAIL_DEPTH / 2.0 + 0.002, FRAME_HEIGHT - 0.070),
        (0.325, FRAME_Y + RAIL_DEPTH / 2.0 + 0.002, FRAME_HEIGHT - 0.070),
    )
    for index, xyz in enumerate(screw_positions):
        frame.visual(
            Cylinder(radius=0.017, length=0.006),
            origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_mat,
            name=f"screw_{index}",
        )

    edge_offset_y = (BLADE_CHORD / 2.0 - BLADE_THICKNESS * 0.25) * math.cos(BLADE_REST_ROLL)
    edge_offset_z = (BLADE_CHORD / 2.0 - BLADE_THICKNESS * 0.25) * math.sin(BLADE_REST_ROLL)

    for index, z_axis in enumerate(VANE_AXIS_Z):
        vane = model.part(f"vane_{index}")
        vane.visual(
            Box((BLADE_SPAN, BLADE_CHORD, BLADE_THICKNESS)),
            origin=Origin(rpy=(BLADE_REST_ROLL, 0.0, 0.0)),
            material=vane_mat,
            name="blade",
        )
        vane.visual(
            Cylinder(radius=BLADE_THICKNESS / 2.0, length=BLADE_SPAN),
            origin=Origin(
                xyz=(0.0, edge_offset_y, edge_offset_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=vane_mat,
            name="front_edge",
        )
        vane.visual(
            Cylinder(radius=BLADE_THICKNESS / 2.0, length=BLADE_SPAN),
            origin=Origin(
                xyz=(0.0, -edge_offset_y, -edge_offset_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=vane_mat,
            name="rear_edge",
        )
        vane.visual(
            Cylinder(radius=BOSS_RADIUS, length=BOSS_LENGTH),
            origin=Origin(
                xyz=(-BOSS_CENTER_X, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=boss_mat,
            name="boss_0",
        )
        vane.visual(
            Cylinder(radius=BOSS_RADIUS, length=BOSS_LENGTH),
            origin=Origin(
                xyz=(BOSS_CENTER_X, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=boss_mat,
            name="boss_1",
        )

        model.articulation(
            f"frame_to_vane_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(xyz=(0.0, FRAME_Y, z_axis)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=1.4, lower=-0.45, upper=0.75),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    joints = [object_model.get_articulation(f"frame_to_vane_{i}") for i in range(VANE_COUNT)]

    ctx.check(
        "separate revolute vane joints",
        len(joints) == VANE_COUNT
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={joints}",
    )

    for index, joint in enumerate(joints):
        vane = object_model.get_part(f"vane_{index}")
        ctx.allow_overlap(
            frame,
            vane,
            elem_a="side_rail_0",
            elem_b="boss_0",
            reason="The vane end boss is intentionally captured a few millimeters inside the side rail bearing pocket.",
        )
        ctx.allow_overlap(
            frame,
            vane,
            elem_a="side_rail_1",
            elem_b="boss_1",
            reason="The vane end boss is intentionally captured a few millimeters inside the side rail bearing pocket.",
        )
        ctx.expect_gap(
            vane,
            frame,
            axis="x",
            positive_elem="boss_0",
            negative_elem="side_rail_0",
            max_penetration=0.012,
            max_gap=0.001,
            name=f"vane_{index} left boss is seated in rail",
        )
        ctx.expect_gap(
            frame,
            vane,
            axis="x",
            positive_elem="side_rail_1",
            negative_elem="boss_1",
            max_penetration=0.012,
            max_gap=0.001,
            name=f"vane_{index} right boss is seated in rail",
        )
        ctx.expect_overlap(
            vane,
            frame,
            axes="yz",
            elem_a="boss_0",
            elem_b="side_rail_0",
            min_overlap=0.020,
            name=f"vane_{index} left boss shares bearing area",
        )
        ctx.expect_overlap(
            vane,
            frame,
            axes="yz",
            elem_a="boss_1",
            elem_b="side_rail_1",
            min_overlap=0.020,
            name=f"vane_{index} right boss shares bearing area",
        )

        ctx.check(
            f"vane_{index} has bounded louver motion",
            joint.motion_limits is not None
            and joint.motion_limits.lower < 0.0
            and joint.motion_limits.upper > 0.5,
            details=str(joint.motion_limits),
        )

    first_joint = joints[0]
    first_vane = object_model.get_part("vane_0")
    rest_aabb = ctx.part_element_world_aabb(first_vane, elem="blade")
    with ctx.pose({first_joint: first_joint.motion_limits.upper}):
        open_aabb = ctx.part_element_world_aabb(first_vane, elem="blade")
    if rest_aabb is not None and open_aabb is not None:
        rest_z_span = rest_aabb[1][2] - rest_aabb[0][2]
        open_z_span = open_aabb[1][2] - open_aabb[0][2]
        ctx.check(
            "vane rotates about supported horizontal axis",
            open_z_span > rest_z_span + 0.030,
            details=f"rest_z={rest_z_span:.3f}, open_z={open_z_span:.3f}",
        )
    else:
        ctx.fail("vane rotates about supported horizontal axis", "blade AABB unavailable")

    return ctx.report()


object_model = build_object_model()
