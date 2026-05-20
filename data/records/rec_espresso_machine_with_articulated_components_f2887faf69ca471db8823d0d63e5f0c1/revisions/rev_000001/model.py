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


BODY_WIDTH = 0.34
BODY_DEPTH = 0.44
BODY_HEIGHT = 0.41
BODY_WALL = 0.008


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def _body_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, centered=(True, True, False))
        .cut(
            cq.Workplane("XY")
            .transformed(offset=(0.0, 0.0, BODY_WALL))
            .box(
                BODY_WIDTH - 2.0 * BODY_WALL,
                BODY_DEPTH - 2.0 * BODY_WALL,
                BODY_HEIGHT - BODY_WALL + 0.002,
                centered=(True, True, False),
            )
        )
        .faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .moveTo(0.0, 0.082)
        .rect(0.146, 0.102)
        .cutBlind(-0.095)
        .faces("<Y")
        .workplane(centerOption="CenterOfBoundBox")
        .moveTo(0.0, 0.115)
        .rect(0.205, 0.062)
        .cutBlind(-0.006)
    )
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prosumer_espresso_machine")

    model.material("steel", rgba=(0.79, 0.80, 0.81, 1.0))
    model.material("dark_steel", rgba=(0.32, 0.34, 0.36, 1.0))
    model.material("black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("plastic", rgba=(0.20, 0.20, 0.22, 1.0))
    model.material("brass", rgba=(0.73, 0.58, 0.29, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "espresso_body_shell"),
        material="steel",
        name="shell",
    )

    group_head = model.part("group_head")
    group_head.visual(
        Box((0.085, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH * 0.5 - 0.020, 0.273)),
        material="dark_steel",
        name="group_mount",
    )
    group_head.visual(
        Cylinder(radius=0.027, length=0.030),
        origin=Origin(
            xyz=(0.0, -BODY_DEPTH * 0.5 - 0.028, 0.259),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material="dark_steel",
        name="group_head",
    )

    steam_support = model.part("steam_support")
    steam_support.visual(
        Box((0.030, 0.040, 0.055)),
        origin=Origin(xyz=(BODY_WIDTH * 0.5 + 0.015, -BODY_DEPTH * 0.5 + 0.024, 0.286)),
        material="dark_steel",
        name="steam_mount",
    )

    cup_rest_mount_0 = model.part("cup_rest_mount_0")
    cup_rest_mount_0.visual(
        Box((0.086, 0.016, 0.014)),
        origin=Origin(xyz=(-0.062, -BODY_DEPTH * 0.5 - 0.008, 0.082)),
        material="dark_steel",
        name="cup_rest_pad_0",
    )

    cup_rest_mount_1 = model.part("cup_rest_mount_1")
    cup_rest_mount_1.visual(
        Box((0.096, 0.016, 0.014)),
        origin=Origin(xyz=(0.072, -BODY_DEPTH * 0.5 - 0.008, 0.132)),
        material="dark_steel",
        name="cup_rest_pad_1",
    )

    cup_rail = model.part("cup_rail")
    cup_rail.visual(
        Cylinder(radius=0.0045, length=0.042),
        origin=Origin(xyz=(-0.120, -0.175, BODY_HEIGHT + 0.020)),
        material="steel",
        name="rail_post_0",
    )
    cup_rail.visual(
        Cylinder(radius=0.0045, length=0.042),
        origin=Origin(xyz=(0.120, -0.175, BODY_HEIGHT + 0.020)),
        material="steel",
        name="rail_post_1",
    )
    cup_rail.visual(
        Cylinder(radius=0.0045, length=0.042),
        origin=Origin(xyz=(-0.120, -0.055, BODY_HEIGHT + 0.020)),
        material="steel",
        name="rail_post_2",
    )
    cup_rail.visual(
        Cylinder(radius=0.0045, length=0.042),
        origin=Origin(xyz=(0.120, -0.055, BODY_HEIGHT + 0.020)),
        material="steel",
        name="rail_post_3",
    )
    cup_rail.visual(
        Cylinder(radius=0.004, length=0.248),
        origin=Origin(
            xyz=(0.0, -0.175, BODY_HEIGHT + 0.041),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material="steel",
        name="rail_front",
    )
    cup_rail.visual(
        Cylinder(radius=0.004, length=0.120),
        origin=Origin(
            xyz=(-0.120, -0.115, BODY_HEIGHT + 0.041),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material="steel",
        name="rail_side_0",
    )
    cup_rail.visual(
        Cylinder(radius=0.004, length=0.120),
        origin=Origin(
            xyz=(0.120, -0.115, BODY_HEIGHT + 0.041),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material="steel",
        name="rail_side_1",
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.032, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material="brass",
        name="basket",
    )
    portafilter.visual(
        Cylinder(radius=0.036, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material="dark_steel",
        name="flange",
    )
    portafilter.visual(
        Box((0.018, 0.126, 0.022)),
        origin=Origin(xyz=(0.0, -0.079, -0.028), rpy=(0.25, 0.0, 0.0)),
        material="black",
        name="handle",
    )
    portafilter.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(
            xyz=(0.0, -0.139, -0.042),
            rpy=(-math.pi * 0.5 + 0.25, 0.0, 0.0),
        ),
        material="plastic",
        name="grip_end",
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="dark_steel",
        name="pivot_collar",
    )
    steam_wand.visual(
        Cylinder(radius=0.0065, length=0.050),
        origin=Origin(
            xyz=(0.0, -0.025, 0.025),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material="steel",
        name="upper_tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.0055, length=0.135),
        origin=Origin(xyz=(0.0, -0.050, -0.046)),
        material="steel",
        name="wand_tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=Origin(
            xyz=(0.0, -0.050, -0.122),
            rpy=(0.25, 0.0, 0.0),
        ),
        material="steel",
        name="wand_tip",
    )

    cup_rest_0 = model.part("cup_rest_0")
    cup_rest_0.visual(
        Cylinder(radius=0.005, length=0.084),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material="dark_steel",
        name="hinge_barrel",
    )
    cup_rest_0.visual(
        Box((0.082, 0.006, 0.052)),
        origin=Origin(xyz=(0.0, -0.003, 0.026)),
        material="dark_steel",
        name="rest_plate",
    )
    cup_rest_0.visual(
        Cylinder(radius=0.004, length=0.082),
        origin=Origin(
            xyz=(0.0, -0.006, 0.050),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material="steel",
        name="front_rod",
    )

    cup_rest_1 = model.part("cup_rest_1")
    cup_rest_1.visual(
        Cylinder(radius=0.005, length=0.094),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material="dark_steel",
        name="hinge_barrel",
    )
    cup_rest_1.visual(
        Box((0.094, 0.006, 0.060)),
        origin=Origin(xyz=(0.0, -0.003, 0.030)),
        material="dark_steel",
        name="rest_plate",
    )
    cup_rest_1.visual(
        Cylinder(radius=0.004, length=0.094),
        origin=Origin(
            xyz=(0.0, -0.006, 0.058),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material="steel",
        name="front_rod",
    )

    reservoir_lid = model.part("reservoir_lid")
    reservoir_lid.visual(
        Box((0.170, 0.125, 0.006)),
        origin=Origin(xyz=(0.0, -0.0625, 0.003)),
        material="steel",
        name="lid_panel",
    )
    reservoir_lid.visual(
        Cylinder(radius=0.004, length=0.040),
        origin=Origin(
            xyz=(-0.046, 0.0, 0.003),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material="dark_steel",
        name="hinge_knuckle_0",
    )
    reservoir_lid.visual(
        Cylinder(radius=0.004, length=0.040),
        origin=Origin(
            xyz=(0.046, 0.0, 0.003),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material="dark_steel",
        name="hinge_knuckle_1",
    )
    reservoir_lid.visual(
        Box((0.060, 0.012, 0.015)),
        origin=Origin(xyz=(0.0, -0.121, -0.0045)),
        material="plastic",
        name="lid_tab",
    )

    model.articulation(
        "body_to_group_head",
        ArticulationType.FIXED,
        parent=body,
        child=group_head,
        origin=Origin(),
    )
    model.articulation(
        "body_to_steam_support",
        ArticulationType.FIXED,
        parent=body,
        child=steam_support,
        origin=Origin(),
    )
    model.articulation(
        "body_to_cup_rest_mount_0",
        ArticulationType.FIXED,
        parent=body,
        child=cup_rest_mount_0,
        origin=Origin(),
    )
    model.articulation(
        "body_to_cup_rest_mount_1",
        ArticulationType.FIXED,
        parent=body,
        child=cup_rest_mount_1,
        origin=Origin(),
    )
    model.articulation(
        "body_to_cup_rail",
        ArticulationType.FIXED,
        parent=body,
        child=cup_rail,
        origin=Origin(),
    )

    model.articulation(
        "brew_lock",
        ArticulationType.REVOLUTE,
        parent=group_head,
        child=portafilter,
        origin=Origin(xyz=(0.0, -BODY_DEPTH * 0.5 - 0.028, 0.228)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-1.0,
            upper=0.2,
        ),
    )
    model.articulation(
        "steam_swing",
        ArticulationType.REVOLUTE,
        parent=steam_support,
        child=steam_wand,
        origin=Origin(xyz=(BODY_WIDTH * 0.5 + 0.038, -BODY_DEPTH * 0.5 + 0.024, 0.272)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=-1.55,
            upper=1.0,
        ),
    )
    model.articulation(
        "cup_rest_hinge_0",
        ArticulationType.REVOLUTE,
        parent=cup_rest_mount_0,
        child=cup_rest_0,
        origin=Origin(xyz=(-0.062, -BODY_DEPTH * 0.5 - 0.021, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=0.0,
            upper=1.52,
        ),
    )
    model.articulation(
        "cup_rest_hinge_1",
        ArticulationType.REVOLUTE,
        parent=cup_rest_mount_1,
        child=cup_rest_1,
        origin=Origin(xyz=(0.072, -BODY_DEPTH * 0.5 - 0.021, 0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=0.0,
            upper=1.52,
        ),
    )
    model.articulation(
        "reservoir_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=reservoir_lid,
        origin=Origin(xyz=(0.0, 0.141, BODY_HEIGHT)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.28,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cup_rail = object_model.get_part("cup_rail")
    group_head = object_model.get_part("group_head")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    cup_rest_0 = object_model.get_part("cup_rest_0")
    cup_rest_1 = object_model.get_part("cup_rest_1")
    reservoir_lid = object_model.get_part("reservoir_lid")

    brew_lock = object_model.get_articulation("brew_lock")
    steam_swing = object_model.get_articulation("steam_swing")
    cup_rest_hinge_0 = object_model.get_articulation("cup_rest_hinge_0")
    cup_rest_hinge_1 = object_model.get_articulation("cup_rest_hinge_1")
    reservoir_hinge = object_model.get_articulation("reservoir_hinge")

    ctx.allow_isolated_part(
        reservoir_lid,
        reason="The reservoir lid is intentionally carried by the rear hinge articulation; the simplified top opening does not create a compiler-detected support path in the closed pose.",
    )
    ctx.allow_isolated_part(
        cup_rail,
        reason="The welded cup rail is carried by four stanchions that meet the top deck with simplified line contact rather than a compiler-detected supported mount.",
    )

    with ctx.pose(
        {
            brew_lock: 0.0,
            steam_swing: 0.0,
            cup_rest_hinge_0: 0.0,
            cup_rest_hinge_1: 0.0,
            reservoir_hinge: 0.0,
        }
    ):
        ctx.expect_gap(
            body,
            cup_rest_0,
            axis="y",
            positive_elem="shell",
            negative_elem="rest_plate",
            min_gap=0.0,
            max_gap=0.030,
            name="lower cup rest stows just ahead of the front face",
        )
        ctx.expect_gap(
            body,
            cup_rest_1,
            axis="y",
            positive_elem="shell",
            negative_elem="rest_plate",
            min_gap=0.0,
            max_gap=0.030,
            name="upper cup rest stows just ahead of the front face",
        )
        ctx.expect_gap(
            reservoir_lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="shell",
            min_gap=0.0,
            max_gap=0.005,
            name="reservoir lid sits flush on the top deck",
        )
        ctx.expect_gap(
            group_head,
            portafilter,
            axis="z",
            positive_elem="group_head",
            negative_elem="flange",
            min_gap=0.0,
            max_gap=0.030,
            name="portafilter nests just below the group head",
        )

    with ctx.pose({cup_rest_hinge_0: 0.0, cup_rest_hinge_1: 0.0}):
        low_closed = _center_from_aabb(ctx.part_element_world_aabb(cup_rest_0, elem="rest_plate"))
        high_closed = _center_from_aabb(ctx.part_element_world_aabb(cup_rest_1, elem="rest_plate"))
    with ctx.pose({cup_rest_hinge_0: 1.40, cup_rest_hinge_1: 1.40}):
        low_open = _center_from_aabb(ctx.part_element_world_aabb(cup_rest_0, elem="rest_plate"))
        high_open = _center_from_aabb(ctx.part_element_world_aabb(cup_rest_1, elem="rest_plate"))

    ctx.check(
        "cup rests fold forward for cup support",
        low_closed is not None
        and low_open is not None
        and high_closed is not None
        and high_open is not None
        and low_open[1] < low_closed[1] - 0.020
        and high_open[1] < high_closed[1] - 0.020,
        details=f"low_closed={low_closed}, low_open={low_open}, high_closed={high_closed}, high_open={high_open}",
    )

    with ctx.pose({reservoir_hinge: 0.0}):
        lid_closed = _center_from_aabb(ctx.part_element_world_aabb(reservoir_lid, elem="lid_tab"))
    with ctx.pose({reservoir_hinge: 1.10}):
        lid_open = _center_from_aabb(ctx.part_element_world_aabb(reservoir_lid, elem="lid_tab"))

    ctx.check(
        "reservoir lid lifts upward from the rear hinge",
        lid_closed is not None
        and lid_open is not None
        and lid_open[2] > lid_closed[2] + 0.050,
        details=f"lid_closed={lid_closed}, lid_open={lid_open}",
    )

    with ctx.pose({steam_swing: 0.0}):
        wand_home = _center_from_aabb(ctx.part_element_world_aabb(steam_wand, elem="wand_tube"))
    with ctx.pose({steam_swing: -1.20}):
        wand_swung = _center_from_aabb(ctx.part_element_world_aabb(steam_wand, elem="wand_tube"))

    ctx.check(
        "steam wand swings sideways around its pivot",
        wand_home is not None
        and wand_swung is not None
        and abs(wand_swung[0] - wand_home[0]) > 0.030,
        details=f"wand_home={wand_home}, wand_swung={wand_swung}",
    )

    with ctx.pose({brew_lock: 0.0}):
        handle_locked = _center_from_aabb(ctx.part_element_world_aabb(portafilter, elem="handle"))
    with ctx.pose({brew_lock: -0.85}):
        handle_released = _center_from_aabb(ctx.part_element_world_aabb(portafilter, elem="handle"))

    ctx.check(
        "portafilter rotates about the vertical brew axis",
        handle_locked is not None
        and handle_released is not None
        and abs(handle_released[0] - handle_locked[0]) > 0.040,
        details=f"handle_locked={handle_locked}, handle_released={handle_released}",
    )

    return ctx.report()


object_model = build_object_model()
