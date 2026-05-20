from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


GRAPHITE = Material("satin_graphite", color=(0.06, 0.065, 0.07, 1.0))
BLACK = Material("matte_black", color=(0.005, 0.005, 0.006, 1.0))
ALUMINUM = Material("brushed_aluminum", color=(0.62, 0.64, 0.62, 1.0))
RUBBER = Material("dark_rubber", color=(0.015, 0.014, 0.013, 1.0))
SCREW = Material("blackened_screw", color=(0.01, 0.01, 0.012, 1.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="backplate_monitor_arm")

    shoulder_axis_x = 0.115
    shoulder_axis_z = 0.180
    shoulder_len = 0.360
    elbow_len = 0.320

    backplate = model.part("backplate")
    backplate.visual(
        Box((0.030, 0.240, 0.340)),
        origin=Origin(xyz=(-0.015, 0.0, 0.170)),
        material=GRAPHITE,
        name="wall_plate",
    )
    backplate.visual(
        Box((0.065, 0.100, 0.140)),
        origin=Origin(xyz=(0.0325, 0.0, shoulder_axis_z)),
        material=BLACK,
        name="pivot_standoff",
    )
    backplate.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(
            xyz=(0.090, 0.0, shoulder_axis_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=ALUMINUM,
        name="shoulder_pin",
    )
    for y in (-0.072, 0.072):
        for z in (0.095, 0.265):
            backplate.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(
                    xyz=(0.003, y, z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=SCREW,
                name=f"screw_{'upper' if z > 0.18 else 'lower'}_{'a' if y < 0 else 'b'}",
            )
    backplate.visual(
        Box((0.020, 0.180, 0.018)),
        origin=Origin(xyz=(-0.010, 0.0, 0.012)),
        material=RUBBER,
        name="bottom_pad",
    )

    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(
        Cylinder(radius=0.043, length=0.070),
        origin=Origin(),
        material=BLACK,
        name="root_hub",
    )
    shoulder_link.visual(
        Cylinder(radius=0.043, length=0.070),
        origin=Origin(xyz=(shoulder_len, 0.0, 0.0)),
        material=BLACK,
        name="end_hub",
    )
    for y in (-0.036, 0.036):
        shoulder_link.visual(
            Box((0.276, 0.018, 0.026)),
            origin=Origin(xyz=(shoulder_len / 2.0, y, 0.0)),
            material=ALUMINUM,
            name=f"rail_{'a' if y < 0 else 'b'}",
        )
    shoulder_link.visual(
        Box((0.288, 0.035, 0.018)),
        origin=Origin(xyz=(0.174, 0.0, -0.026)),
        material=RUBBER,
        name="cable_channel",
    )
    shoulder_link.visual(
        Box((0.035, 0.090, 0.030)),
        origin=Origin(xyz=(shoulder_len / 2.0, 0.0, -0.012)),
        material=BLACK,
        name="rail_clamp",
    )

    elbow_link = model.part("elbow_link")
    elbow_link.visual(
        Cylinder(radius=0.036, length=0.120),
        origin=Origin(),
        material=BLACK,
        name="root_hub",
    )
    elbow_link.visual(
        Cylinder(radius=0.036, length=0.120),
        origin=Origin(xyz=(elbow_len, 0.0, 0.0)),
        material=BLACK,
        name="end_hub",
    )
    for y in (-0.030, 0.030):
        elbow_link.visual(
            Box((0.256, 0.016, 0.024)),
            origin=Origin(xyz=(0.162, y, 0.055)),
            material=ALUMINUM,
            name=f"rail_{'a' if y < 0 else 'b'}",
        )
    elbow_link.visual(
        Box((0.256, 0.082, 0.014)),
        origin=Origin(xyz=(0.162, 0.0, 0.055)),
        material=RUBBER,
        name="cable_channel",
    )

    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.030, length=0.170),
        origin=Origin(),
        material=BLACK,
        name="swivel_post",
    )
    swivel.visual(
        Box((0.070, 0.050, 0.020)),
        origin=Origin(xyz=(0.035, 0.0, -0.080)),
        material=BLACK,
        name="lower_neck",
    )
    swivel.visual(
        Box((0.035, 0.100, 0.020)),
        origin=Origin(xyz=(0.075, 0.0, -0.080)),
        material=BLACK,
        name="yoke_bridge",
    )
    for y in (-0.048, 0.048):
        swivel.visual(
            Box((0.035, 0.014, 0.170)),
            origin=Origin(xyz=(0.100, y, 0.0)),
            material=BLACK,
            name=f"yoke_cheek_{'a' if y < 0 else 'b'}",
        )
    swivel.visual(
        Cylinder(radius=0.011, length=0.120),
        origin=Origin(
            xyz=(0.100, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=ALUMINUM,
        name="tilt_pin",
    )

    mount_head = model.part("mount_head")
    mount_head.visual(
        Cylinder(radius=0.025, length=0.060),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BLACK,
        name="tilt_lug",
    )
    mount_head.visual(
        Box((0.045, 0.060, 0.045)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material=BLACK,
        name="hinge_block",
    )
    mount_head.visual(
        Box((0.018, 0.145, 0.120)),
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        material=GRAPHITE,
        name="face_plate",
    )
    mount_head.visual(
        Box((0.010, 0.084, 0.060)),
        origin=Origin(xyz=(0.0795, 0.0, 0.0)),
        material=BLACK,
        name="raised_boss",
    )
    for y in (-0.050, 0.050):
        for z in (-0.035, 0.035):
            mount_head.visual(
                Cylinder(radius=0.0065, length=0.006),
                origin=Origin(
                    xyz=(0.077, y, z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=SCREW,
                name=f"vesa_socket_{'lower' if z < 0 else 'upper'}_{'a' if y < 0 else 'b'}",
            )

    model.articulation(
        "backplate_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=shoulder_link,
        origin=Origin(xyz=(shoulder_axis_x, 0.0, shoulder_axis_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=-1.75, upper=1.75),
    )
    model.articulation(
        "shoulder_to_elbow",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=elbow_link,
        origin=Origin(xyz=(shoulder_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "elbow_to_swivel",
        ArticulationType.REVOLUTE,
        parent=elbow_link,
        child=swivel,
        origin=Origin(xyz=(elbow_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "swivel_to_head",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=mount_head,
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    shoulder = object_model.get_part("shoulder_link")
    elbow = object_model.get_part("elbow_link")
    swivel = object_model.get_part("swivel")
    head = object_model.get_part("mount_head")

    shoulder_joint = object_model.get_articulation("backplate_to_shoulder")
    elbow_joint = object_model.get_articulation("shoulder_to_elbow")
    swivel_joint = object_model.get_articulation("elbow_to_swivel")
    tilt_joint = object_model.get_articulation("swivel_to_head")

    ctx.check(
        "four revolute mechanisms",
        len(object_model.articulations) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"articulations={object_model.articulations}",
    )
    ctx.check(
        "single grounded backplate root",
        [p.name for p in object_model.root_parts()] == ["backplate"],
        details=f"roots={[p.name for p in object_model.root_parts()]}",
    )

    ctx.allow_overlap(
        backplate,
        shoulder,
        elem_a="shoulder_pin",
        elem_b="root_hub",
        reason="The fixed shoulder pin is intentionally captured inside the first arm hub.",
    )
    ctx.expect_within(
        backplate,
        shoulder,
        axes="yz",
        inner_elem="shoulder_pin",
        outer_elem="root_hub",
        margin=0.002,
        name="shoulder pin is centered in hub",
    )
    ctx.expect_overlap(
        backplate,
        shoulder,
        axes="x",
        elem_a="shoulder_pin",
        elem_b="root_hub",
        min_overlap=0.030,
        name="shoulder pin remains inserted",
    )

    ctx.allow_overlap(
        shoulder,
        elbow,
        elem_a="end_hub",
        elem_b="root_hub",
        reason="The elbow revolute is represented by nested coaxial hinge hubs.",
    )
    ctx.expect_overlap(
        shoulder,
        elbow,
        axes="xyz",
        elem_a="end_hub",
        elem_b="root_hub",
        min_overlap=0.030,
        name="elbow hinge hubs are coaxial",
    )

    ctx.allow_overlap(
        elbow,
        swivel,
        elem_a="end_hub",
        elem_b="swivel_post",
        reason="The vertical swivel post is seated inside the end-arm bearing.",
    )
    ctx.expect_overlap(
        elbow,
        swivel,
        axes="xyz",
        elem_a="end_hub",
        elem_b="swivel_post",
        min_overlap=0.030,
        name="swivel post is retained in end hub",
    )

    ctx.allow_overlap(
        swivel,
        head,
        elem_a="tilt_pin",
        elem_b="tilt_lug",
        reason="The horizontal tilt pin intentionally passes through the head lug.",
    )
    ctx.expect_within(
        swivel,
        head,
        axes="xz",
        inner_elem="tilt_pin",
        outer_elem="tilt_lug",
        margin=0.002,
        name="tilt pin is centered in lug",
    )
    ctx.expect_overlap(
        swivel,
        head,
        axes="y",
        elem_a="tilt_pin",
        elem_b="tilt_lug",
        min_overlap=0.050,
        name="tilt pin spans the lug",
    )

    rest_elbow_pos = ctx.part_world_position(elbow)
    with ctx.pose({shoulder_joint: 0.85}):
        swung_elbow_pos = ctx.part_world_position(elbow)
    ctx.check(
        "first arm folds about vertical axis",
        rest_elbow_pos is not None
        and swung_elbow_pos is not None
        and swung_elbow_pos[1] > rest_elbow_pos[1] + 0.20,
        details=f"rest={rest_elbow_pos}, swung={swung_elbow_pos}",
    )

    rest_swivel_pos = ctx.part_world_position(swivel)
    with ctx.pose({elbow_joint: -0.90}):
        folded_swivel_pos = ctx.part_world_position(swivel)
    ctx.check(
        "second arm folds independently",
        rest_swivel_pos is not None
        and folded_swivel_pos is not None
        and folded_swivel_pos[1] < rest_swivel_pos[1] - 0.18,
        details=f"rest={rest_swivel_pos}, folded={folded_swivel_pos}",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({swivel_joint: 1.00}):
        swiveled_head_pos = ctx.part_world_position(head)
    ctx.check(
        "mount head swivels vertically",
        rest_head_pos is not None
        and swiveled_head_pos is not None
        and swiveled_head_pos[1] > rest_head_pos[1] + 0.075,
        details=f"rest={rest_head_pos}, swiveled={swiveled_head_pos}",
    )

    rest_plate_aabb = ctx.part_element_world_aabb(head, elem="face_plate")
    with ctx.pose({tilt_joint: 0.45}):
        tilted_plate_aabb = ctx.part_element_world_aabb(head, elem="face_plate")
    rest_plate_dx = (
        rest_plate_aabb[1][0] - rest_plate_aabb[0][0] if rest_plate_aabb is not None else 0.0
    )
    tilted_plate_dx = (
        tilted_plate_aabb[1][0] - tilted_plate_aabb[0][0] if tilted_plate_aabb is not None else 0.0
    )
    ctx.check(
        "face plate tilts about horizontal hinge",
        tilted_plate_dx > rest_plate_dx + 0.030,
        details=f"rest_dx={rest_plate_dx}, tilted_dx={tilted_plate_dx}",
    )

    return ctx.report()


object_model = build_object_model()
