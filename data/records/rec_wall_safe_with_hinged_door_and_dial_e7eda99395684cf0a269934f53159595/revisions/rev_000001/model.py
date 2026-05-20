from __future__ import annotations

from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="document_wall_safe")

    gunmetal = model.material("gunmetal_powdercoat", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    worn_edge = model.material("worn_edge_steel", rgba=(0.34, 0.36, 0.36, 1.0))
    black = model.material("matte_black", rgba=(0.01, 0.01, 0.012, 1.0))
    brass = model.material("brushed_brass", rgba=(0.72, 0.56, 0.28, 1.0))
    white = model.material("engraved_white", rgba=(0.92, 0.92, 0.86, 1.0))

    # A tall wall-safe body sized for documents: the open front is wrapped by a
    # separate fixed flange/sleeve, rather than being fused into the cabinet.
    safe_body = model.part("safe_body")
    safe_body.visual(
        Box((0.025, 0.42, 1.05)),
        origin=Origin(xyz=(-0.2675, 0.0, 0.525)),
        material=dark_steel,
        name="rear_wall",
    )
    safe_body.visual(
        Box((0.28, 0.030, 1.05)),
        origin=Origin(xyz=(-0.14, 0.195, 0.525)),
        material=gunmetal,
        name="side_wall_0",
    )
    safe_body.visual(
        Box((0.28, 0.030, 1.05)),
        origin=Origin(xyz=(-0.14, -0.195, 0.525)),
        material=gunmetal,
        name="side_wall_1",
    )
    safe_body.visual(
        Box((0.28, 0.42, 0.035)),
        origin=Origin(xyz=(-0.14, 0.0, 1.0325)),
        material=gunmetal,
        name="top_wall",
    )
    safe_body.visual(
        Box((0.28, 0.42, 0.035)),
        origin=Origin(xyz=(-0.14, 0.0, 0.0175)),
        material=gunmetal,
        name="bottom_wall",
    )

    front_flange = model.part("front_flange")
    # Outer sleeve 0.40 m wide by 0.99 m tall, with a 0.30 m by 0.88 m
    # rectangular aperture.  Four rails overlap at their mitered corners so the
    # part remains one connected manufactured sleeve.
    front_flange.visual(
        Box((0.055, 0.050, 0.990)),
        origin=Origin(xyz=(0.0275, 0.175, 0.525)),
        material=gunmetal,
        name="side_sleeve_0",
    )
    front_flange.visual(
        Box((0.055, 0.050, 0.990)),
        origin=Origin(xyz=(0.0275, -0.175, 0.525)),
        material=gunmetal,
        name="side_sleeve_1",
    )
    front_flange.visual(
        Box((0.055, 0.400, 0.055)),
        origin=Origin(xyz=(0.0275, 0.0, 0.9925)),
        material=gunmetal,
        name="top_sleeve",
    )
    front_flange.visual(
        Box((0.055, 0.400, 0.055)),
        origin=Origin(xyz=(0.0275, 0.0, 0.0575)),
        material=gunmetal,
        name="bottom_sleeve",
    )
    front_flange.visual(
        Box((0.012, 0.018, 0.840)),
        origin=Origin(xyz=(0.052, -0.160, 0.525)),
        material=worn_edge,
        name="hinge_leaf",
    )
    front_flange.visual(
        Box((0.035, 0.008, 0.820)),
        origin=Origin(xyz=(0.0695, -0.154, 0.525)),
        material=worn_edge,
        name="hinge_receiver",
    )

    model.articulation(
        "body_to_front_flange",
        ArticulationType.FIXED,
        parent=safe_body,
        child=front_flange,
    )

    door = model.part("door")
    door.visual(
        Box((0.040, 0.285, 0.840)),
        origin=Origin(xyz=(0.003, 0.138, 0.0)),
        material=dark_steel,
        name="door_panel",
    )
    door.visual(
        Box((0.006, 0.245, 0.790)),
        origin=Origin(xyz=(0.026, 0.138, 0.0)),
        material=gunmetal,
        name="raised_inner_panel",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.780),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=worn_edge,
        name="door_hinge_barrel",
    )
    door.visual(
        Box((0.008, 0.018, 0.070)),
        origin=Origin(xyz=(0.027, 0.173, 0.290)),
        material=worn_edge,
        name="key_hinge_mount",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=front_flange,
        child=door,
        origin=Origin(xyz=(0.075, -0.138, 0.525)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=0.0, upper=1.75),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.052, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="dial_center_cap",
    )
    dial.visual(
        Box((0.004, 0.008, 0.035)),
        origin=Origin(xyz=(0.0265, 0.0, 0.030)),
        material=white,
        name="dial_indicator",
    )

    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(0.029, 0.138, 0.180)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="lever_hub",
    )
    lever.visual(
        Box((0.022, 0.115, 0.022)),
        origin=Origin(xyz=(0.032, -0.055, 0.0)),
        material=brass,
        name="lever_arm",
    )
    lever.visual(
        Cylinder(radius=0.017, length=0.028),
        origin=Origin(xyz=(0.034, -0.112, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="lever_end",
    )

    model.articulation(
        "lever_spindle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=lever,
        origin=Origin(xyz=(0.029, 0.138, 0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.25),
    )

    key_cover = model.part("key_cover")
    key_cover.visual(
        Box((0.010, 0.070, 0.050)),
        origin=Origin(xyz=(0.005, -0.035, 0.0)),
        material=dark_steel,
        name="cover_plate",
    )
    key_cover.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=worn_edge,
        name="cover_hinge_barrel",
    )
    key_cover.visual(
        Box((0.003, 0.026, 0.004)),
        origin=Origin(xyz=(0.011, -0.035, 0.0)),
        material=black,
        name="key_slot",
    )

    model.articulation(
        "key_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=door,
        child=key_cover,
        origin=Origin(xyz=(0.031, 0.173, 0.290)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=2.0, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    door = object_model.get_part("door")
    flange = object_model.get_part("front_flange")
    dial = object_model.get_part("dial")
    lever = object_model.get_part("lever")
    key_cover = object_model.get_part("key_cover")

    door_hinge = object_model.get_articulation("door_hinge")
    dial_spin = object_model.get_articulation("dial_spin")
    lever_spindle = object_model.get_articulation("lever_spindle")
    key_cover_hinge = object_model.get_articulation("key_cover_hinge")

    def aabb_center_x(aabb):
        return None if aabb is None else (aabb[0][0] + aabb[1][0]) / 2.0

    def aabb_center_yz(aabb):
        return None if aabb is None else (
            (aabb[0][1] + aabb[1][1]) / 2.0,
            (aabb[0][2] + aabb[1][2]) / 2.0,
        )

    ctx.expect_gap(
        door,
        flange,
        axis="x",
        min_gap=0.001,
        max_gap=0.006,
        positive_elem="door_panel",
        negative_elem="top_sleeve",
        name="closed door sits just proud of the front sleeve",
    )
    ctx.expect_gap(
        dial,
        door,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="dial_body",
        negative_elem="raised_inner_panel",
        name="dial is mounted on the raised door face",
    )
    ctx.expect_gap(
        lever,
        door,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="lever_hub",
        negative_elem="raised_inner_panel",
        name="lever hub is mounted on the raised door face",
    )
    ctx.expect_gap(
        key_cover,
        door,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="cover_plate",
        negative_elem="key_hinge_mount",
        name="key override cover is hinged from a separate door-face mount",
    )

    closed_door = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.1}):
        opened_door = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "main door opens outward on the vertical left hinge",
        aabb_center_x(opened_door) is not None
        and aabb_center_x(closed_door) is not None
        and aabb_center_x(opened_door) > aabb_center_x(closed_door) + 0.06,
        details=f"closed={closed_door}, opened={opened_door}",
    )

    closed_indicator = aabb_center_yz(ctx.part_element_world_aabb(dial, elem="dial_indicator"))
    with ctx.pose({dial_spin: pi / 2.0}):
        spun_indicator = aabb_center_yz(ctx.part_element_world_aabb(dial, elem="dial_indicator"))
    ctx.check(
        "dial indicator rotates around the dial axis",
        closed_indicator is not None
        and spun_indicator is not None
        and abs(spun_indicator[1] - closed_indicator[1]) > 0.015,
        details=f"closed={closed_indicator}, spun={spun_indicator}",
    )

    closed_lever = aabb_center_yz(ctx.part_element_world_aabb(lever, elem="lever_end"))
    with ctx.pose({lever_spindle: 1.0}):
        turned_lever = aabb_center_yz(ctx.part_element_world_aabb(lever, elem="lever_end"))
    ctx.check(
        "short lever handle turns on its spindle",
        closed_lever is not None
        and turned_lever is not None
        and abs(turned_lever[1] - closed_lever[1]) > 0.035,
        details=f"closed={closed_lever}, turned={turned_lever}",
    )

    closed_cover = ctx.part_element_world_aabb(key_cover, elem="cover_plate")
    with ctx.pose({key_cover_hinge: 1.0}):
        opened_cover = ctx.part_element_world_aabb(key_cover, elem="cover_plate")
    ctx.check(
        "key override cover swings outward on its small hinge",
        aabb_center_x(opened_cover) is not None
        and aabb_center_x(closed_cover) is not None
        and aabb_center_x(opened_cover) > aabb_center_x(closed_cover) + 0.015,
        details=f"closed={closed_cover}, opened={opened_cover}",
    )

    return ctx.report()


object_model = build_object_model()
