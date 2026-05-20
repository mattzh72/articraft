from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_toaster_oven")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.70, 0.66, 1.0))
    dark = model.material("black_enamel", rgba=(0.025, 0.024, 0.023, 1.0))
    trim = model.material("charcoal_trim", rgba=(0.08, 0.075, 0.07, 1.0))
    glass = model.material("smoked_glass", rgba=(0.12, 0.17, 0.20, 0.38))
    rack_metal = model.material("polished_wire", rgba=(0.86, 0.84, 0.78, 1.0))
    heater = model.material("warm_quartz", rgba=(1.0, 0.46, 0.16, 1.0))
    white = model.material("white_indicator", rgba=(0.95, 0.92, 0.86, 1.0))

    depth = 0.38
    width = 0.54
    height = 0.29
    wall = 0.025

    body = model.part("body")
    body.visual(
        Box((depth, width, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=stainless,
        name="bottom_shell",
    )
    body.visual(
        Box((depth, width, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=stainless,
        name="top_shell",
    )
    body.visual(
        Box((depth, wall, height)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall / 2.0, height / 2.0)),
        material=stainless,
        name="side_wall_0",
    )
    body.visual(
        Box((depth, wall, height)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall / 2.0, height / 2.0)),
        material=stainless,
        name="side_wall_1",
    )
    body.visual(
        Box((wall, width, height)),
        origin=Origin(xyz=(depth / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=stainless,
        name="rear_wall",
    )

    # Dark front bezel frames the visible oven mouth and covers the wall seams.
    front_x = -depth / 2.0 - 0.002
    body.visual(
        Box((0.014, 0.46, 0.020)),
        origin=Origin(xyz=(front_x, 0.0, 0.010)),
        material=trim,
        name="front_bottom_trim",
    )
    body.visual(
        Box((0.014, 0.46, 0.035)),
        origin=Origin(xyz=(front_x, 0.0, 0.2525)),
        material=trim,
        name="front_top_trim",
    )
    body.visual(
        Box((0.014, 0.035, 0.24)),
        origin=Origin(xyz=(front_x, 0.235, 0.140)),
        material=trim,
        name="front_side_trim_0",
    )
    body.visual(
        Box((0.014, 0.035, 0.24)),
        origin=Origin(xyz=(front_x, -0.235, 0.140)),
        material=trim,
        name="front_side_trim_1",
    )

    # Straight rack ledges molded into the side walls.
    body.visual(
        Box((0.30, 0.055, 0.009)),
        origin=Origin(xyz=(0.01, 0.2175, 0.1105)),
        material=trim,
        name="guide_0",
    )
    body.visual(
        Box((0.30, 0.055, 0.009)),
        origin=Origin(xyz=(0.01, -0.2175, 0.1105)),
        material=trim,
        name="guide_1",
    )

    # Quartz heater tubes pass into the side walls so they read as mounted.
    for idx, (x, z) in enumerate(((0.02, 0.068), (0.11, 0.068), (0.02, 0.225), (0.11, 0.225))):
        body.visual(
            Cylinder(radius=0.006, length=0.51),
            origin=Origin(xyz=(x, 0.0, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=heater,
            name=f"heater_{idx}",
        )

    # Side-mounted control collars and small rubber feet.
    knob_x = -0.08
    knob_y = -width / 2.0 - 0.003
    knob_zs = (0.19, 0.105)
    body.visual(
        Cylinder(radius=0.035, length=0.006),
        origin=Origin(xyz=(knob_x, knob_y, knob_zs[0]), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="knob_collar_0",
    )
    body.visual(
        Cylinder(radius=0.035, length=0.006),
        origin=Origin(xyz=(knob_x, knob_y, knob_zs[1]), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="knob_collar_1",
    )

    for idx, (x, y) in enumerate(((-0.13, 0.19), (-0.13, -0.19), (0.13, 0.19), (0.13, -0.19))):
        body.visual(
            Box((0.060, 0.055, 0.018)),
            origin=Origin(xyz=(x, y, -0.007)),
            material=dark,
            name=f"foot_{idx}",
        )

    # Alternating outer hinge knuckles on the body; the door barrel occupies
    # the center span between them.
    hinge_x = -depth / 2.0 - 0.015
    hinge_z = 0.046
    for idx, y in enumerate((0.255, -0.255)):
        body.visual(
            Cylinder(radius=0.008, length=0.035),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=trim,
            name=f"body_hinge_{idx}",
        )

    door = model.part("door")
    door_width = 0.46
    door_height = 0.215
    door_thick = 0.018
    rail = 0.035
    door_x = -0.014
    door.visual(
        Box((door_thick, door_width, rail)),
        origin=Origin(xyz=(door_x, 0.0, rail / 2.0)),
        material=trim,
        name="bottom_rail",
    )
    door.visual(
        Box((door_thick, door_width, rail)),
        origin=Origin(xyz=(door_x, 0.0, door_height - rail / 2.0)),
        material=trim,
        name="top_rail",
    )
    for idx, y in enumerate((door_width / 2.0 - rail / 2.0, -door_width / 2.0 + rail / 2.0)):
        door.visual(
            Box((door_thick, rail, door_height)),
            origin=Origin(xyz=(door_x, y, door_height / 2.0)),
            material=trim,
            name=f"side_rail_{idx}",
        )
    door.visual(
        Box((0.006, door_width - 2.0 * rail + 0.014, door_height - 2.0 * rail + 0.014)),
        origin=Origin(xyz=(door_x + 0.003, 0.0, door_height / 2.0)),
        material=glass,
        name="glass_pane",
    )
    door.visual(
        Box((0.018, 0.28, 0.018)),
        origin=Origin(xyz=(-0.038, 0.0, 0.160)),
        material=dark,
        name="handle_bar",
    )
    for idx, y in enumerate((0.11, -0.11)):
        door.visual(
            Box((0.030, 0.018, 0.022)),
            origin=Origin(xyz=(-0.026, y, 0.160)),
            material=dark,
            name=f"handle_post_{idx}",
        )
    door.visual(
        Cylinder(radius=0.008, length=0.475),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="door_hinge",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.45),
    )

    rack = model.part("rack")
    rack_length = 0.26
    rack_width = 0.39
    wire_r = 0.003
    # Side rails and cross wires form one welded sliding rack.
    rack.visual(
        Cylinder(radius=wire_r, length=rack_length),
        origin=Origin(xyz=(rack_length / 2.0, rack_width / 2.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rack_metal,
        name="side_rod_0",
    )
    rack.visual(
        Cylinder(radius=wire_r, length=rack_length),
        origin=Origin(xyz=(rack_length / 2.0, -rack_width / 2.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rack_metal,
        name="side_rod_1",
    )
    for idx, x in enumerate((0.0, 0.065, 0.130, 0.195, 0.260)):
        rack.visual(
            Cylinder(radius=wire_r, length=rack_width),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rack_metal,
            name=f"cross_rod_{idx}",
        )
    for idx, y in enumerate((-0.12, -0.06, 0.0, 0.06, 0.12)):
        rack.visual(
            Cylinder(radius=wire_r * 0.85, length=rack_length),
            origin=Origin(xyz=(rack_length / 2.0, y, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=rack_metal,
            name=f"deck_rod_{idx}",
        )
    for idx, y in enumerate((0.16, -0.16)):
        rack.visual(
            Cylinder(radius=wire_r, length=0.035),
            origin=Origin(xyz=(-0.005, y, 0.0175)),
            material=rack_metal,
            name=f"front_upright_{idx}",
        )
    rack.visual(
        Cylinder(radius=wire_r, length=0.32),
        origin=Origin(xyz=(-0.005, 0.0, 0.035), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rack_metal,
        name="front_lip",
    )

    model.articulation(
        "body_to_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rack,
        origin=Origin(xyz=(-0.12, 0.0, 0.118)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.35, lower=0.0, upper=0.18),
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.050,
            0.026,
            body_style="skirted",
            top_diameter=0.040,
            skirt=KnobSkirt(0.058, 0.006, flare=0.05, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=20, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
        ),
        "appliance_knob",
    )

    for name, z in (("upper_knob", knob_zs[0]), ("lower_knob", knob_zs[1])):
        knob = model.part(name)
        knob.visual(
            Cylinder(radius=0.008, length=0.012),
            origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=trim,
            name="shaft",
        )
        knob.visual(
            knob_mesh,
            origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark,
            name="cap",
        )
        knob.visual(
            Box((0.004, 0.002, 0.022)),
            origin=Origin(xyz=(0.0, -0.039, 0.010)),
            material=white,
            name="pointer",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, -width / 2.0 - 0.006, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=4.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    rack = object_model.get_part("rack")
    upper_knob = object_model.get_part("upper_knob")
    lower_knob = object_model.get_part("lower_knob")
    door_joint = object_model.get_articulation("body_to_door")
    rack_joint = object_model.get_articulation("body_to_rack")
    upper_joint = object_model.get_articulation("body_to_upper_knob")
    lower_joint = object_model.get_articulation("body_to_lower_knob")

    with ctx.pose({door_joint: 0.0, rack_joint: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="x",
            min_gap=0.0005,
            max_gap=0.020,
            positive_elem="front_top_trim",
            negative_elem="top_rail",
            name="closed door sits just proud of front bezel",
        )
        ctx.expect_contact(
            rack,
            body,
            elem_a="side_rod_0",
            elem_b="guide_0",
            contact_tol=0.001,
            name="rack side rod rests on its straight guide",
        )
        ctx.expect_overlap(
            rack,
            body,
            axes="x",
            elem_a="side_rod_0",
            elem_b="guide_0",
            min_overlap=0.12,
            name="rack is retained on guide when stowed",
        )

    closed_door_box = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.45}):
        opened_door_box = ctx.part_world_aabb(door)
    ctx.check(
        "door rotates downward and forward",
        closed_door_box is not None
        and opened_door_box is not None
        and opened_door_box[0][0] < closed_door_box[0][0] - 0.12
        and opened_door_box[1][2] < closed_door_box[1][2] - 0.10,
        details=f"closed={closed_door_box}, opened={opened_door_box}",
    )

    stowed_pos = ctx.part_world_position(rack)
    with ctx.pose({rack_joint: 0.18}):
        extended_pos = ctx.part_world_position(rack)
        ctx.expect_overlap(
            rack,
            body,
            axes="x",
            elem_a="side_rod_0",
            elem_b="guide_0",
            min_overlap=0.07,
            name="extended rack remains captured by guide",
        )
    ctx.check(
        "rack slides out toward the front",
        stowed_pos is not None and extended_pos is not None and extended_pos[0] < stowed_pos[0] - 0.15,
        details=f"stowed={stowed_pos}, extended={extended_pos}",
    )

    ctx.expect_contact(
        upper_knob,
        body,
        elem_a="shaft",
        elem_b="knob_collar_0",
        contact_tol=0.001,
        name="upper knob shaft seats on collar",
    )
    ctx.expect_contact(
        lower_knob,
        body,
        elem_a="shaft",
        elem_b="knob_collar_1",
        contact_tol=0.001,
        name="lower knob shaft seats on collar",
    )
    ctx.check(
        "both side knobs are continuous rotary controls",
        upper_joint.articulation_type == ArticulationType.CONTINUOUS
        and lower_joint.articulation_type == ArticulationType.CONTINUOUS,
    )

    return ctx.report()


object_model = build_object_model()
