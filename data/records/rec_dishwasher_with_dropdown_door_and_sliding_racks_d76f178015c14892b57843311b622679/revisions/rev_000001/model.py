from __future__ import annotations

from math import pi

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.60
BODY_DEPTH = 0.62
BODY_HEIGHT = 0.86
WALL_THICKNESS = 0.02
TOE_HEIGHT = 0.10
DOOR_HEIGHT = 0.74
DOOR_THICKNESS = 0.045
RAIL_LENGTH = 0.46
RAIL_CENTER_Y = -0.035
RACK_JOINT_Y = -0.265
LOWER_RAIL_Z = 0.180
UPPER_RAIL_Z = 0.450
TRAY_RAIL_Z = 0.690
CONTROL_SURFACE_Y = DOOR_THICKNESS + 0.010


def make_cabinet_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(
        BODY_WIDTH,
        BODY_DEPTH,
        BODY_HEIGHT,
        centered=(True, True, False),
    )
    inner = cq.Workplane("XY").transformed(
        offset=(0.0, WALL_THICKNESS / 2.0, WALL_THICKNESS)
    ).box(
        BODY_WIDTH - 2.0 * WALL_THICKNESS,
        BODY_DEPTH - WALL_THICKNESS,
        BODY_HEIGHT - 2.0 * WALL_THICKNESS,
        centered=(True, True, False),
    )
    shell = outer.cut(inner)

    toe_recess = cq.Workplane("XY").transformed(
        offset=(0.0, BODY_DEPTH / 2.0 - 0.05, 0.0)
    ).box(
        BODY_WIDTH * 0.72,
        0.10,
        TOE_HEIGHT,
        centered=(True, True, False),
    )
    return shell.cut(toe_recess)


def make_door_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(
        BODY_WIDTH - 0.012,
        DOOR_THICKNESS,
        DOOR_HEIGHT,
        centered=(True, True, False),
    )
    inner_liner = cq.Workplane("XY").transformed(
        offset=(0.0, -0.006, 0.030)
    ).box(
        BODY_WIDTH - 0.110,
        DOOR_THICKNESS - 0.012,
        DOOR_HEIGHT - 0.145,
        centered=(True, True, False),
    )
    fascia = cq.Workplane("XY").transformed(
        offset=(0.0, 0.004, DOOR_HEIGHT - 0.110)
    ).box(
        BODY_WIDTH - 0.012,
        DOOR_THICKNESS + 0.008,
        0.110,
        centered=(True, True, False),
    )
    return outer.cut(inner_liner).union(fascia)


def add_rack_visuals(
    part,
    *,
    width: float,
    depth: float,
    height: float,
    guide_z: float,
    material,
    support_post_name: str | None = None,
    tine_height: float = 0.0,
    shallow: bool = False,
) -> None:
    wire = 0.010 if not shallow else 0.008
    slat_z = wire * 0.6
    slat_positions = [0.040 + index * (depth - 0.080) / max(1, (5 if not shallow else 4) - 1) for index in range(5 if not shallow else 4)]

    for y_center in (wire / 2.0, depth - wire / 2.0):
        part.visual(
            Box((width, wire, wire)),
            origin=Origin(xyz=(0.0, y_center, wire / 2.0)),
            material=material,
        )
    for x_center in (-width / 2.0 + wire / 2.0, width / 2.0 - wire / 2.0):
        part.visual(
            Box((wire, depth, wire)),
            origin=Origin(xyz=(x_center, depth / 2.0, wire / 2.0)),
            material=material,
        )

    for y_center in slat_positions:
        part.visual(
            Box((width - 0.020, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, y_center, slat_z)),
            material=material,
        )

    if shallow:
        base_wire_x_positions = (-0.150, -0.075, 0.0, 0.075, 0.150)
    elif support_post_name is None:
        base_wire_x_positions = (-0.110, -0.055, 0.0, 0.055, 0.110)
    else:
        base_wire_x_positions = (-0.120, -0.070, 0.070, 0.120)
    for x_center in base_wire_x_positions:
        part.visual(
            Box((0.006, depth, 0.006)),
            origin=Origin(xyz=(x_center, depth / 2.0, slat_z)),
            material=material,
        )

    if not shallow:
        for x_center in (-width / 2.0 + wire / 2.0, width / 2.0 - wire / 2.0):
            for y_center in (wire / 2.0, depth - wire / 2.0):
                part.visual(
                    Box((wire, wire, height)),
                    origin=Origin(xyz=(x_center, y_center, height / 2.0)),
                    material=material,
                )
        for y_center in (wire / 2.0, depth - wire / 2.0):
            part.visual(
                Box((width, wire, wire)),
                origin=Origin(xyz=(0.0, y_center, height - wire / 2.0)),
                material=material,
            )
        for x_center in (-width / 2.0 + wire / 2.0, width / 2.0 - wire / 2.0):
            part.visual(
                Box((wire, depth, wire)),
                origin=Origin(xyz=(x_center, depth / 2.0, height - wire / 2.0)),
                material=material,
            )

    if tine_height > 0.0:
        if support_post_name is None:
            tine_y_positions = (0.120, 0.190, 0.260, 0.330, 0.400)
            tine_x_positions = (-0.110, -0.055, 0.0, 0.055, 0.110)
        else:
            tine_y_positions = (0.120, 0.190, 0.330, 0.400)
            tine_x_positions = (-0.120, -0.070, 0.070, 0.120)
        tine_base_z = slat_z + 0.003
        for row_sign in (-1.0, 1.0):
            for x_center, y_center in zip(tine_x_positions, tine_y_positions):
                part.visual(
                    Box((0.006, 0.006, tine_height)),
                    origin=Origin(
                        xyz=(
                            x_center,
                            y_center + row_sign * 0.012,
                            tine_base_z + tine_height / 2.0,
                        )
                    ),
                    material=material,
                )

    guide_width = 0.020
    guide_height = 0.010
    guide_length = 0.44 if not shallow else 0.40
    support_height = guide_z + guide_height / 2.0
    support_y_positions = (0.024, min(depth - 0.024, guide_length - 0.024))
    for guide_index, guide_sign in enumerate((-1.0, 1.0)):
        guide_x = guide_sign * 0.260
        support_x = guide_sign * (width / 2.0 - wire / 2.0 + 0.005)
        part.visual(
            Box((guide_width, guide_length, guide_height)),
            origin=Origin(xyz=(guide_x, guide_length / 2.0, guide_z)),
            material=material,
            name=f"guide_{guide_index}",
        )
        for y_center in support_y_positions:
            part.visual(
                Box((0.018, 0.020, support_height)),
                origin=Origin(xyz=(support_x, y_center, support_height / 2.0)),
                material=material,
            )

    handle_width = width * (0.42 if not shallow else 0.50)
    handle_z = height + (0.010 if not shallow else 0.006)
    handle_thickness = 0.016 if not shallow else 0.012
    part.visual(
        Box((handle_width, 0.012, handle_thickness)),
        origin=Origin(xyz=(0.0, depth + 0.006, handle_z)),
        material=material,
        name="handle",
    )
    handle_bottom_z = handle_z - handle_thickness / 2.0
    stem_base_z = height if not shallow else wire
    stem_height = max(0.012, handle_bottom_z - stem_base_z)
    for stem_sign in (-1.0, 1.0):
        part.visual(
            Box((0.010, 0.012, stem_height)),
            origin=Origin(
                xyz=(
                    stem_sign * handle_width * 0.30,
                    depth + 0.006,
                    stem_base_z + stem_height / 2.0,
                )
            ),
            material=material,
        )

    if support_post_name is not None:
        part.visual(
            Cylinder(radius=0.012, length=0.054),
            origin=Origin(xyz=(0.0, depth * 0.50, 0.027)),
            material=material,
            name=support_post_name,
        )


def add_spray_arm_visuals(part, *, span: float, material) -> None:
    part.visual(
        Cylinder(radius=0.023, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=material,
        name="hub",
    )
    part.visual(
        Box((span, 0.022, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=material,
        name="arm_beam",
    )
    for arm_sign in (-1.0, 1.0):
        part.visual(
            Box((0.060, 0.030, 0.008)),
            origin=Origin(xyz=(arm_sign * (span * 0.35), 0.0, 0.010)),
            material=material,
        )
        part.visual(
            Box((0.018, 0.010, 0.012)),
            origin=Origin(
                xyz=(arm_sign * (span * 0.43), arm_sign * 0.010, 0.012)
            ),
            material=material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_dishwasher")

    stainless = model.material("stainless", rgba=(0.72, 0.75, 0.78, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.11, 0.12, 1.0))
    chamber = model.material("chamber", rgba=(0.58, 0.61, 0.66, 1.0))
    rack_coating = model.material("rack_coating", rgba=(0.78, 0.86, 0.92, 1.0))
    control_black = model.material("control_black", rgba=(0.08, 0.09, 0.10, 1.0))
    control_gray = model.material("control_gray", rgba=(0.32, 0.34, 0.37, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(make_cabinet_shell(), "cabinet_shell"),
        material=stainless,
        name="shell",
    )

    rail_x = BODY_WIDTH / 2.0 - WALL_THICKNESS / 2.0 - 0.010
    for level_name, rail_z in (
        ("lower", LOWER_RAIL_Z),
        ("upper", UPPER_RAIL_Z),
        ("tray", TRAY_RAIL_Z),
    ):
        for side_name, rail_sign in (("0", -1.0), ("1", 1.0)):
            cabinet.visual(
                Box((0.020, RAIL_LENGTH, 0.010)),
                origin=Origin(xyz=(rail_sign * rail_x, RAIL_CENTER_Y, rail_z)),
                material=chamber,
                name=f"{level_name}_rail_{side_name}",
            )

    cabinet.visual(
        Cylinder(radius=0.015, length=0.090),
        origin=Origin(xyz=(0.0, -0.010, 0.065)),
        material=chamber,
        name="lower_spray_support",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(make_door_shell(), "door_shell"),
        origin=Origin(xyz=(0.0, DOOR_THICKNESS / 2.0, 0.0)),
        material=stainless,
        name="door_panel",
    )
    door.visual(
        Box((BODY_WIDTH - 0.060, 0.006, 0.095)),
        origin=Origin(xyz=(0.0, CONTROL_SURFACE_Y - 0.003, DOOR_HEIGHT - 0.062)),
        material=dark_trim,
        name="fascia_trim",
    )
    door.visual(
        geometry=mesh_from_cadquery(
            cq.Workplane("XY").box(BODY_WIDTH - 0.090, 0.010, 0.040),
            "door_handle",
        ),
        origin=Origin(xyz=(0.0, DOOR_THICKNESS + 0.010, DOOR_HEIGHT - 0.125)),
        material=dark_trim,
        name="handle",
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0, TOE_HEIGHT)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.55,
            effort=40.0,
            velocity=1.2,
        ),
    )

    lower_rack = model.part("lower_rack")
    add_rack_visuals(
        lower_rack,
        width=0.500,
        depth=0.500,
        height=0.110,
        guide_z=0.055,
        material=rack_coating,
        tine_height=0.085,
    )
    model.articulation(
        "cabinet_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_rack,
        origin=Origin(xyz=(0.0, RACK_JOINT_Y, 0.125)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.340,
            effort=40.0,
            velocity=0.35,
        ),
    )

    upper_rack = model.part("upper_rack")
    add_rack_visuals(
        upper_rack,
        width=0.490,
        depth=0.480,
        height=0.090,
        guide_z=0.055,
        material=rack_coating,
        support_post_name="upper_spray_support",
        tine_height=0.055,
    )
    model.articulation(
        "cabinet_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_rack,
        origin=Origin(xyz=(0.0, RACK_JOINT_Y, 0.395)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.320,
            effort=35.0,
            velocity=0.35,
        ),
    )

    tray = model.part("tray")
    add_rack_visuals(
        tray,
        width=0.490,
        depth=0.460,
        height=0.035,
        guide_z=0.055,
        material=rack_coating,
        shallow=True,
    )
    for divider_x in (-0.080, 0.0, 0.080):
        tray.visual(
            Box((0.006, 0.320, 0.025)),
            origin=Origin(xyz=(divider_x, 0.210, 0.018)),
            material=rack_coating,
        )
    model.articulation(
        "cabinet_to_tray",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=tray,
        origin=Origin(xyz=(0.0, RACK_JOINT_Y, 0.635)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.240,
            effort=20.0,
            velocity=0.30,
        ),
    )

    lower_spray_arm = model.part("lower_spray_arm")
    add_spray_arm_visuals(lower_spray_arm, span=0.250, material=control_gray)
    model.articulation(
        "cabinet_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=lower_spray_arm,
        origin=Origin(xyz=(0.0, -0.010, 0.110)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0),
    )

    upper_spray_arm = model.part("upper_spray_arm")
    add_spray_arm_visuals(upper_spray_arm, span=0.220, material=control_gray)
    model.articulation(
        "upper_rack_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=upper_rack,
        child=upper_spray_arm,
        origin=Origin(xyz=(0.0, 0.240, 0.054)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=10.0),
    )

    wing_0 = model.part("wing_0")
    wing_1 = model.part("wing_1")
    for wing in (wing_0, wing_1):
        wing.visual(
            Box((0.008, 0.180, 0.008)),
            origin=Origin(xyz=(0.0, 0.090, 0.004)),
            material=rack_coating,
            name="hinge_bar",
        )
        for y_center in (0.030, 0.090, 0.150):
            wing.visual(
                Box((0.008, 0.008, 0.080)),
                origin=Origin(xyz=(0.0, y_center, 0.048)),
                material=rack_coating,
            )
        wing.visual(
            Box((0.008, 0.180, 0.008)),
            origin=Origin(xyz=(0.0, 0.090, 0.088)),
            material=rack_coating,
        )
    model.articulation(
        "lower_rack_to_wing_0",
        ArticulationType.REVOLUTE,
        parent=lower_rack,
        child=wing_0,
        origin=Origin(xyz=(-0.185, 0.150, 0.009)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.20,
            effort=5.0,
            velocity=1.0,
        ),
    )
    model.articulation(
        "lower_rack_to_wing_1",
        ArticulationType.REVOLUTE,
        parent=lower_rack,
        child=wing_1,
        origin=Origin(xyz=(0.185, 0.150, 0.009)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.20,
            effort=5.0,
            velocity=1.0,
        ),
    )

    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.046,
            0.024,
            body_style="skirted",
            top_diameter=0.036,
            skirt=KnobSkirt(0.054, 0.006, flare=0.10),
            grip=KnobGrip(style="fluted", count=18, depth=0.0011),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
            center=False,
        ),
        "dishwasher_dial",
    )
    for index, dial_x in enumerate((-0.175, 0.175)):
        dial = model.part(f"dial_{index}")
        dial.visual(
            dial_mesh,
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
            material=control_black,
            name="knob",
        )
        model.articulation(
            f"door_to_dial_{index}",
            ArticulationType.CONTINUOUS,
            parent=door,
            child=dial,
            origin=Origin(xyz=(dial_x, CONTROL_SURFACE_Y, DOOR_HEIGHT - 0.060)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=8.0),
        )

    button_x_positions = (-0.060, 0.0, 0.060)
    for index, button_x in enumerate(button_x_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.040, 0.014, 0.022)),
            origin=Origin(xyz=(0.0, 0.007, 0.011)),
            material=control_gray,
            name="cap",
        )
        model.articulation(
            f"door_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(button_x, CONTROL_SURFACE_Y, DOOR_HEIGHT - 0.062)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                lower=0.0,
                upper=0.005,
                effort=3.0,
                velocity=0.08,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    tray = object_model.get_part("tray")
    lower_spray_arm = object_model.get_part("lower_spray_arm")
    upper_spray_arm = object_model.get_part("upper_spray_arm")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    lower_slide = object_model.get_articulation("cabinet_to_lower_rack")
    upper_slide = object_model.get_articulation("cabinet_to_upper_rack")
    tray_slide = object_model.get_articulation("cabinet_to_tray")
    lower_spray_joint = object_model.get_articulation("cabinet_to_lower_spray_arm")
    upper_spray_joint = object_model.get_articulation("upper_rack_to_upper_spray_arm")
    wing_0_joint = object_model.get_articulation("lower_rack_to_wing_0")
    wing_1_joint = object_model.get_articulation("lower_rack_to_wing_1")
    dial_joints = [
        object_model.get_articulation("door_to_dial_0"),
        object_model.get_articulation("door_to_dial_1"),
    ]
    button_parts = [object_model.get_part(f"button_{index}") for index in range(3)]
    button_joints = [
        object_model.get_articulation(f"door_to_button_{index}") for index in range(3)
    ]

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            min_gap=0.0,
            max_gap=0.004,
            name="closed door sits flush to the cabinet front",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            min_overlap=0.50,
            name="door covers the front opening area",
        )
        ctx.expect_contact(
            lower_spray_arm,
            cabinet,
            elem_a="hub",
            elem_b="lower_spray_support",
            name="lower spray arm sits on its central support",
        )
        ctx.expect_contact(
            upper_spray_arm,
            upper_rack,
            elem_a="hub",
            elem_b="upper_spray_support",
            name="upper spray arm sits on its rack support",
        )

    with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
        door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        ctx.check(
            "door opens downward on the bottom hinge",
            door_aabb is not None
            and door_aabb[1][1] > BODY_DEPTH / 2.0 + 0.55
            and door_aabb[0][2] < TOE_HEIGHT + 0.02,
            details=f"door_panel_aabb={door_aabb}",
        )

    ctx.check(
        "dial pair uses continuous fascia-normal rotation",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            and abs(joint.axis[1]) > 0.99
            for joint in dial_joints
        ),
        details=f"dial_axes={[joint.axis for joint in dial_joints]}",
    )
    ctx.check(
        "spray arms rotate on vertical supports",
        lower_spray_joint.articulation_type == ArticulationType.CONTINUOUS
        and upper_spray_joint.articulation_type == ArticulationType.CONTINUOUS
        and abs(lower_spray_joint.axis[2]) > 0.99
        and abs(upper_spray_joint.axis[2]) > 0.99,
        details=(
            f"lower_axis={lower_spray_joint.axis}, "
            f"upper_axis={upper_spray_joint.axis}"
        ),
    )

    for rack_name, rack_part, slide_joint, min_overlap, min_delta in (
        ("lower rack", lower_rack, lower_slide, 0.18, 0.30),
        ("upper rack", upper_rack, upper_slide, 0.18, 0.28),
        ("utensil tray", tray, tray_slide, 0.14, 0.20),
    ):
        rest_pos = ctx.part_world_position(rack_part)
        with ctx.pose({slide_joint: slide_joint.motion_limits.upper}):
            extended_pos = ctx.part_world_position(rack_part)
            ctx.expect_overlap(
                rack_part,
                cabinet,
                axes="y",
                min_overlap=min_overlap,
                name=f"{rack_name} retains rail engagement at full extension",
            )
        ctx.check(
            f"{rack_name} slides outward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[1] > rest_pos[1] + min_delta,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    for button_part, button_joint in zip(button_parts, button_joints):
        rest_pos = ctx.part_world_position(button_part)
        with ctx.pose({button_joint: button_joint.motion_limits.upper}):
            pressed_pos = ctx.part_world_position(button_part)
        ctx.check(
            f"{button_part.name} presses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] < rest_pos[1] - 0.003,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    for wing_part, wing_joint in ((wing_0, wing_0_joint), (wing_1, wing_1_joint)):
        rest_aabb = ctx.part_world_aabb(wing_part)
        with ctx.pose({wing_joint: wing_joint.motion_limits.upper}):
            folded_aabb = ctx.part_world_aabb(wing_part)
        ctx.check(
            f"{wing_part.name} folds down from the lower rack",
            rest_aabb is not None
            and folded_aabb is not None
            and folded_aabb[1][2] < rest_aabb[1][2] - 0.045,
            details=f"rest={rest_aabb}, folded={folded_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
