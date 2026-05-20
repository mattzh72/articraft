from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_loading_washer")

    white = model.material("warm_white_enamel", rgba=(0.94, 0.94, 0.90, 1.0))
    light_gray = model.material("satin_control_gray", rgba=(0.72, 0.74, 0.74, 1.0))
    dark_gray = model.material("dark_rubber", rgba=(0.03, 0.032, 0.035, 1.0))
    glass = model.material("smoky_blue_glass", rgba=(0.35, 0.55, 0.70, 0.38))
    steel = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    black = model.material("black_shadow", rgba=(0.005, 0.005, 0.006, 1.0))
    detergent_blue = model.material("detergent_blue_plastic", rgba=(0.18, 0.38, 0.66, 1.0))

    cabinet = model.part("cabinet")

    # Cabinet coordinate frame: front face is near x=0, depth runs toward -X.
    width = 0.60
    depth = 0.60
    height = 0.85
    wall = 0.025
    half_w = width / 2.0

    cabinet.visual(
        Box((depth, wall, height)),
        origin=Origin(xyz=(-depth / 2.0, -half_w + wall / 2.0, height / 2.0)),
        material=white,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((depth, wall, height)),
        origin=Origin(xyz=(-depth / 2.0, half_w - wall / 2.0, height / 2.0)),
        material=white,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((depth, width, wall)),
        origin=Origin(xyz=(-depth / 2.0, 0.0, wall / 2.0)),
        material=white,
        name="base_panel",
    )
    cabinet.visual(
        Box((depth, width, wall)),
        origin=Origin(xyz=(-depth / 2.0, 0.0, height - wall / 2.0)),
        material=white,
        name="top_panel",
    )
    cabinet.visual(
        Box((wall, width, height)),
        origin=Origin(xyz=(-depth + wall / 2.0, 0.0, height / 2.0)),
        material=white,
        name="back_panel",
    )

    # Front frame panels leave a large circular porthole zone and an open drawer slot.
    cabinet.visual(
        Box((wall, width, 0.205)),
        origin=Origin(xyz=(-wall / 2.0, 0.0, 0.1025)),
        material=white,
        name="front_toe_panel",
    )
    cabinet.visual(
        Box((wall, 0.085, 0.455)),
        origin=Origin(xyz=(-wall / 2.0, -0.2575, 0.4325)),
        material=white,
        name="front_side_0",
    )
    cabinet.visual(
        Box((wall, 0.085, 0.455)),
        origin=Origin(xyz=(-wall / 2.0, 0.2575, 0.4325)),
        material=white,
        name="front_side_1",
    )
    cabinet.visual(
        Box((wall, width, 0.045)),
        origin=Origin(xyz=(-wall / 2.0, 0.0, 0.6825)),
        material=white,
        name="front_bridge",
    )
    cabinet.visual(
        Box((wall, width, 0.025)),
        origin=Origin(xyz=(-wall / 2.0, 0.0, 0.8375)),
        material=white,
        name="front_top_cap",
    )

    gasket_ring = BezelGeometry(
        (0.34, 0.34),
        (0.46, 0.46),
        0.024,
        opening_shape="circle",
        outer_shape="circle",
    )
    cabinet.visual(
        mesh_from_geometry(gasket_ring, "cabinet_gasket_ring"),
        origin=Origin(xyz=(0.002, 0.0, 0.43), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="gasket_ring",
    )

    # Split control panel fascia, leaving a rectangular detergent-drawer slot.
    cabinet.visual(
        Box((0.018, 0.34, 0.12)),
        origin=Origin(xyz=(0.006, -0.11, 0.765)),
        material=light_gray,
        name="control_field",
    )
    for idx, (mark_y, mark_z, mark_h) in enumerate(
        (
            (-0.188, 0.765, 0.020),
            (-0.171, 0.803, 0.014),
            (-0.130, 0.820, 0.020),
            (-0.089, 0.803, 0.014),
            (-0.072, 0.765, 0.020),
            (-0.089, 0.727, 0.014),
            (-0.130, 0.710, 0.020),
        )
    ):
        cabinet.visual(
            Box((0.003, 0.005, mark_h)),
            origin=Origin(xyz=(0.016, mark_y, mark_z)),
            material=black,
            name=f"program_tick_{idx}",
        )
    cabinet.visual(
        Box((0.018, 0.22, 0.025)),
        origin=Origin(xyz=(0.006, 0.18, 0.8125)),
        material=light_gray,
        name="slot_top_rail",
    )
    cabinet.visual(
        Box((0.018, 0.22, 0.025)),
        origin=Origin(xyz=(0.006, 0.18, 0.7075)),
        material=light_gray,
        name="slot_bottom_rail",
    )
    cabinet.visual(
        Box((0.018, 0.020, 0.080)),
        origin=Origin(xyz=(0.006, 0.070, 0.760)),
        material=light_gray,
        name="slot_side_0",
    )
    cabinet.visual(
        Box((0.018, 0.020, 0.080)),
        origin=Origin(xyz=(0.006, 0.290, 0.760)),
        material=light_gray,
        name="slot_side_1",
    )
    cabinet.visual(
        Box((0.250, 0.012, 0.014)),
        origin=Origin(xyz=(-0.125, 0.084, 0.773)),
        material=light_gray,
        name="drawer_guide_0",
    )
    cabinet.visual(
        Box((0.250, 0.012, 0.014)),
        origin=Origin(xyz=(-0.125, 0.276, 0.773)),
        material=light_gray,
        name="drawer_guide_1",
    )
    cabinet.visual(
        Box((0.033, 0.050, 0.052)),
        origin=Origin(xyz=(0.0155, 0.250, 0.545)),
        material=white,
        name="hinge_mount_0",
    )
    cabinet.visual(
        Box((0.033, 0.050, 0.052)),
        origin=Origin(xyz=(0.0155, 0.250, 0.315)),
        material=white,
        name="hinge_mount_1",
    )
    cabinet.visual(
        Cylinder(radius=0.065, length=0.040),
        origin=Origin(xyz=(-0.555, 0.0, 0.430), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="rear_bearing",
    )

    # Porthole door: child frame is exactly on the left hinge line.  The round
    # door extends toward local -Y when closed; positive yaw opens it outward.
    door = model.part("door")
    door_bezel = BezelGeometry(
        (0.30, 0.30),
        (0.44, 0.44),
        0.055,
        opening_shape="circle",
        outer_shape="circle",
    )
    door.visual(
        mesh_from_geometry(door_bezel, "door_bezel"),
        origin=Origin(xyz=(0.0, -0.225, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=0.155, length=0.012),
        origin=Origin(xyz=(0.018, -0.225, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="door_glass",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=steel,
        name="hinge_barrel_0",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, -0.115)),
        material=steel,
        name="hinge_barrel_1",
    )
    door.visual(
        Box((0.032, 0.090, 0.045)),
        origin=Origin(xyz=(0.0, -0.047, 0.115)),
        material=white,
        name="hinge_leaf_0",
    )
    door.visual(
        Box((0.032, 0.090, 0.045)),
        origin=Origin(xyz=(0.0, -0.047, -0.115)),
        material=white,
        name="hinge_leaf_1",
    )
    door.visual(
        Box((0.036, 0.050, 0.125)),
        origin=Origin(xyz=(0.022, -0.430, 0.0)),
        material=light_gray,
        name="pull_handle",
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.050, 0.225, 0.430)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.6, lower=0.0, upper=2.05),
    )

    drum = model.part("drum")
    drum_shell = BezelGeometry(
        (0.305, 0.305),
        (0.360, 0.360),
        0.340,
        opening_shape="circle",
        outer_shape="circle",
    )
    drum.visual(
        mesh_from_geometry(drum_shell, "drum_shell"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.158, length=0.010),
        origin=Origin(xyz=(-0.170, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="back_plate",
    )
    drum.visual(
        Cylinder(radius=0.040, length=0.490),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="axle_shadow",
    )
    for idx, angle in enumerate((math.pi / 2.0, 7.0 * math.pi / 6.0, 11.0 * math.pi / 6.0)):
        radius = 0.145
        y = radius * math.cos(angle)
        z = radius * math.sin(angle)
        drum.visual(
            Box((0.255, 0.020, 0.042)),
            origin=Origin(xyz=(0.010, y, z), rpy=(angle, 0.0, 0.0)),
            material=light_gray,
            name=f"drum_lifter_{idx}",
        )
    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(-0.290, 0.0, 0.430)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=20.0),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.025, 0.180, 0.058)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=white,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.004, 0.135, 0.012)),
        origin=Origin(xyz=(0.029, 0.0, -0.006)),
        material=black,
        name="drawer_pull",
    )
    drawer.visual(
        Box((0.300, 0.155, 0.035)),
        origin=Origin(xyz=(-0.135, 0.0, -0.005)),
        material=detergent_blue,
        name="tray",
    )
    drawer.visual(
        Box((0.290, 0.012, 0.014)),
        origin=Origin(xyz=(-0.140, -0.084, 0.013)),
        material=light_gray,
        name="slide_rail_0",
    )
    drawer.visual(
        Box((0.290, 0.012, 0.014)),
        origin=Origin(xyz=(-0.140, 0.084, 0.013)),
        material=light_gray,
        name="slide_rail_1",
    )
    model.articulation(
        "cabinet_to_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(0.015, 0.180, 0.760)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.30, lower=0.0, upper=0.180),
    )

    knob = model.part("program_knob")
    program_knob = KnobGeometry(
        0.075,
        0.035,
        body_style="skirted",
        top_diameter=0.060,
        skirt=KnobSkirt(0.086, 0.006, flare=0.05, chamfer=0.0015),
        grip=KnobGrip(style="fluted", count=28, depth=0.0015),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(program_knob, "program_knob"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_gray,
        name="knob_cap",
    )
    model.articulation(
        "cabinet_to_program_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(0.015, -0.130, 0.765)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    drum = object_model.get_part("drum")
    drawer = object_model.get_part("drawer")
    knob = object_model.get_part("program_knob")

    door_hinge = object_model.get_articulation("cabinet_to_door")
    drum_axle = object_model.get_articulation("cabinet_to_drum")
    drawer_slide = object_model.get_articulation("cabinet_to_drawer")
    knob_axis = object_model.get_articulation("cabinet_to_program_knob")

    ctx.check(
        "door uses a left vertical revolute hinge",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(door_hinge.axis) == (0.0, 0.0, 1.0)
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.upper > 1.8,
        details=f"type={door_hinge.articulation_type}, axis={door_hinge.axis}, limits={door_hinge.motion_limits}",
    )
    ctx.check(
        "drum uses a continuous horizontal axle",
        drum_axle.articulation_type == ArticulationType.CONTINUOUS
        and tuple(drum_axle.axis) == (1.0, 0.0, 0.0),
        details=f"type={drum_axle.articulation_type}, axis={drum_axle.axis}",
    )
    ctx.check(
        "detergent drawer uses front prismatic travel",
        drawer_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(drawer_slide.axis) == (1.0, 0.0, 0.0)
        and drawer_slide.motion_limits is not None
        and drawer_slide.motion_limits.upper >= 0.16,
        details=f"type={drawer_slide.articulation_type}, axis={drawer_slide.axis}, limits={drawer_slide.motion_limits}",
    )
    ctx.check(
        "program selector is a rotary front control",
        knob_axis.articulation_type == ArticulationType.CONTINUOUS
        and tuple(knob_axis.axis) == (1.0, 0.0, 0.0),
        details=f"type={knob_axis.articulation_type}, axis={knob_axis.axis}",
    )

    ctx.expect_overlap(
        door,
        cabinet,
        axes="yz",
        elem_a="door_ring",
        elem_b="gasket_ring",
        min_overlap=0.30,
        name="closed porthole door covers the circular gasket",
    )
    ctx.expect_gap(
        door,
        cabinet,
        axis="x",
        positive_elem="door_ring",
        negative_elem="gasket_ring",
        min_gap=0.004,
        max_gap=0.014,
        name="closed porthole door sits just proud of the front face",
    )
    ctx.expect_gap(
        door,
        cabinet,
        axis="x",
        positive_elem="hinge_barrel_0",
        negative_elem="hinge_mount_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper hinge barrel is seated against cabinet mount",
    )
    ctx.expect_gap(
        door,
        cabinet,
        axis="x",
        positive_elem="hinge_barrel_1",
        negative_elem="hinge_mount_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower hinge barrel is seated against cabinet mount",
    )
    ctx.expect_gap(
        drum,
        cabinet,
        axis="x",
        positive_elem="axle_shadow",
        negative_elem="rear_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="drum axle bears against the rear bearing",
    )
    ctx.expect_gap(
        knob,
        cabinet,
        axis="x",
        positive_elem="knob_cap",
        negative_elem="control_field",
        max_gap=0.001,
        max_penetration=0.00001,
        name="selector knob is mounted on the control panel",
    )
    ctx.expect_gap(
        drawer,
        cabinet,
        axis="y",
        positive_elem="slide_rail_0",
        negative_elem="drawer_guide_0",
        max_gap=0.001,
        max_penetration=0.001,
        name="drawer lower guide rail is captured",
    )
    ctx.expect_gap(
        cabinet,
        drawer,
        axis="y",
        positive_elem="drawer_guide_1",
        negative_elem="slide_rail_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="drawer upper guide rail is captured",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
    with ctx.pose({door_hinge: 1.35}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
    ctx.check(
        "door opens outward from the left hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.18,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_drawer_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_face")
    with ctx.pose({drawer_slide: 0.180}):
        extended_drawer_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_face")
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="x",
            elem_a="slide_rail_0",
            elem_b="drawer_guide_0",
            min_overlap=0.050,
            name="extended detergent drawer remains retained on rails",
        )
    ctx.check(
        "detergent drawer slides forward",
        closed_drawer_aabb is not None
        and extended_drawer_aabb is not None
        and extended_drawer_aabb[1][0] > closed_drawer_aabb[1][0] + 0.15,
        details=f"closed={closed_drawer_aabb}, extended={extended_drawer_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
