from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    CylinderGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


WHITE = Material("slightly_warm_white", rgba=(0.94, 0.94, 0.90, 1.0))
LIGHT_GRAY = Material("soft_gray_plastic", rgba=(0.70, 0.72, 0.72, 1.0))
DARK = Material("black_rubber_shadow", rgba=(0.015, 0.017, 0.018, 1.0))
STEEL = Material("brushed_stainless", rgba=(0.62, 0.64, 0.63, 1.0))
CHROME = Material("polished_chrome", rgba=(0.86, 0.88, 0.88, 1.0))
GLASS = Material("smoky_blue_glass", rgba=(0.33, 0.55, 0.72, 0.38))


def _drum_shell_mesh():
    """Return a shallow, open-front metal washing drum as a CadQuery mesh."""
    radius = 0.158
    length = 0.340
    wall = 0.010
    back = 0.014
    lip = 0.014

    outer = cq.Workplane("XY").circle(radius).extrude(length)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(radius - wall)
        .extrude(length - back + 0.001)
    )
    cup = outer.cut(inner)
    front_lip_hole = (
        cq.Workplane("XY")
        .workplane(offset=-0.002)
        .circle(radius - lip)
        .extrude(lip + 0.004)
    )
    cup = cup.cut(front_lip_hole).translate((0.0, 0.0, -length / 2.0))
    return mesh_from_cadquery(cup, "drum_shell", tolerance=0.0012, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_counter_washing_machine")

    width = 0.45
    depth = 0.58
    height = 0.82
    panel_thickness = 0.025
    front_y = -depth / 2.0
    porthole_z = 0.410

    cabinet = model.part("cabinet")

    front_panel = BezelGeometry(
        (0.335, 0.335),
        (width, height),
        panel_thickness,
        opening_shape="circle",
        outer_shape="rect",
        center=True,
    )
    cabinet.visual(
        mesh_from_geometry(front_panel, "front_panel"),
        origin=Origin(xyz=(0.0, front_y, height / 2.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=WHITE,
        name="front_panel",
    )

    # Thin appliance carcass panels: narrow enough for a built-in kitchen bay and
    # hollow enough to leave real clearance for the suspended drum.
    cabinet.visual(Box((0.018, depth, height)), origin=Origin(xyz=(-width / 2.0 + 0.009, 0.0, height / 2.0)), material=WHITE, name="side_panel_0")
    cabinet.visual(Box((0.018, depth, height)), origin=Origin(xyz=(width / 2.0 - 0.009, 0.0, height / 2.0)), material=WHITE, name="side_panel_1")
    cabinet.visual(Box((width, depth, 0.024)), origin=Origin(xyz=(0.0, 0.0, height - 0.012)), material=WHITE, name="top_panel")
    cabinet.visual(Box((width, depth, 0.024)), origin=Origin(xyz=(0.0, 0.0, 0.012)), material=WHITE, name="bottom_panel")
    cabinet.visual(Box((width, 0.024, height)), origin=Origin(xyz=(0.0, depth / 2.0 - 0.012, height / 2.0)), material=WHITE, name="back_panel")
    cabinet.visual(Box((width * 0.92, 0.020, 0.055)), origin=Origin(xyz=(0.0, front_y - 0.010, 0.055)), material=DARK, name="toe_kick")

    cabinet.visual(Box((0.420, 0.012, 0.074)), origin=Origin(xyz=(0.0, front_y - 0.018, 0.745)), material=LIGHT_GRAY, name="control_fascia")
    cabinet.visual(Box((0.155, 0.014, 0.045)), origin=Origin(xyz=(-0.100, front_y - 0.027, 0.746)), material=WHITE, name="detergent_drawer")
    cabinet.visual(Box((0.070, 0.010, 0.034)), origin=Origin(xyz=(0.174, front_y - 0.029, 0.748)), material=DARK, name="display_window")

    # Two fixed hinge leaves and pins mounted to the cabinet face.  The matching
    # moving barrels live on the door part and rotate around this same vertical line.
    hinge_x = -0.205
    hinge_y = front_y - 0.044
    hinge_z_offsets = (-0.145, 0.145)
    for i, zoff in enumerate(hinge_z_offsets):
        cabinet.visual(
            Box((0.030, 0.036, 0.034)),
            origin=Origin(xyz=(hinge_x - 0.010, front_y - 0.022, porthole_z + zoff)),
            material=CHROME,
            name=f"hinge_leaf_{i}",
        )
        cabinet.visual(
            Cylinder(0.006, 0.092),
            origin=Origin(xyz=(hinge_x, hinge_y, porthole_z + zoff)),
            material=CHROME,
            name=f"hinge_pin_{i}",
        )

    # Internal rear bearing support makes the drum axle physically read as mounted,
    # not a floating rotor inside the cabinet.
    cabinet.visual(Box((0.125, 0.140, 0.018)), origin=Origin(xyz=(0.0, 0.210, porthole_z + 0.058)), material=LIGHT_GRAY, name="bearing_support_top")
    cabinet.visual(Box((0.125, 0.140, 0.018)), origin=Origin(xyz=(0.0, 0.210, porthole_z - 0.058)), material=LIGHT_GRAY, name="bearing_support_bottom")
    cabinet.visual(Box((0.018, 0.140, 0.125)), origin=Origin(xyz=(-0.058, 0.210, porthole_z)), material=LIGHT_GRAY, name="bearing_support_0")
    cabinet.visual(Box((0.018, 0.140, 0.125)), origin=Origin(xyz=(0.058, 0.210, porthole_z)), material=LIGHT_GRAY, name="bearing_support_1")
    cabinet.visual(
        Cylinder(0.050, 0.044),
        origin=Origin(xyz=(0.0, 0.150, porthole_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=DARK,
        name="rear_bearing",
    )

    drum = model.part("drum")
    drum.visual(
        _drum_shell_mesh(),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="drum_shell",
    )
    drum.visual(
        mesh_from_geometry(CylinderGeometry(0.160, 0.012, radial_segments=64, closed=True), "drum_front_rim"),
        origin=Origin(xyz=(0.0, -0.171, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=CHROME,
        name="drum_front_rim",
    )
    drum.visual(
        Cylinder(0.025, 0.430),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="axle",
    )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        x = 0.142 * math.cos(angle)
        z = 0.142 * math.sin(angle)
        drum.visual(
            Box((0.030, 0.230, 0.016)),
            origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, -angle, 0.0)),
            material=LIGHT_GRAY if i else Material("blue_drum_lifter", rgba=(0.40, 0.55, 0.68, 1.0)),
            name=f"lifter_{i}",
        )
    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, -0.055, porthole_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=12.0),
    )

    door = model.part("door")
    door_ring = BezelGeometry(
        (0.278, 0.278),
        (0.410, 0.410),
        0.045,
        opening_shape="circle",
        outer_shape="circle",
        center=True,
    )
    door.visual(
        mesh_from_geometry(door_ring, "door_ring"),
        origin=Origin(xyz=(0.205, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=WHITE,
        name="door_ring",
    )
    door.visual(
        Cylinder(0.145, 0.014),
        origin=Origin(xyz=(0.205, -0.032, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=GLASS,
        name="glass_bowl",
    )
    for i, zoff in enumerate(hinge_z_offsets):
        door.visual(
            Cylinder(0.012, 0.096),
            origin=Origin(xyz=(0.0, 0.0, zoff)),
            material=CHROME,
            name=f"hinge_barrel_{i}",
        )
        door.visual(
            Box((0.092, 0.028, 0.036)),
            origin=Origin(xyz=(0.054, -0.005, zoff)),
            material=CHROME,
            name=f"door_hinge_leaf_{i}",
        )
    door.visual(
        Cylinder(0.012, 0.235),
        origin=Origin(xyz=(0.362, -0.064, 0.0)),
        material=CHROME,
        name="bar_handle",
    )
    for zoff in (-0.078, 0.078):
        door.visual(
            Cylinder(0.007, 0.052),
            origin=Origin(xyz=(0.362, -0.041, zoff), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=CHROME,
            name=f"handle_post_{0 if zoff < 0 else 1}",
        )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, porthole_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.95),
    )

    program_knob = model.part("program_knob")
    program_knob.visual(
        Cylinder(0.030, 0.022),
        origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=WHITE,
        name="knob_cap",
    )
    program_knob.visual(
        Box((0.006, 0.004, 0.030)),
        origin=Origin(xyz=(0.0, -0.024, 0.011)),
        material=DARK,
        name="knob_indicator",
    )
    model.articulation(
        "cabinet_to_program_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=program_knob,
        origin=Origin(xyz=(0.098, front_y - 0.024, 0.746)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5),
    )

    for i, x in enumerate((0.136, 0.164)):
        button = model.part(f"button_{i}")
        button.visual(
            Box((0.020, 0.010, 0.018)),
            origin=Origin(xyz=(0.0, -0.005, 0.0)),
            material=WHITE,
            name="button_cap",
        )
        model.articulation(
            f"cabinet_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, front_y - 0.024, 0.714)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drum_joint = object_model.get_articulation("cabinet_to_drum")
    door_joint = object_model.get_articulation("cabinet_to_door")

    for i in (0, 1):
        ctx.allow_overlap(
            cabinet,
            door,
            elem_a=f"hinge_pin_{i}",
            elem_b=f"hinge_barrel_{i}",
            reason="Each door barrel intentionally captures the fixed hinge pin on the cabinet.",
        )
        ctx.expect_within(
            cabinet,
            door,
            axes="xy",
            inner_elem=f"hinge_pin_{i}",
            outer_elem=f"hinge_barrel_{i}",
            margin=0.001,
            name=f"hinge pin {i} is concentric inside its barrel",
        )
        ctx.expect_overlap(
            cabinet,
            door,
            axes="z",
            elem_a=f"hinge_pin_{i}",
            elem_b=f"hinge_barrel_{i}",
            min_overlap=0.085,
            name=f"hinge pin {i} remains captured along the hinge length",
        )
        ctx.allow_overlap(
            cabinet,
            door,
            elem_a=f"hinge_leaf_{i}",
            elem_b=f"hinge_barrel_{i}",
            reason="The simplified cabinet hinge leaf locally nests against the moving barrel knuckle.",
        )
        ctx.expect_gap(
            cabinet,
            door,
            axis="y",
            positive_elem=f"hinge_leaf_{i}",
            negative_elem=f"hinge_barrel_{i}",
            max_penetration=0.010,
            name=f"hinge leaf {i} has only local barrel nesting",
        )

    ctx.allow_overlap(
        cabinet,
        drum,
        elem_a="rear_bearing",
        elem_b="axle",
        reason="The drum axle is intentionally seated through the rear bearing.",
    )
    ctx.expect_within(
        drum,
        cabinet,
        axes="xz",
        inner_elem="axle",
        outer_elem="rear_bearing",
        margin=0.001,
        name="axle and rear bearing share the continuous spin axis",
    )
    ctx.expect_overlap(
        cabinet,
        drum,
        axes="y",
        elem_a="rear_bearing",
        elem_b="axle",
        min_overlap=0.030,
        name="axle is retained in the rear bearing",
    )

    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        positive_elem="front_panel",
        negative_elem="door_ring",
        min_gap=0.003,
        max_gap=0.020,
        name="closed porthole door sits just proud of the front panel",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        elem_a="door_ring",
        elem_b="front_panel",
        min_overlap=0.35,
        name="wide circular door covers the front porthole opening",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.15}):
        opened_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward from the left hinge",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[0][1] < closed_aabb[0][1] - 0.10,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    with ctx.pose({drum_joint: math.pi / 2.0}):
        rotated_lifter = ctx.part_element_world_aabb(drum, elem="lifter_0")
    rest_lifter = ctx.part_element_world_aabb(drum, elem="lifter_0")
    ctx.check(
        "drum lifter visibly rotates around the continuous axle",
        rest_lifter is not None
        and rotated_lifter is not None
        and abs(((rotated_lifter[0][2] + rotated_lifter[1][2]) / 2.0) - ((rest_lifter[0][2] + rest_lifter[1][2]) / 2.0)) > 0.08,
        details=f"rest={rest_lifter}, rotated={rotated_lifter}",
    )

    return ctx.report()


object_model = build_object_model()
