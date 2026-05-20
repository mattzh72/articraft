from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _front_fascia_shape() -> cq.Workplane:
    """Single connected front skin with a round porthole and filter slot."""
    width = 0.600
    height = 0.860
    depth = 0.020
    front_y = -0.280
    door_z = 0.480
    opening_radius = 0.238
    drawer_z = 0.115

    fascia = (
        cq.Workplane("XZ")
        .rect(width, height)
        .extrude(depth)
        .translate((0.0, front_y, height / 2.0))
    )

    porthole_cut = (
        cq.Workplane("XZ")
        .circle(opening_radius)
        .extrude(depth * 3.0)
        .translate((0.0, front_y + depth * 1.5, door_z))
    )
    drawer_cut = (
        cq.Workplane("XZ")
        .rect(0.455, 0.105)
        .extrude(depth * 3.0)
        .translate((0.0, front_y + depth * 1.5, drawer_z))
    )

    return fascia.cut(porthole_cut).cut(drawer_cut)


def _door_ring_shape() -> cq.Workplane:
    """Annular circular door frame in the child frame; hinge line is local X=0."""
    outer_radius = 0.275
    glass_radius = 0.208
    thickness = 0.052
    center_x = outer_radius
    return (
        cq.Workplane("XZ")
        .center(center_x, 0.0)
        .circle(outer_radius)
        .circle(glass_radius)
        .extrude(thickness)
        .translate((0.0, thickness / 2.0, 0.0))
    )


def _body_gasket_shape() -> cq.Workplane:
    """Stationary black annulus around the front porthole opening."""
    return (
        cq.Workplane("XZ")
        .circle(0.258)
        .circle(0.238)
        .extrude(0.010)
        .translate((0.0, -0.300, 0.480))
    )


def _drum_shape() -> cq.Workplane:
    """Open stainless drum: a thin tube, rear pan, hub, and three tumble baffles."""
    radius = 0.220
    inner_radius = 0.203
    length = 0.405
    shell = (
        cq.Workplane("XZ")
        .circle(radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, length / 2.0, 0.0))
    )
    rear_pan = (
        cq.Workplane("XZ")
        .circle(inner_radius)
        .extrude(0.014)
        .translate((0.0, length / 2.0, 0.0))
    )
    hub = (
        cq.Workplane("XZ")
        .circle(0.045)
        .extrude(0.030)
        .translate((0.0, length / 2.0, 0.0))
    )
    baffle = cq.Workplane("XY").box(0.040, length * 0.78, 0.034).translate(
        (0.0, 0.0, inner_radius - 0.010)
    )
    baffles = baffle
    for angle in (120.0, 240.0):
        baffles = baffles.union(
            baffle.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle)
        )
    return shell.union(rear_pan).union(hub).union(baffles)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ventless_condensation_dryer")

    white = model.material("warm_white_enamel", rgba=(0.94, 0.95, 0.93, 1.0))
    shadow = model.material("deep_shadow", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_rubber = model.material("dark_rubber_gasket", rgba=(0.02, 0.022, 0.024, 1.0))
    glass = model.material("smoky_blue_glass", rgba=(0.30, 0.48, 0.62, 0.42))
    chrome = model.material("polished_chrome", rgba=(0.78, 0.78, 0.74, 1.0))
    steel = model.material("brushed_stainless_steel", rgba=(0.68, 0.70, 0.68, 1.0))
    control_gray = model.material("soft_gray_control_panel", rgba=(0.73, 0.75, 0.74, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.025, 0.560, 0.860)),
        origin=Origin(xyz=(-0.2875, 0.0, 0.430)),
        material=white,
        name="side_panel_0",
    )
    body.visual(
        Box((0.025, 0.560, 0.860)),
        origin=Origin(xyz=(0.2875, 0.0, 0.430)),
        material=white,
        name="side_panel_1",
    )
    body.visual(
        Box((0.600, 0.560, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.8475)),
        material=white,
        name="top_panel",
    )
    body.visual(
        Box((0.600, 0.560, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=white,
        name="bottom_panel",
    )
    body.visual(
        Box((0.600, 0.025, 0.860)),
        origin=Origin(xyz=(0.0, 0.2675, 0.430)),
        material=white,
        name="back_panel",
    )
    body.visual(
        mesh_from_cadquery(_front_fascia_shape(), "front_fascia", tolerance=0.0015),
        material=white,
        name="front_fascia",
    )
    body.visual(
        Box((0.540, 0.012, 0.075)),
        origin=Origin(xyz=(0.0, -0.306, 0.798)),
        material=control_gray,
        name="control_panel",
    )
    body.visual(
        mesh_from_cadquery(_body_gasket_shape(), "body_gasket", tolerance=0.0012),
        material=dark_rubber,
        name="porthole_gasket",
    )
    body.visual(
        Box((0.045, 0.026, 0.105)),
        origin=Origin(xyz=(-0.290, -0.301, 0.612)),
        material=chrome,
        name="hinge_leaf_0",
    )
    body.visual(
        Box((0.045, 0.026, 0.105)),
        origin=Origin(xyz=(-0.290, -0.301, 0.348)),
        material=chrome,
        name="hinge_leaf_1",
    )
    body.visual(
        Cylinder(radius=0.043, length=0.0675),
        origin=Origin(xyz=(0.0, 0.22125, 0.480), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_axle_bearing",
    )
    body.visual(
        Box((0.025, 0.230, 0.017)),
        origin=Origin(xyz=(-0.185, -0.170, 0.095)),
        material=control_gray,
        name="drawer_rail_0",
    )
    body.visual(
        Box((0.025, 0.230, 0.017)),
        origin=Origin(xyz=(0.185, -0.170, 0.095)),
        material=control_gray,
        name="drawer_rail_1",
    )
    body.visual(
        Box((0.092, 0.025, 0.017)),
        origin=Origin(xyz=(-0.239, -0.170, 0.095)),
        material=control_gray,
        name="rail_bracket_0",
    )
    body.visual(
        Box((0.092, 0.025, 0.017)),
        origin=Origin(xyz=(0.239, -0.170, 0.095)),
        material=control_gray,
        name="rail_bracket_1",
    )

    drum = model.part("drum")
    drum.visual(
        mesh_from_cadquery(_drum_shape(), "rotating_drum", tolerance=0.0015),
        material=steel,
        name="perforated_drum",
    )

    door = model.part("porthole_door")
    door.visual(
        mesh_from_cadquery(_door_ring_shape(), "door_ring", tolerance=0.0012),
        material=white,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=0.218, length=0.012),
        origin=Origin(xyz=(0.275, -0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="glass_bowl",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(xyz=(0.0, -0.005, 0.132)),
        material=chrome,
        name="hinge_barrel_0",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(xyz=(0.0, -0.005, -0.132)),
        material=chrome,
        name="hinge_barrel_1",
    )
    door.visual(
        Box((0.054, 0.030, 0.066)),
        origin=Origin(xyz=(0.026, -0.010, 0.132)),
        material=white,
        name="hinge_leaf_0",
    )
    door.visual(
        Box((0.054, 0.030, 0.066)),
        origin=Origin(xyz=(0.026, -0.010, -0.132)),
        material=white,
        name="hinge_leaf_1",
    )
    door.visual(
        Box((0.028, 0.034, 0.235)),
        origin=Origin(xyz=(0.490, -0.054, 0.0)),
        material=chrome,
        name="bar_handle",
    )
    door.visual(
        Box((0.040, 0.042, 0.030)),
        origin=Origin(xyz=(0.490, -0.033, 0.085)),
        material=chrome,
        name="handle_post_0",
    )
    door.visual(
        Box((0.040, 0.042, 0.030)),
        origin=Origin(xyz=(0.490, -0.033, -0.085)),
        material=chrome,
        name="handle_post_1",
    )

    lint_drawer = model.part("lint_drawer")
    lint_drawer.visual(
        Box((0.420, 0.028, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=white,
        name="filter_face",
    )
    lint_drawer.visual(
        Box((0.350, 0.245, 0.035)),
        origin=Origin(xyz=(0.0, 0.122, 0.006)),
        material=control_gray,
        name="filter_tray",
    )
    lint_drawer.visual(
        Box((0.210, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.019, -0.012)),
        material=shadow,
        name="finger_recess",
    )

    program_knob = model.part("program_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.070,
            0.030,
            body_style="skirted",
            top_diameter=0.052,
            edge_radius=0.002,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0015),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        ),
        "program_knob",
    )
    program_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white,
        name="knob_cap",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Cylinder(radius=0.021, length=0.014),
        origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="button_cap",
    )

    model.articulation(
        "body_to_drum",
        ArticulationType.REVOLUTE,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, -0.015, 0.480)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.275, -0.335, 0.480)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.85),
    )
    model.articulation(
        "body_to_lint_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lint_drawer,
        origin=Origin(xyz=(0.0, -0.295, 0.115)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.145),
    )
    model.articulation(
        "body_to_program_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=program_knob,
        origin=Origin(xyz=(0.170, -0.312, 0.798)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=5.0),
    )
    model.articulation(
        "body_to_start_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=start_button,
        origin=Origin(xyz=(-0.200, -0.312, 0.798)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.006),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("porthole_door")
    lint_drawer = object_model.get_part("lint_drawer")
    program_knob = object_model.get_part("program_knob")
    start_button = object_model.get_part("start_button")

    drum_axle = object_model.get_articulation("body_to_drum")
    door_hinge = object_model.get_articulation("body_to_door")
    drawer_slide = object_model.get_articulation("body_to_lint_drawer")
    button_slide = object_model.get_articulation("body_to_start_button")

    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="front_fascia",
        negative_elem="door_ring",
        min_gap=0.0,
        max_gap=0.018,
        name="closed circular door sits just proud of the front skin",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        elem_a="glass_bowl",
        elem_b="porthole_gasket",
        min_overlap=0.38,
        name="smoky glass covers the round gasket opening",
    )
    ctx.expect_contact(
        body,
        lint_drawer,
        elem_a="drawer_rail_0",
        elem_b="filter_tray",
        contact_tol=0.001,
        name="lint filter tray rides on a base rail",
    )
    ctx.expect_overlap(
        lint_drawer,
        body,
        axes="y",
        elem_a="filter_tray",
        elem_b="bottom_panel",
        min_overlap=0.20,
        name="closed lint drawer remains inserted in the base",
    )
    ctx.expect_contact(
        body,
        program_knob,
        elem_a="control_panel",
        elem_b="knob_cap",
        contact_tol=0.001,
        name="program knob is mounted on the control panel",
    )
    ctx.expect_contact(
        body,
        start_button,
        elem_a="control_panel",
        elem_b="button_cap",
        contact_tol=0.001,
        name="start button is mounted on the control panel",
    )

    def _center_y(aabb):
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) / 2.0

    closed_ring = ctx.part_element_world_aabb(door, elem="door_ring")
    closed_ring_y = _center_y(closed_ring)
    with ctx.pose({door_hinge: 1.20}):
        opened_ring = ctx.part_element_world_aabb(door, elem="door_ring")
        opened_ring_y = _center_y(opened_ring)
    ctx.check(
        "left-hinged porthole door swings outward",
        closed_ring_y is not None
        and opened_ring_y is not None
        and opened_ring_y < closed_ring_y - 0.20,
        details=f"closed_y={closed_ring_y}, opened_y={opened_ring_y}",
    )

    drawer_rest = ctx.part_world_position(lint_drawer)
    with ctx.pose({drawer_slide: 0.145}):
        ctx.expect_overlap(
            lint_drawer,
            body,
            axes="y",
            elem_a="filter_tray",
            elem_b="bottom_panel",
            min_overlap=0.07,
            name="extended lint drawer retains hidden insertion",
        )
        drawer_extended = ctx.part_world_position(lint_drawer)
    ctx.check(
        "lint filter drawer slides outward from the base",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[1] < drawer_rest[1] - 0.10,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    drum_rest = ctx.part_world_position(drum)
    with ctx.pose({drum_axle: 1.40}):
        drum_spun = ctx.part_world_position(drum)
    ctx.check(
        "drum rotates about a fixed axle",
        drum_rest is not None
        and drum_spun is not None
        and abs(drum_spun[0] - drum_rest[0]) < 1e-6
        and abs(drum_spun[1] - drum_rest[1]) < 1e-6
        and abs(drum_spun[2] - drum_rest[2]) < 1e-6,
        details=f"rest={drum_rest}, spun={drum_spun}",
    )

    button_rest = ctx.part_world_position(start_button)
    with ctx.pose({button_slide: 0.006}):
        button_pressed = ctx.part_world_position(start_button)
    ctx.check(
        "start button depresses into the fascia",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[1] > button_rest[1] + 0.004,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
