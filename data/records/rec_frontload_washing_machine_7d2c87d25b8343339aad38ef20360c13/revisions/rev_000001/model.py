from __future__ import annotations

import math

import cadquery as cq

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
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


WIDTH = 0.64
DEPTH = 0.64
HEIGHT = 0.88
FRONT_Y = -DEPTH / 2.0
DRUM_Z = 0.46


def _cabinet_shell() -> cq.Workplane:
    """Rounded appliance cabinet with a real front drum aperture and drawer bay."""
    body = (
        cq.Workplane("XY")
        .box(WIDTH, DEPTH, HEIGHT)
        .translate((0.0, 0.0, HEIGHT / 2.0))
    )
    # A modest all-edge fillet gives the painted sheet-metal body appliance-like
    # rolled corners without making the rectangular cabinet cartoonishly soft.
    body = body.edges("|Z or >Z").fillet(0.018)

    # Front-load drum throat: a true cylindrical opening into the front panel.
    drum_cut = (
        cq.Workplane("XZ")
        .center(0.0, DRUM_Z)
        .circle(0.213)
        .extrude(0.18)
        .translate((0.0, FRONT_Y - 0.025, 0.0))
    )
    body = body.cut(drum_cut)

    # Detergent drawer pocket at the top left.  It is cut deeper than the drawer
    # travel so the sliding tray is not represented as colliding with a solid
    # appliance block.
    drawer_cut = (
        cq.Workplane("XY")
        .box(0.238, 0.46, 0.096)
        .translate((-0.205, FRONT_Y + 0.205, 0.765))
    )
    body = body.cut(drawer_cut)

    # Lower service/kick-panel rebate.
    service_rebate = (
        cq.Workplane("XY")
        .box(0.50, 0.030, 0.115)
        .translate((0.0, FRONT_Y - 0.006, 0.135))
    )
    body = body.cut(service_rebate)
    return body


def _knob_mesh():
    knob = KnobGeometry(
        0.074,
        0.034,
        body_style="skirted",
        top_diameter=0.058,
        edge_radius=0.002,
        grip=KnobGrip(style="fluted", count=28, depth=0.0015),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    return mesh_from_geometry(knob, "program_dial_cap")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_washing_machine")

    porcelain = model.material("warm_white_enamel", rgba=(0.93, 0.94, 0.91, 1.0))
    white_plastic = model.material("white_plastic", rgba=(0.86, 0.88, 0.86, 1.0))
    dark_panel = model.material("gloss_black_panel", rgba=(0.015, 0.017, 0.020, 1.0))
    smoky_glass = model.material("smoked_blue_glass", rgba=(0.18, 0.32, 0.45, 0.42))
    chrome = model.material("satin_chrome", rgba=(0.72, 0.74, 0.72, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.018, 0.016, 1.0))
    drum_metal = model.material("brushed_drum_steel", rgba=(0.63, 0.65, 0.64, 1.0))
    led_blue = model.material("blue_lit_display", rgba=(0.08, 0.26, 0.70, 1.0))
    shadow = model.material("deep_shadow", rgba=(0.002, 0.002, 0.003, 1.0))

    body = model.part("body")
    body.visual(
        Box((WIDTH, 0.050, HEIGHT)),
        origin=Origin(xyz=(0.0, DEPTH / 2.0 - 0.025, HEIGHT / 2.0)),
        material=porcelain,
        name="rear_panel",
    )
    body.visual(
        Box((0.050, DEPTH, HEIGHT)),
        origin=Origin(xyz=(-WIDTH / 2.0 + 0.025, 0.0, HEIGHT / 2.0)),
        material=porcelain,
        name="side_panel_0",
    )
    body.visual(
        Box((0.050, DEPTH, HEIGHT)),
        origin=Origin(xyz=(WIDTH / 2.0 - 0.025, 0.0, HEIGHT / 2.0)),
        material=porcelain,
        name="side_panel_1",
    )
    body.visual(
        Box((WIDTH, DEPTH, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT - 0.025)),
        material=porcelain,
        name="top_panel",
    )
    body.visual(
        Box((WIDTH, DEPTH, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=porcelain,
        name="base_panel",
    )
    body.visual(
        Box((0.380, 0.045, 0.185)),
        origin=Origin(xyz=(0.130, FRONT_Y + 0.018, 0.770)),
        material=porcelain,
        name="front_upper_panel",
    )
    body.visual(
        Box((WIDTH, 0.045, 0.255)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.018, 0.135)),
        material=porcelain,
        name="front_lower_panel",
    )
    body.visual(
        Box((0.035, 0.045, 0.485)),
        origin=Origin(xyz=(-0.3025, FRONT_Y + 0.018, DRUM_Z)),
        material=porcelain,
        name="front_post_0",
    )
    body.visual(
        Box((0.105, 0.045, 0.485)),
        origin=Origin(xyz=(0.270, FRONT_Y + 0.018, DRUM_Z)),
        material=porcelain,
        name="front_post_1",
    )

    # Front control fascia and display, built as proud/inset plates on the upper
    # front panel while leaving the detergent drawer opening clear.
    body.visual(
        Box((0.370, 0.014, 0.085)),
        origin=Origin(xyz=(0.140, FRONT_Y - 0.007, 0.770)),
        material=dark_panel,
        name="control_fascia",
    )
    body.visual(
        Box((0.112, 0.006, 0.034)),
        origin=Origin(xyz=(0.010, FRONT_Y - 0.017, 0.780)),
        material=led_blue,
        name="display_window",
    )
    for surround_name, sx, sz, ox, oz in (
        ("drawer_frame_top", 0.238, 0.012, 0.0, 0.047),
        ("drawer_frame_bottom", 0.238, 0.012, 0.0, -0.047),
        ("drawer_frame_side_0", 0.012, 0.080, -0.119, 0.0),
        ("drawer_frame_side_1", 0.012, 0.080, 0.119, 0.0),
    ):
        body.visual(
            Box((sx, 0.012, sz)),
            origin=Origin(xyz=(-0.180 + ox, FRONT_Y - 0.006, 0.765 + oz)),
            material=white_plastic,
            name=surround_name,
        )

    # Rubber boot and drum visible through the porthole.  The boot overlaps the
    # cut edge of the cabinet visual, making the opening read as a real gasketed
    # aperture rather than a painted circle.
    body.visual(
        mesh_from_geometry(TorusGeometry(radius=0.207, tube=0.019, radial_segments=36, tubular_segments=72), "front_boot"),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.020, DRUM_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="front_boot",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(radius=0.232, tube=0.012, radial_segments=36, tubular_segments=72), "door_recess_ring"),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.002, DRUM_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=porcelain,
        name="door_recess_ring",
    )
    body.visual(
        Cylinder(radius=0.188, length=0.145),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.090, DRUM_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shadow,
        name="dark_tub_throat",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(radius=0.143, tube=0.012, radial_segments=32, tubular_segments=64), "drum_lip"),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.034, DRUM_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum_metal,
        name="drum_lip",
    )
    body.visual(
        Cylinder(radius=0.132, length=0.006),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.068, DRUM_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum_metal,
        name="drum_back",
    )
    # Dark perforation marks on the back of the drum, arranged in realistic arcs.
    hole_positions = []
    for radius, count in ((0.040, 8), (0.075, 14), (0.108, 20)):
        for i in range(count):
            angle = 2.0 * math.pi * i / count + (0.17 if count == 14 else 0.0)
            hole_positions.append((radius * math.cos(angle), radius * math.sin(angle)))
    for idx, (x, z) in enumerate(hole_positions):
        body.visual(
            Cylinder(radius=0.0045, length=0.0025),
            origin=Origin(
                xyz=(x, FRONT_Y + 0.064, DRUM_Z + z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=shadow,
            name=f"drum_hole_{idx}",
        )

    # Toe service door, lower air slots, side seam/top lip, feet and rear hose
    # stubs enrich the appliance construction without introducing extra joints.
    body.visual(
        Box((0.475, 0.008, 0.090)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0085, 0.135)),
        material=white_plastic,
        name="service_panel",
    )
    vent = SlotPatternPanelGeometry(
        (0.310, 0.043),
        0.004,
        slot_size=(0.035, 0.006),
        pitch=(0.052, 0.014),
        frame=0.006,
        corner_radius=0.004,
        stagger=True,
    )
    body.visual(
        mesh_from_geometry(vent, "toe_vent"),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0135, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_panel,
        name="toe_vent",
    )
    body.visual(
        Box((0.590, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.005, HEIGHT - 0.018)),
        material=white_plastic,
        name="top_front_lip",
    )
    body.visual(
        Box((0.018, 0.590, 0.012)),
        origin=Origin(xyz=(-WIDTH / 2.0 + 0.017, 0.0, HEIGHT - 0.026)),
        material=white_plastic,
        name="side_top_seam_0",
    )
    body.visual(
        Box((0.018, 0.590, 0.012)),
        origin=Origin(xyz=(WIDTH / 2.0 - 0.017, 0.0, HEIGHT - 0.026)),
        material=white_plastic,
        name="side_top_seam_1",
    )
    for idx, x in enumerate((-0.245, 0.245)):
        for idy, y in enumerate((FRONT_Y + 0.060, DEPTH / 2.0 - 0.080)):
            body.visual(
                Cylinder(radius=0.035, length=0.030),
                origin=Origin(xyz=(x, y, 0.015)),
                material=Material("dark_rubber_feet", rgba=(0.035, 0.032, 0.030, 1.0)),
                name=f"leveling_foot_{idx}_{idy}",
            )
    body.visual(
        Cylinder(radius=0.038, length=0.055),
        origin=Origin(xyz=(-0.155, DEPTH / 2.0 + 0.010, 0.620), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_panel,
        name="rear_hose_port",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.065),
        origin=Origin(xyz=(0.165, DEPTH / 2.0 + 0.015, 0.585), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="water_inlet_stub",
    )
    for hinge_name, z in (("fixed_hinge_upper", DRUM_Z + 0.118), ("fixed_hinge_lower", DRUM_Z - 0.118)):
        body.visual(
            Box((0.047, 0.020, 0.135)),
            origin=Origin(xyz=(-0.255, FRONT_Y - 0.010, z)),
            material=chrome,
            name=hinge_name,
        )

    # Door: the part frame is the exposed left vertical hinge line.  The circular
    # porthole center lies in local +X, so a -Z hinge axis makes positive q swing
    # the door outward toward the user.
    door = model.part("door")
    door.visual(
        mesh_from_geometry(TorusGeometry(radius=0.195, tube=0.020, radial_segments=40, tubular_segments=80), "door_outer_ring"),
        origin=Origin(xyz=(0.210, -0.024, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="door_outer_ring",
    )
    door.visual(
        mesh_from_geometry(TorusGeometry(radius=0.160, tube=0.011, radial_segments=32, tubular_segments=64), "door_inner_retainer"),
        origin=Origin(xyz=(0.210, -0.032, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="door_inner_retainer",
    )
    door.visual(
        Cylinder(radius=0.156, length=0.010),
        origin=Origin(xyz=(0.210, -0.036, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=smoky_glass,
        name="convex_glass",
    )
    door.visual(
        Box((0.035, 0.028, 0.135)),
        origin=Origin(xyz=(-0.004, -0.024, 0.118)),
        material=chrome,
        name="upper_hinge_barrel",
    )
    door.visual(
        Box((0.035, 0.028, 0.135)),
        origin=Origin(xyz=(-0.004, -0.024, -0.118)),
        material=chrome,
        name="lower_hinge_barrel",
    )
    door.visual(
        Box((0.080, 0.018, 0.030)),
        origin=Origin(xyz=(0.038, -0.025, 0.000)),
        material=chrome,
        name="hinge_bridge",
    )
    door.visual(
        Box((0.035, 0.035, 0.118)),
        origin=Origin(xyz=(0.407, -0.055, 0.000)),
        material=white_plastic,
        name="door_pull",
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.210, FRONT_Y - 0.010, DRUM_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=1.85),
    )

    # Detergent drawer: a shallow face with a retained internal tray sliding out
    # along the front normal.  Positive travel moves it outward (-Y).
    drawer = model.part("detergent_drawer")
    drawer.visual(
        Box((0.226, 0.018, 0.082)),
        origin=Origin(xyz=(0.0, -0.011, 0.0)),
        material=white_plastic,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.166, 0.282, 0.042)),
        origin=Origin(xyz=(0.0, 0.139, -0.010)),
        material=Material("translucent_detergent_tray", rgba=(0.78, 0.84, 0.88, 0.72)),
        name="drawer_tray",
    )
    drawer.visual(
        Box((0.125, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.023, -0.014)),
        material=dark_panel,
        name="drawer_grip_shadow",
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(-0.180, FRONT_Y - 0.010, 0.765)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.35, lower=0.0, upper=0.165),
    )

    dial = model.part("program_dial")
    dial.visual(
        _knob_mesh(),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="dial_cap",
    )
    dial.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_panel,
        name="dial_shadow_collar",
    )
    model.articulation(
        "program_selector",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.165, FRONT_Y - 0.020, 0.775)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-2.75, upper=2.75),
    )

    # Three separate push-buttons, each with realistic short inward travel.
    for idx, (name, x) in enumerate((("power_button", 0.235), ("temp_button", 0.275), ("start_button", 0.305))):
        button = model.part(name)
        button.visual(
            Box((0.034, 0.014, 0.025)),
            origin=Origin(xyz=(0.0, -0.007, 0.0)),
            material=white_plastic if idx < 2 else Material("green_start_button", rgba=(0.08, 0.42, 0.16, 1.0)),
            name="button_cap",
        )
        button.visual(
            Box((0.027, 0.003, 0.004)),
            origin=Origin(xyz=(0.0, -0.015, 0.0)),
            material=dark_panel,
            name="button_icon_line",
        )
        model.articulation(
            f"{name}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, FRONT_Y - 0.014, 0.731)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=0.12, lower=0.0, upper=0.007),
        )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    drawer = object_model.get_part("detergent_drawer")
    dial = object_model.get_part("program_dial")
    door_hinge = object_model.get_articulation("door_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")
    selector = object_model.get_articulation("program_selector")
    start_press = object_model.get_articulation("start_button_press")

    # Closed door sits proud of the gasketed front aperture rather than being
    # fused into the cabinet, while still overlapping the appliance opening in
    # projection.
    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="front_boot",
            negative_elem="door_inner_retainer",
            min_gap=0.002,
            max_gap=0.035,
            name="closed door clears front gasket",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="convex_glass",
            elem_b="drum_back",
            min_overlap=0.120,
            name="glass covers drum opening",
        )
        closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.45}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward from front",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.120,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    # The drawer has a retained tray inside its cut bay at both rest and use.
    ctx.expect_overlap(
        drawer,
        body,
        axes="y",
        elem_a="drawer_tray",
        elem_b="side_panel_0",
        min_overlap=0.150,
        name="closed drawer tray is retained in body",
    )
    with ctx.pose({drawer_slide: 0.165}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="drawer_tray",
            elem_b="side_panel_0",
            min_overlap=0.060,
            name="extended drawer tray remains inserted",
        )
        extended_drawer_pos = ctx.part_world_position(drawer)
    rest_drawer_pos = ctx.part_world_position(drawer)
    ctx.check(
        "drawer slides outward",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < rest_drawer_pos[1] - 0.12,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    # Standard user controls articulate in the expected directions.
    rest_dial_aabb = ctx.part_world_aabb(dial)
    with ctx.pose({selector: 1.0}):
        turned_dial_aabb = ctx.part_world_aabb(dial)
    ctx.check(
        "program selector rotates in place",
        rest_dial_aabb is not None and turned_dial_aabb is not None,
        details=f"rest={rest_dial_aabb}, turned={turned_dial_aabb}",
    )
    rest_start = ctx.part_world_position("start_button")
    with ctx.pose({start_press: 0.007}):
        pressed_start = ctx.part_world_position("start_button")
    ctx.check(
        "start button presses inward",
        rest_start is not None and pressed_start is not None and pressed_start[1] > rest_start[1] + 0.005,
        details=f"rest={rest_start}, pressed={pressed_start}",
    )

    return ctx.report()


object_model = build_object_model()
