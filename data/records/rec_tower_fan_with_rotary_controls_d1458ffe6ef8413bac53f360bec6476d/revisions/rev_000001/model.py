from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
    superellipse_profile,
)


def _oval_plate_mesh(name: str, width: float, depth: float, height: float, *, exponent: float = 2.5):
    profile = superellipse_profile(width, depth, exponent=exponent, segments=72)
    return mesh_from_geometry(ExtrudeGeometry.from_z0(profile, height), name)


def _tower_shell_mesh(
    name: str,
    *,
    width: float,
    depth: float,
    height: float,
    wall: float,
    open_half_angle: float = 0.66,
    segments: int = 56,
):
    """Thin U-shaped tower shell: rounded sides/back with a front grille opening."""

    outer_rx = width * 0.5
    outer_ry = depth * 0.5
    inner_rx = outer_rx - wall
    inner_ry = outer_ry - wall
    start = -math.pi / 2.0 + open_half_angle
    end = 3.0 * math.pi / 2.0 - open_half_angle

    def point(theta: float, rx: float, ry: float, z: float):
        return (rx * math.cos(theta), ry * math.sin(theta), z)

    outer_bottom = []
    outer_top = []
    inner_bottom = []
    inner_top = []
    for i in range(segments + 1):
        t = start + (end - start) * i / segments
        outer_bottom.append(point(t, outer_rx, outer_ry, 0.0))
        outer_top.append(point(t, outer_rx, outer_ry, height))
        inner_bottom.append(point(t, inner_rx, inner_ry, 0.0))
        inner_top.append(point(t, inner_rx, inner_ry, height))

    geom = MeshGeometry()

    def add_loop(points):
        return [geom.add_vertex(*p) for p in points]

    ob = add_loop(outer_bottom)
    ot = add_loop(outer_top)
    ib = add_loop(inner_bottom)
    it = add_loop(inner_top)

    def quad(a, b, c, d):
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i in range(segments):
        # Outer and inner curved wall faces.
        quad(ob[i], ob[i + 1], ot[i + 1], ot[i])
        quad(ib[i + 1], ib[i], it[i], it[i + 1])
        # Top and bottom lips, making the shell a real thin-walled part.
        quad(ot[i], ot[i + 1], it[i + 1], it[i])
        quad(ob[i + 1], ob[i], ib[i], ib[i + 1])

    # Close the two vertical front jambs.
    quad(ob[0], ot[0], it[0], ib[0])
    quad(ob[-1], ib[-1], it[-1], ot[-1])

    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedroom_tower_fan")

    warm_white = Material("warm_white_plastic", color=(0.86, 0.84, 0.78, 1.0))
    dark = Material("charcoal_grille", color=(0.035, 0.037, 0.040, 1.0))
    black = Material("black_rubber", color=(0.005, 0.005, 0.006, 1.0))
    satin = Material("satin_control_gray", color=(0.48, 0.50, 0.52, 1.0))
    light_button = Material("light_button_plastic", color=(0.92, 0.93, 0.91, 1.0))
    blue_gray = Material("shadow_blower", color=(0.18, 0.22, 0.25, 1.0))
    tick = Material("printed_tick_marks", color=(0.08, 0.08, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        _oval_plate_mesh("low_oval_base", 0.38, 0.25, 0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=warm_white,
        name="low_oval_base",
    )
    base.visual(
        _oval_plate_mesh("black_foot_pad", 0.33, 0.20, 0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=black,
        name="rubber_foot_pad",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.031),
        origin=Origin(xyz=(0.0, 0.0, 0.0555)),
        material=dark,
        name="oscillation_bearing",
    )

    body = model.part("body_shell")
    body.visual(
        _tower_shell_mesh(
            "rounded_tower_shell",
            width=0.180,
            depth=0.132,
            height=0.790,
            wall=0.005,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=warm_white,
        name="rounded_tower_shell",
    )
    body.visual(
        Cylinder(radius=0.047, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=warm_white,
        name="pivot_socket",
    )
    body.visual(
        Cylinder(radius=0.092, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=warm_white,
        name="lower_shell_flare",
    )
    for side, x in enumerate((-0.062, 0.062)):
        body.visual(
            Box((0.014, 0.024, 0.690)),
            origin=Origin(xyz=(x, -0.061, 0.435)),
            material=warm_white,
            name=f"front_side_rail_{side}",
        )
    body.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (0.124, 0.650),
                frame=0.010,
                face_thickness=0.004,
                duct_depth=0.010,
                slat_pitch=0.020,
                slat_width=0.008,
                slat_angle_deg=18.0,
                corner_radius=0.010,
                slats=VentGrilleSlats(profile="airfoil", direction="down", divider_count=2, divider_width=0.003),
                frame_profile=VentGrilleFrame(style="beveled", depth=0.0012),
                sleeve=VentGrilleSleeve(style="none"),
            ),
            "front_outlet_grille",
        ),
        origin=Origin(xyz=(0.0, -0.068, 0.435), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="front_outlet_grille",
    )
    body.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.065, 0.360),
                0.003,
                slot_size=(0.040, 0.006),
                pitch=(0.052, 0.018),
                frame=0.006,
                corner_radius=0.006,
                stagger=True,
            ),
            "rear_intake_slots",
        ),
        origin=Origin(xyz=(0.0, 0.065, 0.430), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="rear_intake_slots",
    )
    body.visual(
        Box((0.018, 0.070, 0.008)),
        origin=Origin(xyz=(0.0, 0.0405, 0.120)),
        material=dark,
        name="lower_blower_bearing_bridge",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, -0.006, 0.120)),
        material=dark,
        name="lower_blower_bearing",
    )
    body.visual(
        Box((0.018, 0.070, 0.008)),
        origin=Origin(xyz=(0.0, 0.0405, 0.740)),
        material=dark,
        name="upper_blower_bearing_bridge",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, -0.006, 0.740)),
        material=dark,
        name="upper_blower_bearing",
    )

    deck = model.part("control_deck")
    deck.visual(
        _oval_plate_mesh("separate_control_deck", 0.168, 0.124, 0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin,
        name="deck_plate",
    )
    deck.visual(
        _oval_plate_mesh("recessed_top_panel", 0.138, 0.092, 0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=dark,
        name="top_control_panel",
    )
    for i, x in enumerate((-0.060, -0.046, -0.032, 0.007, 0.020, 0.033)):
        deck.visual(
            Box((0.0025, 0.014, 0.0012)),
            origin=Origin(xyz=(x, 0.046, 0.0225)),
            material=tick,
            name=f"dial_tick_{i}",
        )

    blower = model.part("blower_wheel")
    blower.visual(
        Cylinder(radius=0.005, length=0.660),
        origin=Origin(),
        material=dark,
        name="blower_axle",
    )
    blower.visual(
        Cylinder(radius=0.024, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.290)),
        material=blue_gray,
        name="lower_blower_hub",
    )
    blower.visual(
        Cylinder(radius=0.024, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=blue_gray,
        name="upper_blower_hub",
    )
    blower.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                0.047,
                0.022,
                0.590,
                24,
                blade_thickness=0.0026,
                blade_sweep_deg=32.0,
                backplate=True,
                shroud=True,
            ),
            "squirrel_cage_blower",
        ),
        origin=Origin(),
        material=blue_gray,
        name="squirrel_cage_blower",
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark,
        name="timer_shaft",
    )
    timer_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.025,
                body_style="skirted",
                top_diameter=0.034,
                edge_radius=0.001,
                grip=KnobGrip(style="fluted", count=18, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "timer_knob_cap",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=light_button,
        name="timer_knob_cap",
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark,
        name="speed_shaft",
    )
    speed_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.038,
                0.023,
                body_style="faceted",
                top_diameter=0.028,
                edge_radius=0.0008,
                grip=KnobGrip(style="ribbed", count=12, depth=0.0008, width=0.0015),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=20.0),
                center=False,
            ),
            "speed_knob_cap",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=light_button,
        name="speed_knob_cap",
    )

    button = model.part("osc_button")
    button.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark,
        name="button_stem",
    )
    button.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=light_button,
        name="button_cap",
    )

    body_joint = model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.80, upper=0.80),
    )
    model.articulation(
        "body_to_deck",
        ArticulationType.FIXED,
        parent=body,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, 0.835)),
    )
    model.articulation(
        "body_to_blower",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower,
        origin=Origin(xyz=(0.0, -0.006, 0.430)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=45.0),
    )
    model.articulation(
        "deck_to_timer",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=timer_dial,
        origin=Origin(xyz=(-0.043, 0.010, 0.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )
    model.articulation(
        "deck_to_speed",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=speed_dial,
        origin=Origin(xyz=(0.020, 0.010, 0.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )
    model.articulation(
        "deck_to_button",
        ArticulationType.PRISMATIC,
        parent=deck,
        child=button,
        origin=Origin(xyz=(0.057, -0.027, 0.025)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.10, lower=0.0, upper=0.008),
    )

    body_joint.meta["description"] = "Limited vertical oscillation joint between base and fan tower."
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    body = object_model.get_part("body_shell")
    deck = object_model.get_part("control_deck")
    blower = object_model.get_part("blower_wheel")
    timer = object_model.get_part("timer_dial")
    speed = object_model.get_part("speed_dial")
    button = object_model.get_part("osc_button")
    oscillation = object_model.get_articulation("base_to_body")
    button_slide = object_model.get_articulation("deck_to_button")

    ctx.allow_overlap(
        body,
        blower,
        elem_a="lower_blower_bearing",
        elem_b="blower_axle",
        reason="The spinning blower shaft is intentionally captured inside the lower bearing proxy.",
    )
    ctx.allow_overlap(
        body,
        blower,
        elem_a="upper_blower_bearing",
        elem_b="blower_axle",
        reason="The spinning blower shaft is intentionally captured inside the upper bearing proxy.",
    )

    ctx.expect_gap(deck, body, axis="z", max_gap=0.003, max_penetration=0.002, name="separate control deck sits on tower shell")
    ctx.expect_gap(timer, deck, axis="z", max_gap=0.004, max_penetration=0.001, name="timer dial is supported by the deck")
    ctx.expect_gap(speed, deck, axis="z", max_gap=0.004, max_penetration=0.001, name="speed dial is supported by the deck")
    ctx.expect_gap(button, deck, axis="z", max_gap=0.004, max_penetration=0.001, name="oscillation button is supported by the deck")
    ctx.expect_origin_distance(timer, speed, axes="xy", min_dist=0.050, max_dist=0.075, name="timer knob is beside speed dial")
    ctx.expect_origin_distance(button, speed, axes="xy", min_dist=0.035, max_dist=0.070, name="oscillation button is a separate nearby control")
    ctx.expect_within(blower, body, axes="xy", margin=0.005, name="blower wheel fits inside the tower shell footprint")
    ctx.expect_overlap(blower, body, axes="z", min_overlap=0.45, name="blower wheel spans the tall outlet region")
    ctx.expect_within(
        blower,
        body,
        axes="xy",
        inner_elem="blower_axle",
        outer_elem="lower_blower_bearing",
        margin=0.001,
        name="lower bearing captures the blower axle center",
    )
    ctx.expect_overlap(
        blower,
        body,
        axes="z",
        elem_a="blower_axle",
        elem_b="lower_blower_bearing",
        min_overlap=0.010,
        name="blower axle is retained in lower bearing",
    )
    ctx.expect_within(
        blower,
        body,
        axes="xy",
        inner_elem="blower_axle",
        outer_elem="upper_blower_bearing",
        margin=0.001,
        name="upper bearing captures the blower axle center",
    )
    ctx.expect_overlap(
        blower,
        body,
        axes="z",
        elem_a="blower_axle",
        elem_b="upper_blower_bearing",
        min_overlap=0.010,
        name="blower axle is retained in upper bearing",
    )

    base_box = ctx.part_world_aabb(base)
    body_box = ctx.part_world_aabb(body)
    ctx.check(
        "bedroom appliance tower proportions",
        base_box is not None
        and body_box is not None
        and (body_box[1][2] - base_box[0][2]) > 0.85
        and (base_box[1][0] - base_box[0][0]) > 0.30,
        details=f"base_aabb={base_box}, body_aabb={body_box}",
    )

    at_rest = ctx.part_world_position(button)
    with ctx.pose({button_slide: 0.008}):
        pressed = ctx.part_world_position(button)
        ctx.expect_gap(
            button,
            deck,
            axis="z",
            max_gap=0.004,
            max_penetration=0.009,
            name="button depresses into its deck travel",
        )
    ctx.check(
        "oscillation button moves downward",
        at_rest is not None and pressed is not None and pressed[2] < at_rest[2] - 0.006,
        details=f"rest={at_rest}, pressed={pressed}",
    )

    with ctx.pose({oscillation: 0.80}):
        swept_body = ctx.part_world_aabb(body)
    ctx.check(
        "body oscillates about the vertical base joint",
        swept_body is not None and body_box is not None and abs(swept_body[1][0] - body_box[1][0]) > 0.005,
        details=f"rest={body_box}, oscillated={swept_body}",
    )

    return ctx.report()


object_model = build_object_model()
