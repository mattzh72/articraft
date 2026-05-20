from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_W = 0.56
BODY_D = 0.44
BODY_H = 0.32
FRONT_Y = -BODY_D / 2.0 - 0.006

DOOR_W = 0.390
DOOR_H = 0.290
DOOR_T = 0.034
DOOR_X0 = -0.260
DOOR_Z0 = 0.015
HINGE_X = DOOR_X0
HINGE_Y = FRONT_Y - 0.003
HINGE_Z = DOOR_Z0

PANEL_X = 0.205
PANEL_W = 0.130
PANEL_Z = 0.163
PANEL_H = 0.285
PANEL_FACE_Y = FRONT_Y - 0.010


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name, rgba=rgba)


def _add_button(
    model: ArticulatedObject,
    body,
    name: str,
    *,
    x: float,
    z: float,
    size: tuple[float, float],
    color: Material,
    label_style: str = "bar",
):
    """Add a front-panel push button whose positive travel presses inward."""

    width, height = size
    depth = 0.0075
    button = model.part(name)
    button.visual(
        Box((width, depth, height)),
        origin=Origin(xyz=(0.0, -depth / 2.0, 0.0)),
        material=color,
        name="cap",
    )

    # Small raised legends are simple geometric cues rather than free-floating
    # decals; they touch the cap's front face.
    if label_style == "dot":
        button.visual(
            Cylinder(radius=min(width, height) * 0.080, length=0.0012),
            origin=Origin(xyz=(0.0, -depth - 0.0006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="button_legend",
            name="legend_dot",
        )
    elif label_style == "double":
        for dz in (-height * 0.16, height * 0.16):
            button.visual(
                Box((width * 0.45, 0.0012, 0.0018)),
                origin=Origin(xyz=(0.0, -depth - 0.0006, dz)),
                material="button_legend",
                name=f"legend_{'low' if dz < 0.0 else 'high'}",
            )
    else:
        button.visual(
            Box((width * 0.52, 0.0012, 0.0018)),
            origin=Origin(xyz=(0.0, -depth - 0.0006, 0.0)),
            material="button_legend",
            name="legend_bar",
        )

    model.articulation(
        f"body_to_{name}",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(x, PANEL_FACE_Y, z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.18, lower=0.0, upper=0.0045),
    )
    return button


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="detailed_microwave_oven")

    stainless = model.material("brushed_stainless", rgba=(0.68, 0.70, 0.68, 1.0))
    dark_plastic = model.material("gloss_black_plastic", rgba=(0.012, 0.014, 0.016, 1.0))
    black = model.material("black_shadow", rgba=(0.0, 0.0, 0.0, 1.0))
    enamel = model.material("warm_white_enamel", rgba=(0.86, 0.84, 0.78, 1.0))
    glass = model.material("smoked_glass", rgba=(0.06, 0.09, 0.10, 0.42))
    screen = model.material("black_perforated_screen", rgba=(0.015, 0.018, 0.018, 0.82))
    rubber = model.material("soft_black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    button_gray = model.material("satin_gray_buttons", rgba=(0.58, 0.59, 0.56, 1.0))
    start_green = model.material("start_green", rgba=(0.13, 0.48, 0.24, 1.0))
    stop_red = model.material("stop_red", rgba=(0.56, 0.10, 0.08, 1.0))
    model.material("button_legend", rgba=(0.02, 0.02, 0.018, 1.0))
    display_glow = model.material("display_glow", rgba=(0.05, 0.75, 0.86, 1.0))

    body = model.part("body")

    wall = 0.018
    # Connected sheet-metal appliance shell, open at the front cooking cavity.
    body.visual(Box((BODY_W, BODY_D, wall)), origin=Origin(xyz=(0.0, 0.0, wall / 2.0)), material=stainless, name="bottom_shell")
    body.visual(Box((BODY_W, BODY_D, wall)), origin=Origin(xyz=(0.0, 0.0, BODY_H - wall / 2.0)), material=stainless, name="top_shell")
    body.visual(Box((wall, BODY_D, BODY_H)), origin=Origin(xyz=(-BODY_W / 2.0 + wall / 2.0, 0.0, BODY_H / 2.0)), material=stainless, name="side_shell_0")
    body.visual(Box((wall, BODY_D, BODY_H)), origin=Origin(xyz=(BODY_W / 2.0 - wall / 2.0, 0.0, BODY_H / 2.0)), material=stainless, name="side_shell_1")
    body.visual(Box((BODY_W, wall, BODY_H)), origin=Origin(xyz=(0.0, BODY_D / 2.0 - wall / 2.0, BODY_H / 2.0)), material=stainless, name="rear_shell")

    # Front fascia around the cooking cavity and the distinct control column.
    face_t = 0.012
    face_center_y = FRONT_Y + face_t / 2.0
    body.visual(Box((0.420, face_t, 0.032)), origin=Origin(xyz=(-0.065, face_center_y, 0.304)), material=stainless, name="front_frame_top")
    body.visual(Box((0.420, face_t, 0.032)), origin=Origin(xyz=(-0.065, face_center_y, 0.016)), material=stainless, name="front_frame_bottom")
    body.visual(Box((0.030, face_t, BODY_H)), origin=Origin(xyz=(-0.270, face_center_y, BODY_H / 2.0)), material=stainless, name="front_frame_hinge")
    body.visual(Box((0.026, face_t, BODY_H)), origin=Origin(xyz=(0.135, face_center_y, BODY_H / 2.0)), material=stainless, name="front_divider")
    body.visual(Box((PANEL_W, 0.010, PANEL_H)), origin=Origin(xyz=(PANEL_X, PANEL_FACE_Y + 0.005, PANEL_Z)), material=dark_plastic, name="control_panel")
    body.visual(Box((0.008, 0.016, DOOR_H * 0.86)), origin=Origin(xyz=(HINGE_X - 0.006, HINGE_Y - 0.004, HINGE_Z + DOOR_H / 2.0)), material=black, name="hinge_socket")

    # Interior oven cavity is visible when the door swings open.
    body.visual(Box((0.372, 0.034, 0.224)), origin=Origin(xyz=(-0.065, 0.186, 0.166)), material=enamel, name="cavity_back")
    body.visual(Box((0.374, 0.355, 0.043)), origin=Origin(xyz=(-0.065, -0.030, 0.0395)), material=enamel, name="cavity_floor")
    body.visual(Box((0.374, 0.355, 0.047)), origin=Origin(xyz=(-0.065, -0.030, 0.2965)), material=enamel, name="cavity_ceiling")
    body.visual(Box((0.006, 0.355, 0.224)), origin=Origin(xyz=(-0.254, -0.030, 0.166)), material=enamel, name="cavity_wall_0")
    body.visual(Box((0.006, 0.355, 0.224)), origin=Origin(xyz=(0.124, -0.030, 0.166)), material=enamel, name="cavity_wall_1")
    body.visual(Cylinder(radius=0.018, length=0.006), origin=Origin(xyz=(-0.065, -0.030, 0.064)), material=rubber, name="turntable_hub")

    # Appliance details: vents, feet, display, screw caps, and panel seams.
    top_vent = SlotPatternPanelGeometry(
        (0.260, 0.070),
        0.003,
        slot_size=(0.026, 0.006),
        pitch=(0.034, 0.014),
        frame=0.010,
        corner_radius=0.004,
        stagger=True,
    )
    body.visual(mesh_from_geometry(top_vent, "top_vent_slots"), origin=Origin(xyz=(-0.065, 0.020, BODY_H + 0.0015)), material=black, name="top_vent")
    side_vent = SlotPatternPanelGeometry(
        (0.220, 0.080),
        0.003,
        slot_size=(0.022, 0.005),
        pitch=(0.030, 0.014),
        frame=0.010,
        corner_radius=0.004,
        slot_angle_deg=0.0,
    )
    body.visual(
        mesh_from_geometry(side_vent, "side_vent_slots"),
        origin=Origin(xyz=(BODY_W / 2.0 + 0.0015, -0.005, 0.205), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="side_vent",
    )
    body.visual(Box((0.105, 0.004, 0.030)), origin=Origin(xyz=(PANEL_X, PANEL_FACE_Y - 0.0020, 0.274)), material=display_glow, name="display_window")
    body.visual(Box((0.112, 0.002, 0.037)), origin=Origin(xyz=(PANEL_X, PANEL_FACE_Y - 0.0035, 0.274)), material=glass, name="display_lens")
    for i, x in enumerate((-0.205, 0.205)):
        for j, y in enumerate((-0.145, 0.145)):
            body.visual(Cylinder(radius=0.018, length=0.010), origin=Origin(xyz=(x, y, -0.005)), material=rubber, name=f"foot_{i}_{j}")

    # Door: a real trim frame with a transparent screened window and a proud pull handle.
    door = model.part("door")
    door_frame_mesh = mesh_from_geometry(
        BezelGeometry(
            (0.275, 0.166),
            (DOOR_W, DOOR_H),
            DOOR_T,
            opening_shape="rounded_rect",
            outer_shape="rounded_rect",
            opening_corner_radius=0.015,
            outer_corner_radius=0.018,
            face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.002),
            center=False,
        ),
        "door_frame_mesh",
    )
    door.visual(
        door_frame_mesh,
        origin=Origin(xyz=(DOOR_W / 2.0, 0.0, DOOR_H / 2.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="door_frame",
    )
    door.visual(Box((0.292, 0.003, 0.182)), origin=Origin(xyz=(DOOR_W / 2.0, -0.020, DOOR_H / 2.0)), material=glass, name="window_glass")
    screen_mesh = mesh_from_geometry(
        PerforatedPanelGeometry(
            (0.300, 0.190),
            0.002,
            hole_diameter=0.0045,
            pitch=(0.010, 0.010),
            frame=0.010,
            corner_radius=0.010,
            stagger=True,
        ),
        "door_screen_mesh",
    )
    door.visual(screen_mesh, origin=Origin(xyz=(DOOR_W / 2.0, -0.017, DOOR_H / 2.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=screen, name="window_screen")
    door.visual(Box((0.010, 0.025, 0.210)), origin=Origin(xyz=(DOOR_W - 0.042, -0.055, DOOR_H / 2.0)), material=dark_plastic, name="handle_grip")
    for z in (DOOR_H * 0.30, DOOR_H * 0.70):
        door.visual(Box((0.030, 0.026, 0.018)), origin=Origin(xyz=(DOOR_W - 0.052, -0.043, z)), material=dark_plastic, name=f"handle_standoff_{int(z * 1000)}")
    door.visual(Box((0.010, 0.006, DOOR_H * 0.84)), origin=Origin(xyz=(-0.005, -0.010, DOOR_H / 2.0)), material=black, name="hinge_shadow")
    door.visual(Box((0.004, 0.024, DOOR_H * 0.80)), origin=Origin(xyz=(0.001, -0.022, DOOR_H / 2.0)), material=black, name="hinge_leaf")

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.85),
    )

    # Rotating glass turntable inside the cavity.
    turntable = model.part("turntable")
    turntable.visual(Cylinder(radius=0.137, length=0.008), origin=Origin(), material=Material("slightly_green_glass", rgba=(0.70, 0.88, 0.86, 0.48)), name="glass_plate")
    turntable.visual(Cylinder(radius=0.022, length=0.011), origin=Origin(), material=glass, name="center_boss")
    model.articulation(
        "body_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=turntable,
        origin=Origin(xyz=(-0.065, -0.030, 0.071)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.65),
    )

    # Keypad grid and larger cooking controls.
    key_xs = (PANEL_X - 0.038, PANEL_X, PANEL_X + 0.038)
    key_zs = (0.190, 0.160, 0.130, 0.100)
    for row, z in enumerate(key_zs):
        for col, x in enumerate(key_xs):
            _add_button(
                model,
                body,
                f"key_{row}_{col}",
                x=x,
                z=z,
                size=(0.028, 0.020),
                color=button_gray,
                label_style="dot" if row == 3 and col == 1 else "bar",
            )

    _add_button(model, body, "start_button", x=PANEL_X - 0.024, z=0.066, size=(0.044, 0.024), color=start_green, label_style="double")
    _add_button(model, body, "stop_button", x=PANEL_X + 0.030, z=0.066, size=(0.044, 0.024), color=stop_red, label_style="double")
    _add_button(model, body, "door_release", x=PANEL_X, z=0.032, size=(0.088, 0.020), color=button_gray, label_style="bar")

    # A separate timer/power dial rotates about the front-panel normal.
    timer_dial = model.part("timer_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.046,
            0.021,
            body_style="faceted",
            base_diameter=0.050,
            top_diameter=0.038,
            edge_radius=0.0012,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0012, width=0.002),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "timer_dial_mesh",
    )
    timer_dial.visual(dial_mesh, origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=button_gray, name="dial_cap")
    model.articulation(
        "body_to_timer_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=timer_dial,
        origin=Origin(xyz=(PANEL_X, PANEL_FACE_Y, 0.234)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-2.7, upper=2.7),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    start = object_model.get_part("start_button")
    dial = object_model.get_part("timer_dial")
    turntable = object_model.get_part("turntable")
    door_hinge = object_model.get_articulation("body_to_door")
    start_slide = object_model.get_articulation("body_to_start_button")

    ctx.allow_overlap(
        body,
        door,
        elem_a="hinge_socket",
        elem_b="hinge_shadow",
        reason="The closed door's hinge leaf is intentionally captured in the body-side hinge socket.",
    )
    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="hinge_socket",
        negative_elem="hinge_shadow",
        max_penetration=0.006,
        name="door hinge is captured without broad door collision",
    )

    # The door is a close-fitting front appliance door: nearly flush to the
    # fascia, with a visible overlap footprint but no solid intersection at rest.
    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="front_frame_top",
        negative_elem="door_frame",
        min_gap=0.001,
        max_gap=0.006,
        name="closed door sits just proud of the fascia",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        elem_a="door_frame",
        elem_b="front_frame_top",
        min_overlap=0.012,
        name="door frame covers the cooking-cavity front",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_frame")
    with ctx.pose({door_hinge: 1.25}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_frame")
    ctx.check(
        "door hinge swings outward",
        closed_aabb is not None and open_aabb is not None and open_aabb[0][1] < closed_aabb[0][1] - 0.14,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    ctx.expect_gap(
        body,
        start,
        axis="y",
        positive_elem="control_panel",
        negative_elem="cap",
        min_gap=0.0,
        max_gap=0.001,
        name="start button is seated on the control panel",
    )
    start_rest = ctx.part_world_position(start)
    with ctx.pose({start_slide: 0.0045}):
        start_pressed = ctx.part_world_position(start)
    ctx.check(
        "start button presses inward",
        start_rest is not None and start_pressed is not None and start_pressed[1] > start_rest[1] + 0.003,
        details=f"rest={start_rest}, pressed={start_pressed}",
    )

    ctx.expect_gap(
        body,
        dial,
        axis="y",
        positive_elem="control_panel",
        negative_elem="dial_cap",
        min_gap=0.0,
        max_gap=0.001,
        name="timer dial is mounted proud of the panel",
    )
    ctx.expect_gap(
        turntable,
        body,
        axis="z",
        positive_elem="glass_plate",
        negative_elem="cavity_floor",
        min_gap=0.002,
        max_gap=0.015,
        name="turntable clears the cavity floor",
    )

    return ctx.report()


object_model = build_object_model()
