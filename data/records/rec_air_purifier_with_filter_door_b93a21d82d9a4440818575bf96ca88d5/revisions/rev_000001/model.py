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
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    SlotPatternPanelGeometry,
    Sphere,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.38
BODY_D = 0.245
BODY_H = 0.88
BODY_BOTTOM_Z = 0.02
BODY_CENTER_Z = BODY_BOTTOM_Z + BODY_H / 2.0
BODY_TOP_Z = BODY_BOTTOM_Z + BODY_H
FRONT_Y = -BODY_D / 2.0

DOOR_W = 0.30
DOOR_H = 0.56
DOOR_T = 0.017
DOOR_Z0 = 0.155
DOOR_HINGE_X = -DOOR_W / 2.0
DOOR_HINGE_Y = FRONT_Y - 0.006


def rounded_box_mesh(name: str, size: tuple[float, float, float], radius: float):
    """Small appliance-style rounded cuboid mesh, authored in meters."""
    shape = cq.Workplane("XY").box(*size)
    safe_radius = min(radius, min(size[0], size[1]) * 0.45)
    if safe_radius > 0.0005:
        shape = shape.edges("|Z").fillet(safe_radius)
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="polished_air_purifier")

    satin_white = model.material("satin_white", rgba=(0.93, 0.94, 0.92, 1.0))
    warm_white = model.material("warm_white_panel", rgba=(0.86, 0.88, 0.86, 1.0))
    charcoal = model.material("charcoal_black", rgba=(0.025, 0.027, 0.030, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.08, 0.085, 0.09, 1.0))
    soft_grey = model.material("soft_grey_trim", rgba=(0.55, 0.57, 0.57, 1.0))
    filter_media = model.material("filter_media", rgba=(0.72, 0.76, 0.74, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    blue_led = model.material("cool_blue_led", rgba=(0.10, 0.55, 1.0, 1.0))
    green_led = model.material("green_led", rgba=(0.18, 0.85, 0.32, 1.0))
    amber_led = model.material("amber_led", rgba=(1.0, 0.60, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        rounded_box_mesh("rounded_body_shell", (BODY_W, BODY_D, BODY_H), 0.035),
        origin=Origin(xyz=(0.0, 0.0, BODY_CENTER_Z)),
        material=satin_white,
        name="rounded_shell",
    )
    body.visual(
        Box((0.325, 0.190, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_plastic,
        name="recessed_plinth",
    )
    for idx, (x, y) in enumerate(
        ((-0.120, -0.065), (0.120, -0.065), (-0.120, 0.065), (0.120, 0.065))
    ):
        body.visual(
            Box((0.070, 0.038, 0.014)),
            origin=Origin(xyz=(x, y, 0.007)),
            material=rubber,
            name=f"rubber_foot_{idx}",
        )

    # A glossy control island and separate outlet grille on the top surface.
    body.visual(
        rounded_box_mesh("top_control_island", (0.315, 0.095, 0.006), 0.014),
        origin=Origin(xyz=(0.0, -0.060, BODY_TOP_Z + 0.003)),
        material=charcoal,
        name="top_control_island",
    )
    top_grille = VentGrilleGeometry(
        (0.292, 0.108),
        frame=0.014,
        face_thickness=0.004,
        duct_depth=0.020,
        duct_wall=0.003,
        slat_pitch=0.014,
        slat_width=0.006,
        slat_angle_deg=28.0,
        corner_radius=0.010,
        slats=VentGrilleSlats(profile="airfoil", direction="up", divider_count=2),
        frame_profile=VentGrilleFrame(style="beveled", depth=0.0015),
        sleeve=VentGrilleSleeve(style="short", depth=0.014, wall=0.003),
        center=False,
    )
    body.visual(
        mesh_from_geometry(top_grille, "top_outlet_grille"),
        origin=Origin(xyz=(0.0, 0.052, BODY_TOP_Z)),
        material=charcoal,
        name="top_outlet_grille",
    )

    # Shadowed front paneling makes the filter door read as an access bay.
    body.visual(
        rounded_box_mesh("front_filter_shadow", (0.328, 0.004, 0.594), 0.014),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.002, DOOR_Z0 + DOOR_H / 2.0)),
        material=dark_plastic,
        name="filter_recess_shadow",
    )
    body.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.238, 0.420),
                0.003,
                slot_size=(0.080, 0.008),
                pitch=(0.092, 0.020),
                frame=0.012,
                corner_radius=0.008,
            ),
            "pleated_filter_media",
        ),
        origin=Origin(
            xyz=(0.0, FRONT_Y - 0.0035, DOOR_Z0 + DOOR_H / 2.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=filter_media,
        name="pleated_filter_media",
    )
    body.visual(
        rounded_box_mesh("front_display_window", (0.205, 0.006, 0.048), 0.010),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.001, 0.777)),
        material=charcoal,
        name="front_display_window",
    )
    for idx, (x, mat) in enumerate(((-0.048, blue_led), (0.0, green_led), (0.048, amber_led))):
        body.visual(
            Sphere(0.006),
            origin=Origin(xyz=(x, FRONT_Y - 0.005, 0.778)),
            material=mat,
            name=f"status_led_{idx}",
        )

    side_panel_geom = PerforatedPanelGeometry(
        (0.152, 0.430),
        0.004,
        hole_diameter=0.008,
        pitch=(0.018, 0.018),
        frame=0.014,
        corner_radius=0.012,
        stagger=True,
    )
    side_panel_mesh = mesh_from_geometry(side_panel_geom, "side_intake_perforations")
    body.visual(
        side_panel_mesh,
        origin=Origin(
            xyz=(BODY_W / 2.0 + 0.001, 0.005, 0.440),
            rpy=(math.pi / 2.0, 0.0, math.pi / 2.0),
        ),
        material=dark_plastic,
        name="side_intake_panel_0",
    )
    body.visual(
        side_panel_mesh,
        origin=Origin(
            xyz=(-BODY_W / 2.0 - 0.001, 0.005, 0.440),
            rpy=(math.pi / 2.0, 0.0, -math.pi / 2.0),
        ),
        material=dark_plastic,
        name="side_intake_panel_1",
    )
    body.visual(
        Box((0.014, 0.020, DOOR_H + 0.050)),
        origin=Origin(xyz=(DOOR_HINGE_X - 0.007, FRONT_Y - 0.010, DOOR_Z0 + DOOR_H / 2.0)),
        material=soft_grey,
        name="body_hinge_socket",
    )

    # Hinged filter access door.  The child frame is the vertical hinge line;
    # the closed door panel extends along local +X and outward along local -Y.
    filter_door = model.part("filter_door")
    filter_door.visual(
        rounded_box_mesh("filter_door_panel", (DOOR_W, DOOR_T, DOOR_H), 0.018),
        origin=Origin(xyz=(DOOR_W / 2.0, -DOOR_T / 2.0, DOOR_H / 2.0)),
        material=warm_white,
        name="door_panel",
    )
    filter_door.visual(
        Cylinder(radius=0.008, length=DOOR_H + 0.045),
        origin=Origin(xyz=(0.0, -DOOR_T / 2.0, DOOR_H / 2.0)),
        material=soft_grey,
        name="hinge_barrel",
    )
    # Raised gasket/trim rails around the removable door, slightly seated into the panel.
    rail_y = -DOOR_T - 0.0005
    filter_door.visual(
        Box((DOOR_W - 0.030, 0.004, 0.010)),
        origin=Origin(xyz=(DOOR_W / 2.0, rail_y, DOOR_H - 0.022)),
        material=soft_grey,
        name="top_trim_rail",
    )
    filter_door.visual(
        Box((DOOR_W - 0.030, 0.004, 0.010)),
        origin=Origin(xyz=(DOOR_W / 2.0, rail_y, 0.022)),
        material=soft_grey,
        name="bottom_trim_rail",
    )
    filter_door.visual(
        Box((0.010, 0.004, DOOR_H - 0.050)),
        origin=Origin(xyz=(0.023, rail_y, DOOR_H / 2.0)),
        material=soft_grey,
        name="hinge_trim_rail",
    )
    filter_door.visual(
        Box((0.010, 0.004, DOOR_H - 0.050)),
        origin=Origin(xyz=(DOOR_W - 0.023, rail_y, DOOR_H / 2.0)),
        material=soft_grey,
        name="latch_trim_rail",
    )
    intake_slots = SlotPatternPanelGeometry(
        (0.232, 0.365),
        0.006,
        slot_size=(0.042, 0.008),
        pitch=(0.054, 0.026),
        frame=0.016,
        corner_radius=0.012,
        slot_angle_deg=0.0,
        stagger=True,
    )
    filter_door.visual(
        mesh_from_geometry(intake_slots, "front_filter_intake_slots"),
        origin=Origin(
            xyz=(DOOR_W / 2.0, -DOOR_T - 0.001, DOOR_H / 2.0 - 0.010),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=charcoal,
        name="intake_slot_panel",
    )
    filter_door.visual(
        rounded_box_mesh("door_finger_pull", (0.102, 0.008, 0.022), 0.006),
        origin=Origin(xyz=(DOOR_W / 2.0, -DOOR_T - 0.003, DOOR_H - 0.070)),
        material=charcoal,
        name="finger_pull",
    )
    filter_door.visual(
        Box((0.030, 0.007, 0.020)),
        origin=Origin(xyz=(DOOR_W - 0.020, -DOOR_T - 0.002, DOOR_H - 0.078)),
        material=soft_grey,
        name="latch_boss",
    )
    model.articulation(
        "body_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=filter_door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_Z0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=0.0, upper=1.75),
    )

    # Separate user controls: press buttons depress into the island, and the fan
    # dial rotates around a real shaft axis.
    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_plastic,
        name="button_cap",
    )
    power_button.visual(
        Cylinder(radius=0.007, length=0.0014),
        origin=Origin(xyz=(0.0, 0.0, 0.0107)),
        material=green_led,
        name="power_symbol_light",
    )
    model.articulation(
        "body_to_power_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_button,
        origin=Origin(xyz=(-0.123, -0.060, BODY_TOP_Z + 0.006)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.8, velocity=0.20, lower=0.0, upper=0.006),
    )

    for name, x, material in (
        ("mode_button", -0.048, blue_led),
        ("timer_button", 0.018, amber_led),
    ):
        button = model.part(name)
        button.visual(
            rounded_box_mesh(f"{name}_cap", (0.052, 0.026, 0.010), 0.007),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=dark_plastic,
            name="button_cap",
        )
        button.visual(
            Box((0.020, 0.004, 0.0012)),
            origin=Origin(xyz=(0.0, -0.003, 0.0102)),
            material=material,
            name="indicator_mark",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, -0.060, BODY_TOP_Z + 0.006)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.5, velocity=0.18, lower=0.0, upper=0.005),
        )

    fan_dial = model.part("fan_dial")
    fan_knob = KnobGeometry(
        0.060,
        0.030,
        body_style="domed",
        edge_radius=0.0012,
        grip=KnobGrip(style="ribbed", count=28, depth=0.0012, width=0.0020),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=70.0),
        center=False,
    )
    fan_dial.visual(
        mesh_from_geometry(fan_knob, "fan_speed_dial"),
        origin=Origin(),
        material=soft_grey,
        name="dial_cap",
    )
    fan_dial.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=dark_plastic,
        name="dial_shaft",
    )
    model.articulation(
        "body_to_fan_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=fan_dial,
        origin=Origin(xyz=(0.108, -0.060, BODY_TOP_Z + 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.4, lower=-2.2, upper=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("filter_door")
    fan_dial = object_model.get_part("fan_dial")
    door_hinge = object_model.get_articulation("body_to_filter_door")
    power_joint = object_model.get_articulation("body_to_power_button")
    dial_joint = object_model.get_articulation("body_to_fan_dial")

    ctx.allow_overlap(
        body,
        door,
        elem_a="body_hinge_socket",
        elem_b="hinge_barrel",
        reason="The simplified hinge socket intentionally captures the filter-door barrel on the pin line.",
    )
    ctx.allow_overlap(
        body,
        fan_dial,
        elem_a="top_control_island",
        elem_b="dial_shaft",
        reason="The rotary dial shaft is intentionally seated through the top control panel.",
    )

    ctx.expect_gap(
        body,
        door,
        axis="y",
        min_gap=0.002,
        max_gap=0.018,
        positive_elem="filter_recess_shadow",
        negative_elem="door_panel",
        name="closed filter door sits just proud of the body",
    )
    ctx.expect_overlap(
        body,
        door,
        axes="z",
        elem_a="body_hinge_socket",
        elem_b="hinge_barrel",
        min_overlap=0.50,
        name="filter door hinge barrel is captured by the body socket",
    )
    ctx.expect_overlap(
        body,
        fan_dial,
        axes="xy",
        elem_a="top_control_island",
        elem_b="dial_shaft",
        min_overlap=0.012,
        name="fan dial shaft is seated inside the top panel",
    )
    ctx.expect_overlap(
        body,
        door,
        axes="xz",
        min_overlap=0.24,
        name="filter door covers the front access bay",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.35}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "filter door swings outward from the front",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.08,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    rest_power = ctx.part_world_position("power_button")
    with ctx.pose({power_joint: 0.006}):
        pressed_power = ctx.part_world_position("power_button")
    ctx.check(
        "power button depresses downward",
        rest_power is not None
        and pressed_power is not None
        and pressed_power[2] < rest_power[2] - 0.004,
        details=f"rest={rest_power}, pressed={pressed_power}",
    )

    ctx.check(
        "fan dial has bounded rotary travel",
        dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower < -2.0
        and dial_joint.motion_limits.upper > 2.0,
        details=f"limits={dial_joint.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
