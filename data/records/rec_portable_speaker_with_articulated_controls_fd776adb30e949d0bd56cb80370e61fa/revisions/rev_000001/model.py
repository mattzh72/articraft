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
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


WIDTH = 0.34
DEPTH = 0.24
HEIGHT = 0.62
FRONT_Y = -DEPTH / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_party_speaker")

    body_mat = model.material("slightly_satin_black_plastic", rgba=(0.018, 0.020, 0.023, 1.0))
    bumper_mat = model.material("soft_charcoal_rubber", rgba=(0.005, 0.006, 0.007, 1.0))
    grille_mat = model.material("black_perforated_steel", rgba=(0.0, 0.0, 0.0, 1.0))
    panel_mat = model.material("recessed_control_black", rgba=(0.010, 0.012, 0.014, 1.0))
    knob_mat = model.material("dark_gunmetal_knob", rgba=(0.11, 0.12, 0.13, 1.0))
    button_mat = model.material("matte_control_buttons", rgba=(0.032, 0.035, 0.038, 1.0))
    indicator_mat = model.material("soft_white_markings", rgba=(0.82, 0.86, 0.88, 1.0))

    enclosure = model.part("enclosure")

    shell = (
        cq.Workplane("XY")
        .box(WIDTH, DEPTH, HEIGHT)
        .edges("|Z")
        .fillet(0.032)
        .edges(">Z")
        .fillet(0.012)
        .edges("<Z")
        .fillet(0.010)
    )
    enclosure.visual(
        mesh_from_cadquery(shell, "rounded_speaker_shell", tolerance=0.0015),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT / 2.0)),
        material=body_mat,
        name="rounded_shell",
    )

    # Protective portable-speaker corner rails and a shallow front equipment panel.
    for x, name in ((-0.158, "front_rail_0"), (0.158, "front_rail_1")):
        enclosure.visual(
            Box((0.020, 0.016, 0.530)),
            origin=Origin(xyz=(x, FRONT_Y - 0.004, 0.315)),
            material=bumper_mat,
            name=name,
        )

    enclosure.visual(
        Box((0.308, 0.010, 0.385)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.002, 0.255)),
        material=panel_mat,
        name="front_recess",
    )

    main_grille = PerforatedPanelGeometry(
        (0.284, 0.310),
        0.006,
        hole_diameter=0.008,
        pitch=(0.014, 0.014),
        frame=0.014,
        corner_radius=0.016,
        stagger=True,
        center=False,
    )
    enclosure.visual(
        mesh_from_geometry(main_grille, "main_perforated_grille"),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.002, 0.228), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grille_mat,
        name="main_grille",
    )

    # Control strip above the grille, with separate movable controls mounted to it.
    enclosure.visual(
        Box((0.250, 0.007, 0.074)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.005, 0.425)),
        material=panel_mat,
        name="button_strip",
    )
    enclosure.visual(
        Cylinder(radius=0.057, length=0.007),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0035, 0.512), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=panel_mat,
        name="knob_recess",
    )
    enclosure.visual(
        Cylinder(radius=0.005, length=0.003),
        origin=Origin(xyz=(0.112, FRONT_Y - 0.0065, 0.455), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=indicator_mat,
        name="status_led",
    )

    # Top recess and hinge saddles for the rotating carrying handle.
    enclosure.visual(
        Box((0.278, 0.170, 0.006)),
        origin=Origin(xyz=(0.0, -0.002, HEIGHT + 0.003)),
        material=panel_mat,
        name="handle_tray",
    )
    for x, name in ((-0.116, "handle_saddle_0"), (0.116, "handle_saddle_1")):
        enclosure.visual(
            Box((0.060, 0.040, 0.016)),
            origin=Origin(xyz=(x, -0.055, HEIGHT + 0.007)),
            material=bumper_mat,
            name=name,
        )

    # Small rubber feet keep it visually portable, not a fixed cabinet.
    for x, y, name in (
        (-0.105, -0.066, "foot_0"),
        (0.105, -0.066, "foot_1"),
        (-0.105, 0.066, "foot_2"),
        (0.105, 0.066, "foot_3"),
    ):
        enclosure.visual(
            Box((0.070, 0.045, 0.014)),
            origin=Origin(xyz=(x, y, 0.003)),
            material=bumper_mat,
            name=name,
        )

    handle = model.part("top_handle")
    handle.visual(
        Box((0.052, 0.028, 0.026)),
        origin=Origin(xyz=(-0.116, 0.0, 0.0)),
        material=bumper_mat,
        name="hinge_lug_0",
    )
    handle.visual(
        Box((0.052, 0.028, 0.026)),
        origin=Origin(xyz=(0.116, 0.0, 0.0)),
        material=bumper_mat,
        name="hinge_lug_1",
    )
    for x, name in ((-0.116, "handle_arm_0"), (0.116, "handle_arm_1")):
        handle.visual(
            Box((0.026, 0.134, 0.020)),
            origin=Origin(xyz=(x, 0.070, 0.006)),
            material=bumper_mat,
            name=name,
        )
    handle.visual(
        Box((0.262, 0.034, 0.024)),
        origin=Origin(xyz=(0.0, 0.142, 0.008)),
        material=bumper_mat,
        name="grip",
    )

    volume_knob = model.part("volume_knob")
    knob_shape = KnobGeometry(
        0.082,
        0.034,
        body_style="faceted",
        top_diameter=0.066,
        base_diameter=0.086,
        edge_radius=0.0012,
        grip=KnobGrip(style="ribbed", count=24, depth=0.0011, width=0.0022),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=90.0),
        center=False,
    )
    volume_knob.visual(
        mesh_from_geometry(knob_shape, "faceted_volume_knob"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_mat,
        name="knob_cap",
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Box((0.058, 0.012, 0.032)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=button_mat,
        name="rocker_cap",
    )
    power_rocker.visual(
        Box((0.030, 0.0015, 0.003)),
        origin=Origin(xyz=(0.0, -0.01275, 0.009)),
        material=indicator_mat,
        name="power_mark",
    )

    for i in range(2):
        menu_button = model.part(f"menu_button_{i}")
        menu_button.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=button_mat,
            name="button_cap",
        )
        menu_button.visual(
            Cylinder(radius=0.0032, length=0.0015),
            origin=Origin(xyz=(0.0, -0.01075, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=indicator_mat,
            name="button_dot",
        )

    control_surface_y = FRONT_Y - 0.0085
    model.articulation(
        "enclosure_to_handle",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=handle,
        origin=Origin(xyz=(0.0, -0.055, HEIGHT + 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.30),
    )
    model.articulation(
        "enclosure_to_volume_knob",
        ArticulationType.CONTINUOUS,
        parent=enclosure,
        child=volume_knob,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0065, 0.512)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=8.0),
    )
    model.articulation(
        "enclosure_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=power_rocker,
        origin=Origin(xyz=(-0.074, control_surface_y, 0.425)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=-0.22, upper=0.22),
    )
    for i, x in enumerate((0.012, 0.058)):
        model.articulation(
            f"enclosure_to_menu_button_{i}",
            ArticulationType.PRISMATIC,
            parent=enclosure,
            child=f"menu_button_{i}",
            origin=Origin(xyz=(x, control_surface_y, 0.425)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=0.06, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    enclosure = object_model.get_part("enclosure")
    handle = object_model.get_part("top_handle")
    knob = object_model.get_part("volume_knob")
    rocker = object_model.get_part("power_rocker")
    menu_0 = object_model.get_part("menu_button_0")
    menu_1 = object_model.get_part("menu_button_1")

    handle_joint = object_model.get_articulation("enclosure_to_handle")
    knob_joint = object_model.get_articulation("enclosure_to_volume_knob")
    rocker_joint = object_model.get_articulation("enclosure_to_power_rocker")
    menu_0_joint = object_model.get_articulation("enclosure_to_menu_button_0")
    menu_1_joint = object_model.get_articulation("enclosure_to_menu_button_1")

    ctx.expect_contact(
        handle,
        enclosure,
        elem_a="hinge_lug_0",
        elem_b="handle_saddle_0",
        contact_tol=0.001,
        name="handle hinge lug rests on its saddle",
    )
    ctx.expect_gap(
        knob,
        rocker,
        axis="z",
        min_gap=0.015,
        positive_elem="knob_cap",
        negative_elem="rocker_cap",
        name="volume knob sits above the front button row",
    )
    ctx.expect_gap(
        rocker,
        enclosure,
        axis="z",
        min_gap=0.015,
        positive_elem="rocker_cap",
        negative_elem="main_grille",
        name="button row is above the main grille",
    )
    ctx.expect_gap(
        menu_0,
        rocker,
        axis="x",
        min_gap=0.010,
        positive_elem="button_cap",
        negative_elem="rocker_cap",
        name="power rocker is distinct from menu buttons",
    )
    ctx.expect_gap(
        menu_1,
        menu_0,
        axis="x",
        min_gap=0.006,
        positive_elem="button_cap",
        negative_elem="button_cap",
        name="two menu buttons are independently separated",
    )

    ctx.check(
        "volume knob is continuous rotary",
        getattr(knob_joint, "articulation_type", None) == ArticulationType.CONTINUOUS
        and tuple(getattr(knob_joint, "axis", ())) == (0.0, -1.0, 0.0),
        details=f"type={getattr(knob_joint, 'articulation_type', None)}, axis={getattr(knob_joint, 'axis', None)}",
    )

    rest_grip = ctx.part_element_world_aabb(handle, elem="grip")
    with ctx.pose({handle_joint: 1.10}):
        raised_grip = ctx.part_element_world_aabb(handle, elem="grip")
    ctx.check(
        "top handle rotates upward",
        rest_grip is not None
        and raised_grip is not None
        and raised_grip[1][2] > rest_grip[1][2] + 0.060,
        details=f"rest={rest_grip}, raised={raised_grip}",
    )

    rest_menu_y = ctx.part_world_position(menu_0)
    with ctx.pose({menu_0_joint: 0.006, menu_1_joint: 0.006}):
        pressed_menu_y = ctx.part_world_position(menu_0)
    ctx.check(
        "menu buttons depress inward independently",
        rest_menu_y is not None
        and pressed_menu_y is not None
        and pressed_menu_y[1] > rest_menu_y[1] + 0.004,
        details=f"rest={rest_menu_y}, pressed={pressed_menu_y}",
    )

    rest_rocker = ctx.part_element_world_aabb(rocker, elem="rocker_cap")
    with ctx.pose({rocker_joint: 0.22}):
        tilted_rocker = ctx.part_element_world_aabb(rocker, elem="rocker_cap")
    ctx.check(
        "power rocker pivots on a short horizontal axis",
        rest_rocker is not None
        and tilted_rocker is not None
        and (tilted_rocker[1][1] - tilted_rocker[0][1]) > (rest_rocker[1][1] - rest_rocker[0][1]) + 0.003,
        details=f"rest={rest_rocker}, tilted={tilted_rocker}",
    )

    return ctx.report()


object_model = build_object_model()
