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
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_tower_fan")

    warm_white = Material("warm_white_plastic", rgba=(0.88, 0.90, 0.88, 1.0))
    satin_grey = Material("satin_grey_plastic", rgba=(0.42, 0.44, 0.45, 1.0))
    charcoal = Material("charcoal_black", rgba=(0.02, 0.023, 0.025, 1.0))
    dark_grille = Material("dark_recessed_grille", rgba=(0.015, 0.018, 0.020, 1.0))
    soft_black = Material("soft_black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    brushed_metal = Material("brushed_metal", rgba=(0.72, 0.72, 0.70, 1.0))
    button_grey = Material("button_grey", rgba=(0.68, 0.70, 0.70, 1.0))

    # Root pedestal: a broad circular residential fan foot with a raised,
    # visibly separate turntable bearing at the top.
    base = model.part("pedestal_base")
    base.visual(
        Cylinder(radius=0.180, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=satin_grey,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.075, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=charcoal,
        name="bearing_hub",
    )
    for index, (x, y) in enumerate(
        ((0.120, 0.085), (-0.120, 0.085), (0.120, -0.085), (-0.120, -0.085))
    ):
        base.visual(
            Cylinder(radius=0.020, length=0.004),
            origin=Origin(xyz=(x, y, 0.002)),
            material=soft_black,
            name=f"rubber_foot_{index}",
        )

    # The tower frame origin is the vertical oscillation bearing center on top
    # of the base, keeping the fan body visibly distinct from the pedestal.
    tower = model.part("tower_body")

    tower.visual(
        Cylinder(radius=0.054, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=charcoal,
        name="lower_neck",
    )
    tower.visual(
        Cylinder(radius=0.038, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=warm_white,
        name="center_pedestal",
    )
    tower.visual(
        Cylinder(radius=0.070, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=warm_white,
        name="body_foot",
    )

    # Main tall rectangular housing with rounded-corner posts and a dark outlet.
    tower.visual(
        Box((0.172, 0.145, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=warm_white,
        name="bottom_cap",
    )
    tower.visual(
        Box((0.018, 0.150, 0.690)),
        origin=Origin(xyz=(-0.084, 0.0, 0.475)),
        material=warm_white,
        name="side_rail_0",
    )
    tower.visual(
        Box((0.018, 0.150, 0.690)),
        origin=Origin(xyz=(0.084, 0.0, 0.475)),
        material=warm_white,
        name="side_rail_1",
    )
    tower.visual(
        Box((0.150, 0.012, 0.680)),
        origin=Origin(xyz=(0.0, 0.069, 0.475)),
        material=warm_white,
        name="rear_shell",
    )
    for index, (x, y) in enumerate(
        ((0.074, -0.064), (-0.074, -0.064), (0.074, 0.064), (-0.074, 0.064))
    ):
        tower.visual(
            Cylinder(radius=0.010, length=0.690),
            origin=Origin(xyz=(x, y, 0.475)),
            material=warm_white,
            name=f"corner_post_{index}",
        )

    front_grille_mesh = mesh_from_geometry(
        VentGrilleGeometry(
            (0.150, 0.670),
            frame=0.012,
            face_thickness=0.006,
            duct_depth=0.012,
            duct_wall=0.003,
            slat_pitch=0.026,
            slat_width=0.010,
            slat_angle_deg=16.0,
            corner_radius=0.010,
            slats=VentGrilleSlats(
                profile="airfoil",
                direction="up",
                inset=0.002,
                divider_count=2,
                divider_width=0.004,
            ),
            frame_profile=VentGrilleFrame(style="radiused", depth=0.0015),
            sleeve=VentGrilleSleeve(style="short", depth=0.012, wall=0.003),
        ),
        "front_grille",
    )
    tower.visual(
        front_grille_mesh,
        origin=Origin(xyz=(0.0, -0.070, 0.470), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grille,
        name="front_grille",
    )

    # Hidden bearing spiders keep the vertical squirrel-cage wheel mechanically
    # plausible without obstructing the outlet.
    tower.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=charcoal,
        name="lower_bearing",
    )
    tower.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.840)),
        material=charcoal,
        name="upper_bearing",
    )
    for z, suffix in ((0.090, "lower"), (0.833, "upper")):
        tower.visual(
            Box((0.128, 0.008, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=charcoal,
            name=f"{suffix}_spider_x",
        )
        tower.visual(
            Box((0.008, 0.118, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=charcoal,
            name=f"{suffix}_spider_y",
        )
    for z, suffix in ((0.145, "lower"), (0.785, "upper")):
        tower.visual(
            Box((0.014, 0.064, 0.008)),
            origin=Origin(xyz=(0.0, 0.037, z)),
            material=charcoal,
            name=f"{suffix}_bearing_pad",
        )

    tower.visual(
        Box((0.172, 0.145, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.815)),
        material=warm_white,
        name="upper_cap",
    )
    deck_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.185, 0.145, 0.032, corner_segments=10), 0.040),
        "control_deck",
    )
    tower.visual(
        deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.855)),
        material=warm_white,
        name="control_deck",
    )
    panel_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.148, 0.078, 0.022, corner_segments=8), 0.006),
        "control_plate",
    )
    tower.visual(
        panel_mesh,
        origin=Origin(xyz=(0.006, -0.018, 0.878)),
        material=charcoal,
        name="control_plate",
    )
    tower.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(-0.048, -0.018, 0.884)),
        material=satin_grey,
        name="speed_socket",
    )
    tower.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.004, -0.018, 0.884)),
        material=satin_grey,
        name="timer_socket",
    )
    tower.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.060, -0.018, 0.884)),
        material=satin_grey,
        name="button_plinth",
    )

    # Moving squirrel-cage blower inside the housing.
    blower = model.part("blower_wheel")
    blower_mesh = mesh_from_geometry(
        BlowerWheelGeometry(
            outer_radius=0.045,
            inner_radius=0.024,
            width=0.620,
            blade_count=30,
            blade_thickness=0.0022,
            blade_sweep_deg=24.0,
            backplate=True,
            shroud=True,
        ),
        "blower_cage",
    )
    blower.visual(
        blower_mesh,
        origin=Origin(),
        material=dark_grille,
        name="blower_cage",
    )
    blower.visual(
        Cylinder(radius=0.006, length=0.640),
        origin=Origin(),
        material=brushed_metal,
        name="blower_axle",
    )
    for z, suffix in ((-0.300, "lower"), (0.300, "upper")):
        blower.visual(
            Cylinder(radius=0.027, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=brushed_metal,
            name=f"{suffix}_hub_disk",
        )

    # Separate continuous rotary controls on the deck.
    knob_geometry = KnobGeometry(
        0.036,
        0.026,
        body_style="skirted",
        top_diameter=0.030,
        edge_radius=0.0012,
        skirt=KnobSkirt(0.044, 0.006, flare=0.06, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=18, depth=0.0010),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(knob_geometry, "speed_knob_cap"),
        origin=Origin(),
        material=satin_grey,
        name="knob_cap",
    )
    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        mesh_from_geometry(knob_geometry, "timer_knob_cap"),
        origin=Origin(),
        material=satin_grey,
        name="knob_cap",
    )

    # Independent small prismatic oscillation push button beside the dials.
    button = model.part("osc_button")
    button.visual(
        Cylinder(radius=0.0055, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=button_grey,
        name="button_stem",
    )
    button.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=button_grey,
        name="button_cap",
    )

    # Articulations.
    model.articulation(
        "base_to_tower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tower,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "tower_to_blower",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=blower,
        origin=Origin(xyz=(0.0, 0.0, 0.465)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=45.0),
    )
    model.articulation(
        "tower_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=speed_knob,
        origin=Origin(xyz=(-0.048, -0.018, 0.887)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=4.0),
    )
    model.articulation(
        "tower_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=timer_knob,
        origin=Origin(xyz=(0.004, -0.018, 0.887)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=4.0),
    )
    model.articulation(
        "tower_to_osc_button",
        ArticulationType.PRISMATIC,
        parent=tower,
        child=button,
        origin=Origin(xyz=(0.060, -0.018, 0.887)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.004),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("pedestal_base")
    tower = object_model.get_part("tower_body")
    blower = object_model.get_part("blower_wheel")
    speed_knob = object_model.get_part("speed_knob")
    timer_knob = object_model.get_part("timer_knob")
    button = object_model.get_part("osc_button")

    oscillation = object_model.get_articulation("base_to_tower")
    blower_spin = object_model.get_articulation("tower_to_blower")
    speed_spin = object_model.get_articulation("tower_to_speed_knob")
    timer_spin = object_model.get_articulation("tower_to_timer_knob")
    button_press = object_model.get_articulation("tower_to_osc_button")

    ctx.check(
        "tower uses limited vertical oscillation joint",
        oscillation.articulation_type == ArticulationType.REVOLUTE
        and oscillation.axis == (0.0, 0.0, 1.0),
        details=f"type={oscillation.articulation_type}, axis={oscillation.axis}",
    )
    ctx.check(
        "blower and rotary dials are continuous",
        blower_spin.articulation_type == ArticulationType.CONTINUOUS
        and speed_spin.articulation_type == ArticulationType.CONTINUOUS
        and timer_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"blower={blower_spin.articulation_type}, "
            f"speed={speed_spin.articulation_type}, timer={timer_spin.articulation_type}"
        ),
    )
    ctx.check(
        "oscillation button is a vertical prismatic push control",
        button_press.articulation_type == ArticulationType.PRISMATIC
        and button_press.axis == (0.0, 0.0, -1.0),
        details=f"type={button_press.articulation_type}, axis={button_press.axis}",
    )

    for suffix in ("lower", "upper"):
        ctx.allow_overlap(
            tower,
            blower,
            elem_a=f"{suffix}_bearing_pad",
            elem_b="blower_axle",
            reason=(
                "The hidden stationary bearing pad is modeled with a tiny local "
                "interference fit against the rotating blower axle."
            ),
        )
        ctx.expect_gap(
            tower,
            blower,
            axis="y",
            positive_elem=f"{suffix}_bearing_pad",
            negative_elem="blower_axle",
            max_gap=0.0005,
            max_penetration=0.0015,
            name=f"{suffix} blower bearing pad lightly captures hub",
        )

    ctx.expect_contact(
        tower,
        base,
        elem_a="lower_neck",
        elem_b="bearing_hub",
        contact_tol=0.001,
        name="tower collar sits on pedestal bearing",
    )
    ctx.expect_within(
        blower,
        tower,
        axes="xy",
        elem_a="blower_cage",
        name="blower wheel is contained inside tower footprint",
    )
    ctx.expect_contact(
        speed_knob,
        tower,
        elem_a="knob_cap",
        elem_b="speed_socket",
        contact_tol=0.0015,
        name="speed dial is seated on its socket",
    )
    ctx.expect_contact(
        timer_knob,
        tower,
        elem_a="knob_cap",
        elem_b="timer_socket",
        contact_tol=0.0015,
        name="timer dial is seated on its socket",
    )
    ctx.expect_contact(
        button,
        tower,
        elem_a="button_stem",
        elem_b="button_plinth",
        contact_tol=0.0015,
        name="oscillation button is supported by its plinth",
    )

    rest_button_position = ctx.part_world_position(button)
    with ctx.pose({button_press: 0.004}):
        pressed_button_position = ctx.part_world_position(button)
    ctx.check(
        "button travel moves downward",
        rest_button_position is not None
        and pressed_button_position is not None
        and pressed_button_position[2] < rest_button_position[2] - 0.003,
        details=f"rest={rest_button_position}, pressed={pressed_button_position}",
    )

    return ctx.report()


object_model = build_object_model()
