from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Small CadQuery helper for consumer-plastic rounded rectangular pieces."""
    solid = cq.Workplane("XY").box(size[0], size[1], size[2])
    if radius > 0.0:
        solid = solid.edges("|Z").fillet(radius)
    return solid


def _tower_shell_geometry() -> cq.Workplane:
    """One connected U-shaped column shell with an open front grille bay."""
    # Local tower frame: the vertical swivel axis is at (0, 0, 0).
    # The column stands above it; front is negative Y.
    back = _rounded_box((0.190, 0.020, 1.020), 0.007).translate((0.0, 0.060, 0.560))
    side_a = _rounded_box((0.030, 0.140, 1.020), 0.012).translate((-0.087, 0.0, 0.560))
    side_b = _rounded_box((0.030, 0.140, 1.020), 0.012).translate((0.087, 0.0, 0.560))
    bottom = _rounded_box((0.205, 0.140, 0.080), 0.014).translate((0.0, 0.0, 0.075))
    top = _rounded_box((0.205, 0.140, 0.100), 0.014).translate((0.0, 0.0, 1.065))

    shell = back.union(side_a).union(side_b).union(bottom).union(top)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_standing_tower_fan")

    warm_white = Material("warm_white_plastic", rgba=(0.86, 0.86, 0.82, 1.0))
    light_gray = Material("light_gray_plastic", rgba=(0.64, 0.65, 0.62, 1.0))
    dark_gray = Material("dark_gray_plastic", rgba=(0.10, 0.11, 0.12, 1.0))
    charcoal = Material("charcoal_grille", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_black = Material("satin_black_control", rgba=(0.02, 0.02, 0.022, 1.0))
    translucent_blue = Material("shadowed_blower", rgba=(0.08, 0.12, 0.16, 1.0))

    # Broad, stable floor base with a raised bearing pedestal for oscillation.
    base = model.part("round_base")
    base.visual(
        Cylinder(radius=0.185, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=warm_white,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.076, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=light_gray,
        name="bearing_pedestal",
    )
    base.visual(
        Cylinder(radius=0.132, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_gray,
        name="rubber_foot_ring",
    )

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=0.068, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=warm_white,
        name="lower_socket",
    )
    tower.visual(
        Box((0.190, 0.020, 1.020)),
        origin=Origin(xyz=(0.0, 0.060, 0.560)),
        material=warm_white,
        name="back_spine",
    )
    tower.visual(
        Box((0.030, 0.140, 1.020)),
        origin=Origin(xyz=(-0.087, 0.0, 0.560)),
        material=warm_white,
        name="side_shell_0",
    )
    tower.visual(
        Box((0.030, 0.140, 1.020)),
        origin=Origin(xyz=(0.087, 0.0, 0.560)),
        material=warm_white,
        name="side_shell_1",
    )
    tower.visual(
        Box((0.205, 0.140, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=warm_white,
        name="bottom_cap",
    )
    tower.visual(
        Box((0.205, 0.140, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 1.065)),
        material=warm_white,
        name="top_cap",
    )
    for x in (-0.091, 0.091):
        tower.visual(
            Cylinder(radius=0.014, length=1.020),
            origin=Origin(xyz=(x, -0.063, 0.560)),
            material=warm_white,
            name=f"front_edge_{0 if x < 0 else 1}",
        )

    grille = SlotPatternPanelGeometry(
        (0.162, 0.800),
        0.004,
        slot_size=(0.112, 0.0065),
        pitch=(0.126, 0.020),
        frame=0.012,
        corner_radius=0.014,
    )
    tower.visual(
        mesh_from_geometry(grille, "front_grille"),
        # Local grille XY becomes world XZ; local Z thickness points out the front.
        origin=Origin(xyz=(0.0, -0.0635, 0.555), rpy=(pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="front_grille",
    )
    tower.visual(
        Box((0.150, 0.012, 0.082)),
        origin=Origin(xyz=(0.0, -0.0715, 1.055)),
        material=light_gray,
        name="control_panel",
    )
    tower.visual(
        Cylinder(radius=0.032, length=0.006),
        origin=Origin(xyz=(-0.038, -0.080, 1.075), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="speed_collar",
    )
    tower.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.036, -0.080, 1.075), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="oscillation_collar",
    )

    blower = model.part("blower_wheel")
    blower.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                outer_radius=0.046,
                inner_radius=0.023,
                width=0.780,
                blade_count=34,
                blade_thickness=0.0022,
                blade_sweep_deg=28.0,
                backplate=True,
                shroud=True,
            ),
            "blower_wheel",
        ),
        origin=Origin(),
        material=translucent_blue,
        name="blower_wheel",
    )
    blower.visual(
        Cylinder(radius=0.0235, length=0.900),
        # The shaft ends bear against the solid bottom and top caps of the tower
        # shell, so the rotating blower reads as captured rather than floating.
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_gray,
        name="rotor_core",
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.024,
                body_style="skirted",
                top_diameter=0.036,
                skirt=KnobSkirt(0.052, 0.0055, flare=0.06, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=20, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                center=False,
            ),
            "speed_dial",
        ),
        origin=Origin(),
        material=satin_black,
        name="speed_cap",
    )

    oscillation_knob = model.part("oscillation_knob")
    oscillation_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.030,
                0.018,
                body_style="faceted",
                base_diameter=0.032,
                top_diameter=0.024,
                edge_radius=0.0007,
                grip=KnobGrip(style="ribbed", count=12, depth=0.0007, width=0.0014),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "oscillation_knob",
        ),
        origin=Origin(),
        material=satin_black,
        name="oscillation_cap",
    )

    model.articulation(
        "base_to_tower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tower,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.8, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "tower_to_blower",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=blower,
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=60.0),
    )
    model.articulation(
        "tower_to_speed_dial",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=speed_dial,
        origin=Origin(xyz=(-0.038, -0.083, 1.075), rpy=(pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )
    model.articulation(
        "tower_to_oscillation_knob",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=oscillation_knob,
        origin=Origin(xyz=(0.036, -0.083, 1.075), rpy=(pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0, lower=0.0, upper=1.5708),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("round_base")
    tower = object_model.get_part("tower")
    blower = object_model.get_part("blower_wheel")
    speed_dial = object_model.get_part("speed_dial")
    oscillation_knob = object_model.get_part("oscillation_knob")

    tower_joint = object_model.get_articulation("base_to_tower")
    blower_joint = object_model.get_articulation("tower_to_blower")
    speed_joint = object_model.get_articulation("tower_to_speed_dial")
    selector_joint = object_model.get_articulation("tower_to_oscillation_knob")

    ctx.check(
        "blower and speed dial spin continuously",
        blower_joint.articulation_type == ArticulationType.CONTINUOUS
        and speed_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"blower={blower_joint.articulation_type}, speed={speed_joint.articulation_type}",
    )
    ctx.check(
        "tower and selector are vertical revolute controls",
        tower_joint.articulation_type == ArticulationType.REVOLUTE
        and selector_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"tower={tower_joint.articulation_type}, selector={selector_joint.articulation_type}",
    )

    ctx.expect_contact(
        tower,
        base,
        elem_a="lower_socket",
        elem_b="bearing_pedestal",
        contact_tol=0.0015,
        name="tower socket rests on base bearing",
    )
    ctx.expect_overlap(
        blower,
        tower,
        axes="xz",
        elem_a="blower_wheel",
        elem_b="front_grille",
        min_overlap=0.09,
        name="blower wheel sits behind broad front grille",
    )
    ctx.expect_gap(
        tower,
        speed_dial,
        axis="y",
        positive_elem="speed_collar",
        negative_elem="speed_cap",
        max_gap=0.0015,
        max_penetration=0.001,
        name="speed dial is seated on its collar",
    )
    ctx.expect_gap(
        tower,
        oscillation_knob,
        axis="y",
        positive_elem="oscillation_collar",
        negative_elem="oscillation_cap",
        max_gap=0.0015,
        max_penetration=0.001,
        name="oscillation selector is seated on adjacent collar",
    )
    ctx.expect_origin_distance(
        speed_dial,
        oscillation_knob,
        axes="x",
        min_dist=0.060,
        max_dist=0.085,
        name="two rotary controls are adjacent",
    )

    rest_position = ctx.part_world_position(tower)
    with ctx.pose({tower_joint: 0.8, blower_joint: 1.0, speed_joint: 1.2, selector_joint: 1.0}):
        posed_position = ctx.part_world_position(tower)
        ctx.expect_contact(
            tower,
            base,
            elem_a="lower_socket",
            elem_b="bearing_pedestal",
            contact_tol=0.0015,
            name="oscillated tower remains on bearing",
        )

    ctx.check(
        "tower oscillates about fixed vertical bearing",
        rest_position is not None
        and posed_position is not None
        and abs(rest_position[2] - posed_position[2]) < 0.001,
        details=f"rest={rest_position}, posed={posed_position}",
    )

    return ctx.report()


object_model = build_object_model()
