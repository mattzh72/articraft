from __future__ import annotations

import math

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
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_DEPTH = 0.132
BODY_WIDTH = 0.180
MAIN_SHELL_Z = 0.055
MAIN_SHELL_HEIGHT = 0.925
TOP_DECK_HEIGHT = 0.055
TOP_DECK_Z = MAIN_SHELL_Z + MAIN_SHELL_HEIGHT
TOP_SURFACE_Z = TOP_DECK_Z + TOP_DECK_HEIGHT


def _rounded_rect_prism(x_size: float, y_size: float, radius: float, height: float) -> cq.Workplane:
    profile = cq.Sketch().rect(x_size, y_size).vertices().fillet(radius)
    return cq.Workplane("XY").placeSketch(profile).extrude(height)


def _tower_body_geometry() -> cq.Workplane:
    """Open continuous tower shell: rear/side walls, outlet frame, lower cover, and top deck."""
    corner = 0.056
    z_mid = MAIN_SHELL_Z + MAIN_SHELL_HEIGHT / 2.0
    rear_wall = cq.Workplane("XY").box(0.016, 0.150, MAIN_SHELL_HEIGHT).translate((-0.058, 0.0, z_mid))
    side_rail_0 = cq.Workplane("XY").box(0.108, 0.022, MAIN_SHELL_HEIGHT).translate((0.000, -0.079, z_mid))
    side_rail_1 = cq.Workplane("XY").box(0.108, 0.022, MAIN_SHELL_HEIGHT).translate((0.000, 0.079, z_mid))
    front_stile_0 = cq.Workplane("XY").box(0.020, 0.020, MAIN_SHELL_HEIGHT).translate((0.056, -0.067, z_mid))
    front_stile_1 = cq.Workplane("XY").box(0.020, 0.020, MAIN_SHELL_HEIGHT).translate((0.056, 0.067, z_mid))
    lower_outlet_lip = cq.Workplane("XY").box(0.020, 0.126, 0.030).translate((0.056, 0.0, 0.145))
    upper_outlet_lip = cq.Workplane("XY").box(0.020, 0.126, 0.030).translate((0.056, 0.0, 0.975))
    top_deck = _rounded_rect_prism(BODY_DEPTH, BODY_WIDTH, corner, TOP_DECK_HEIGHT).translate((0.0, 0.0, TOP_DECK_Z))
    lower_joint_cover = _rounded_rect_prism(0.116, 0.158, 0.048, 0.080).translate((0.0, 0.0, 0.0))
    shell = rear_wall
    for piece in (
        side_rail_0,
        side_rail_1,
        front_stile_0,
        front_stile_1,
        lower_outlet_lip,
        upper_outlet_lip,
        top_deck,
        lower_joint_cover,
    ):
        shell = shell.union(piece)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="residential_tower_fan")

    body_mat = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    grille_mat = model.material("dark_graphite_grille", rgba=(0.05, 0.055, 0.06, 1.0))
    base_mat = model.material("charcoal_base", rgba=(0.02, 0.022, 0.025, 1.0))
    knob_mat = model.material("satin_gray_controls", rgba=(0.36, 0.37, 0.36, 1.0))
    mark_mat = model.material("white_control_marks", rgba=(0.94, 0.94, 0.88, 1.0))
    impeller_mat = model.material("pale_blower_wheel", rgba=(0.72, 0.77, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.158, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=base_mat,
        name="round_foot",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=base_mat,
        name="bearing_plinth",
    )

    body = model.part("body")
    body.visual(
        Box((0.016, 0.150, MAIN_SHELL_HEIGHT)),
        origin=Origin(xyz=(-0.058, 0.0, MAIN_SHELL_Z + MAIN_SHELL_HEIGHT / 2.0)),
        material=body_mat,
        name="rear_wall",
    )
    body.visual(
        Box((0.108, 0.022, MAIN_SHELL_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.079, MAIN_SHELL_Z + MAIN_SHELL_HEIGHT / 2.0)),
        material=body_mat,
        name="side_rail_0",
    )
    body.visual(
        Box((0.108, 0.022, MAIN_SHELL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.079, MAIN_SHELL_Z + MAIN_SHELL_HEIGHT / 2.0)),
        material=body_mat,
        name="side_rail_1",
    )
    body.visual(
        Box((0.020, 0.020, MAIN_SHELL_HEIGHT)),
        origin=Origin(xyz=(0.056, -0.067, MAIN_SHELL_Z + MAIN_SHELL_HEIGHT / 2.0)),
        material=body_mat,
        name="front_stile_0",
    )
    body.visual(
        Box((0.020, 0.020, MAIN_SHELL_HEIGHT)),
        origin=Origin(xyz=(0.056, 0.067, MAIN_SHELL_Z + MAIN_SHELL_HEIGHT / 2.0)),
        material=body_mat,
        name="front_stile_1",
    )
    body.visual(
        Box((0.020, 0.126, 0.030)),
        origin=Origin(xyz=(0.056, 0.0, 0.145)),
        material=body_mat,
        name="lower_outlet_lip",
    )
    body.visual(
        Box((0.020, 0.126, 0.030)),
        origin=Origin(xyz=(0.056, 0.0, 0.975)),
        material=body_mat,
        name="upper_outlet_lip",
    )
    body.visual(
        Box((BODY_DEPTH, BODY_WIDTH, TOP_DECK_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, TOP_DECK_Z + TOP_DECK_HEIGHT / 2.0)),
        material=body_mat,
        name="top_deck",
    )
    body.visual(
        Cylinder(radius=0.079, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=body_mat,
        name="lower_cover",
    )
    outlet_grille = VentGrilleGeometry(
        (0.800, 0.126),
        frame=0.010,
        face_thickness=0.004,
        duct_depth=0.020,
        duct_wall=0.0025,
        slat_pitch=0.018,
        slat_width=0.007,
        slat_angle_deg=17.0,
        corner_radius=0.008,
        slats=VentGrilleSlats(profile="airfoil", direction="up", inset=0.002, divider_count=2, divider_width=0.004),
        frame_profile=VentGrilleFrame(style="beveled", depth=0.001),
        sleeve=VentGrilleSleeve(style="short", depth=0.016, wall=0.0025),
    )
    body.visual(
        mesh_from_geometry(outlet_grille, "broad_outlet_grille"),
        origin=Origin(
            xyz=(BODY_DEPTH * 0.49, 0.0, 0.560),
            rpy=(0.0, -math.pi / 2.0, 0.0),
        ),
        material=grille_mat,
        name="outlet_grille",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=grille_mat,
        name="lower_bearing",
    )

    impeller = model.part("impeller")
    impeller.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                outer_radius=0.043,
                inner_radius=0.021,
                width=0.720,
                blade_count=28,
                blade_thickness=0.0023,
                blade_sweep_deg=28.0,
                backplate=True,
                shroud=True,
            ),
            "vertical_crossflow_impeller",
        ),
        material=impeller_mat,
        name="impeller_wheel",
    )
    impeller.visual(
        Cylinder(radius=0.006, length=0.840),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=grille_mat,
        name="impeller_shaft",
    )
    impeller.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.360)),
        material=impeller_mat,
        name="lower_hub",
    )
    impeller.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=impeller_mat,
        name="upper_hub",
    )

    power_knob = model.part("power_knob")
    power_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.041,
                0.023,
                body_style="skirted",
                base_diameter=0.047,
                top_diameter=0.034,
                skirt=KnobSkirt(0.050, 0.005, flare=0.06, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=18, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "power_knob_cap",
        ),
        material=knob_mat,
        name="knob_body",
    )
    power_knob.visual(
        Box((0.020, 0.003, 0.0012)),
        origin=Origin(xyz=(0.006, 0.0, 0.0233)),
        material=mark_mat,
        name="pointer_mark",
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.044,
                0.024,
                body_style="faceted",
                base_diameter=0.048,
                top_diameter=0.036,
                edge_radius=0.001,
                grip=KnobGrip(style="ribbed", count=12, depth=0.0009, width=0.002),
                indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "timer_knob_cap",
        ),
        material=knob_mat,
        name="knob_body",
    )
    timer_knob.visual(
        Box((0.022, 0.0032, 0.0012)),
        origin=Origin(xyz=(0.007, 0.0, 0.0243)),
        material=mark_mat,
        name="pointer_mark",
    )

    button = model.part("oscillation_button")
    button.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=knob_mat,
        name="button_cap",
    )
    button.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=knob_mat,
        name="button_stem",
    )
    button.visual(
        Box((0.014, 0.0022, 0.0008)),
        origin=Origin(xyz=(0.002, 0.0, 0.0122)),
        material=mark_mat,
        name="button_mark",
    )

    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.8, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "body_to_impeller",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=impeller,
        origin=Origin(xyz=(0.0, 0.0, 0.560)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=70.0),
    )
    model.articulation(
        "body_to_power_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=power_knob,
        origin=Origin(xyz=(0.000, -0.040, TOP_SURFACE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0),
    )
    model.articulation(
        "body_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_knob,
        origin=Origin(xyz=(0.000, 0.034, TOP_SURFACE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0),
    )
    model.articulation(
        "body_to_oscillation_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(-0.038, 0.071, TOP_SURFACE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=0.0, upper=0.006),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    body = object_model.get_part("body")
    impeller = object_model.get_part("impeller")
    power_knob = object_model.get_part("power_knob")
    timer_knob = object_model.get_part("timer_knob")
    button = object_model.get_part("oscillation_button")

    body_joint = object_model.get_articulation("base_to_body")
    impeller_joint = object_model.get_articulation("body_to_impeller")
    power_joint = object_model.get_articulation("body_to_power_knob")
    timer_joint = object_model.get_articulation("body_to_timer_knob")
    button_joint = object_model.get_articulation("body_to_oscillation_button")

    ctx.allow_overlap(
        body,
        button,
        elem_a="top_deck",
        elem_b="button_stem",
        reason="The oscillation push-button stem is intentionally captured by a small top-deck bushing.",
    )

    ctx.check(
        "primary mechanisms are articulated",
        body_joint.articulation_type == ArticulationType.REVOLUTE
        and impeller_joint.articulation_type == ArticulationType.CONTINUOUS
        and power_joint.articulation_type == ArticulationType.CONTINUOUS
        and timer_joint.articulation_type == ArticulationType.CONTINUOUS
        and button_joint.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"types: {body_joint.articulation_type}, {impeller_joint.articulation_type}, "
            f"{power_joint.articulation_type}, {timer_joint.articulation_type}, {button_joint.articulation_type}"
        ),
    )

    ctx.expect_contact(base, body, elem_a="bearing_plinth", elem_b="lower_cover", contact_tol=0.002, name="body sits on base bearing")
    ctx.expect_contact(power_knob, body, elem_a="knob_body", elem_b="top_deck", contact_tol=0.002, name="power knob is seated on top deck")
    ctx.expect_contact(timer_knob, body, elem_a="knob_body", elem_b="top_deck", contact_tol=0.002, name="timer knob is seated on top deck")
    ctx.expect_gap(
        button,
        body,
        axis="z",
        positive_elem="button_stem",
        negative_elem="top_deck",
        max_gap=0.001,
        max_penetration=0.012,
        name="push button stem is captured by deck",
    )
    ctx.expect_gap(
        body,
        impeller,
        axis="x",
        positive_elem="outlet_grille",
        negative_elem="impeller_wheel",
        min_gap=0.006,
        name="impeller sits behind outlet grille",
    )
    ctx.expect_within(
        impeller,
        body,
        axes="xy",
        inner_elem="impeller_wheel",
        margin=0.0,
        name="impeller is contained within tower footprint",
    )

    rest_button = ctx.part_world_position(button)
    with ctx.pose({button_joint: 0.006}):
        pressed_button = ctx.part_world_position(button)
    ctx.check(
        "oscillation button depresses downward",
        rest_button is not None and pressed_button is not None and pressed_button[2] < rest_button[2] - 0.004,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    return ctx.report()


object_model = build_object_model()
