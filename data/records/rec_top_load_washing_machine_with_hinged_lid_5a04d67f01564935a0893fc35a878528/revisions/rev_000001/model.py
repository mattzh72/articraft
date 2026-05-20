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
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CABINET_W = 0.72
CABINET_D = 0.78
PANEL_THICK = 0.035
DECK_THICK = 0.050
DECK_Z = 0.845
DECK_TOP_Z = DECK_Z + DECK_THICK / 2.0
OPENING_Y = -0.070
OPENING_R = 0.290
HINGE_Y = 0.225
HINGE_Z = 0.908
CONTROL_FACE_Y = 0.252


def _deck_with_deep_well() -> object:
    """Top deck slab with a true through-opening and a deep molded well below it."""
    well_h = 0.300
    well_outer = 0.315
    well_inner = 0.282

    deck = cq.Workplane("XY").box(CABINET_W, CABINET_D, DECK_THICK)
    through = (
        cq.Workplane("XY")
        .workplane(offset=-DECK_THICK / 2.0 - 0.020)
        .center(0.0, OPENING_Y)
        .circle(OPENING_R)
        .extrude(DECK_THICK + 0.040)
    )
    deck = deck.cut(through)

    outer_well = (
        cq.Workplane("XY")
        .workplane(offset=-DECK_THICK / 2.0 - well_h)
        .center(0.0, OPENING_Y)
        .circle(well_outer)
        .extrude(well_h)
    )
    inner_well_cut = (
        cq.Workplane("XY")
        .workplane(offset=-DECK_THICK / 2.0 - well_h - 0.010)
        .center(0.0, OPENING_Y)
        .circle(well_inner)
        .extrude(well_h + 0.020)
    )
    well = outer_well.cut(inner_well_cut)
    return deck.union(well).edges("|Z").fillet(0.006)


def _rotating_tub() -> object:
    """Open stainless basket with bottom, rolled lip, and central agitator post."""
    outer_r = 0.255
    inner_r = 0.220
    height = 0.580
    bottom = 0.055

    shell = cq.Workplane("XY").circle(outer_r).extrude(height)
    cavity = cq.Workplane("XY").workplane(offset=bottom).circle(inner_r).extrude(height)
    shell = shell.cut(cavity)

    rim_outer = cq.Workplane("XY").workplane(offset=height - 0.010).circle(0.270).extrude(0.035)
    rim_cut = cq.Workplane("XY").workplane(offset=height - 0.020).circle(0.225).extrude(0.060)
    rim = rim_outer.cut(rim_cut)

    agitator = cq.Workplane("XY").circle(0.055).extrude(0.350)
    cap = cq.Workplane("XY").workplane(offset=0.330).circle(0.075).extrude(0.040)
    fin_a = cq.Workplane("XY").box(0.170, 0.026, 0.180).translate((0.075, 0.0, 0.125))
    fin_b = cq.Workplane("XY").box(0.026, 0.170, 0.180).translate((0.0, 0.075, 0.125))

    tub = shell.union(rim).union(agitator).union(cap).union(fin_a).union(fin_b)
    return tub.edges("|Z").fillet(0.004)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="residential_top_load_washer")

    enamel = model.material("warm_white_enamel", rgba=(0.93, 0.91, 0.86, 1.0))
    dark = model.material("black_control_panel", rgba=(0.035, 0.038, 0.042, 1.0))
    shadow = model.material("deep_shadow", rgba=(0.010, 0.012, 0.014, 1.0))
    steel = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.72, 1.0))
    gray = model.material("soft_gray_plastic", rgba=(0.32, 0.34, 0.36, 1.0))
    button_mat = model.material("satin_buttons", rgba=(0.78, 0.80, 0.79, 1.0))
    glass = model.material("smoked_glass", rgba=(0.16, 0.22, 0.28, 0.45))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((PANEL_THICK, CABINET_D, 0.820)),
        origin=Origin(xyz=(-CABINET_W / 2.0 + PANEL_THICK / 2.0, 0.0, 0.410)),
        material=enamel,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((PANEL_THICK, CABINET_D, 0.820)),
        origin=Origin(xyz=(CABINET_W / 2.0 - PANEL_THICK / 2.0, 0.0, 0.410)),
        material=enamel,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((CABINET_W, PANEL_THICK, 0.820)),
        origin=Origin(xyz=(0.0, -CABINET_D / 2.0 + PANEL_THICK / 2.0, 0.410)),
        material=enamel,
        name="front_panel",
    )
    cabinet.visual(
        Box((CABINET_W, PANEL_THICK, 0.820)),
        origin=Origin(xyz=(0.0, CABINET_D / 2.0 - PANEL_THICK / 2.0, 0.410)),
        material=enamel,
        name="rear_panel",
    )
    cabinet.visual(
        Box((CABINET_W, CABINET_D, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=gray,
        name="toe_base",
    )
    cabinet.visual(
        mesh_from_cadquery(_deck_with_deep_well(), "washer_top_deck", tolerance=0.0015),
        origin=Origin(xyz=(0.0, 0.0, DECK_Z)),
        material=enamel,
        name="top_deck",
    )
    cabinet.visual(
        Box((0.700, 0.130, 0.220)),
        origin=Origin(xyz=(0.0, 0.325, 0.980)),
        material=enamel,
        name="rear_console",
    )
    cabinet.visual(
        Box((0.660, 0.008, 0.155)),
        origin=Origin(xyz=(0.0, 0.256, 1.020)),
        material=dark,
        name="control_face",
    )
    cabinet.visual(
        Box((0.095, 0.004, 0.040)),
        origin=Origin(xyz=(0.190, CONTROL_FACE_Y - 0.0015, 1.020)),
        material=gray,
        name="button_bezel",
    )
    cabinet.visual(
        Box((0.078, 0.040, 0.006)),
        origin=Origin(xyz=(0.0, -0.375, DECK_TOP_Z + 0.003)),
        material=shadow,
        name="lock_socket",
    )
    cabinet.visual(
        Cylinder(radius=0.009, length=0.640),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gray,
        name="hinge_pin",
    )
    cabinet.visual(
        Box((0.035, 0.026, 0.058)),
        origin=Origin(xyz=(-0.290, HINGE_Y + 0.016, HINGE_Z - 0.020)),
        material=enamel,
        name="hinge_bracket_0",
    )
    cabinet.visual(
        Box((0.035, 0.026, 0.058)),
        origin=Origin(xyz=(0.290, HINGE_Y + 0.016, HINGE_Z - 0.020)),
        material=enamel,
        name="hinge_bracket_1",
    )
    cabinet.visual(
        Cylinder(radius=0.032, length=0.300),
        origin=Origin(xyz=(0.0, OPENING_Y, 0.150)),
        material=gray,
        name="drive_spindle",
    )
    for i, dx in enumerate((-0.050, -0.025, 0.0, 0.025, 0.050)):
        cabinet.visual(
            Box((0.006, 0.003, 0.026 if i == 2 else 0.018)),
            origin=Origin(xyz=(-0.205 + dx, CONTROL_FACE_Y - 0.0015, 1.085)),
            material=enamel,
            name=f"dial_tick_{i}",
        )

    tub = model.part("tub")
    tub.visual(
        mesh_from_cadquery(_rotating_tub(), "washer_rotating_tub", tolerance=0.0015),
        origin=Origin(),
        material=steel,
        name="basket_shell",
    )
    tub.visual(
        Cylinder(radius=0.205, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=shadow,
        name="cavity_shadow",
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.620, 0.050, 0.035)),
        origin=Origin(xyz=(0.0, -0.055, -0.005)),
        material=enamel,
        name="rear_rail",
    )
    lid.visual(
        Box((0.620, 0.060, 0.035)),
        origin=Origin(xyz=(0.0, -0.580, -0.005)),
        material=enamel,
        name="front_rail",
    )
    lid.visual(
        Box((0.060, 0.560, 0.035)),
        origin=Origin(xyz=(-0.280, -0.315, -0.005)),
        material=enamel,
        name="side_rail_0",
    )
    lid.visual(
        Box((0.060, 0.560, 0.035)),
        origin=Origin(xyz=(0.280, -0.315, -0.005)),
        material=enamel,
        name="side_rail_1",
    )
    lid.visual(
        Box((0.520, 0.500, 0.010)),
        origin=Origin(xyz=(0.0, -0.300, -0.010)),
        material=glass,
        name="glass_window",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.180),
        origin=Origin(xyz=(-0.175, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=enamel,
        name="hinge_barrel_0",
    )
    lid.visual(
        Box((0.115, 0.032, 0.010)),
        origin=Origin(xyz=(-0.175, -0.024, 0.014)),
        material=enamel,
        name="hinge_leaf_0",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.180),
        origin=Origin(xyz=(0.175, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=enamel,
        name="hinge_barrel_1",
    )
    lid.visual(
        Box((0.115, 0.032, 0.010)),
        origin=Origin(xyz=(0.175, -0.024, 0.014)),
        material=enamel,
        name="hinge_leaf_1",
    )

    dial = model.part("cycle_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.090,
                0.036,
                body_style="skirted",
                top_diameter=0.068,
                skirt=KnobSkirt(0.112, 0.008, flare=0.05, chamfer=0.0015),
                grip=KnobGrip(style="fluted", count=24, depth=0.0018),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
                center=False,
            ),
            "washer_cycle_dial",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_mat,
        name="dial_cap",
    )

    buttons = []
    for i, x in enumerate((0.100, 0.160, 0.220, 0.280)):
        button = model.part(f"button_{i}")
        button.visual(
            Box((0.044, 0.018, 0.026)),
            origin=Origin(xyz=(0.0, -0.009, 0.0)),
            material=button_mat,
            name="button_cap",
        )
        buttons.append(button)

    plunger = model.part("lock_plunger")
    plunger.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark,
        name="lock_pin",
    )

    model.articulation(
        "cabinet_to_tub",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0.0, OPENING_Y, 0.245)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=8.0),
    )
    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.75),
    )
    model.articulation(
        "cabinet_to_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(-0.205, CONTROL_FACE_Y, 1.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )
    for i, button in enumerate(buttons):
        model.articulation(
            f"cabinet_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(0.100 + 0.060 * i, CONTROL_FACE_Y, 1.020)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.10, lower=0.0, upper=0.012),
        )
    model.articulation(
        "lid_to_plunger",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=plunger,
        origin=Origin(xyz=(0.0, -0.600, -0.036)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.08, lower=0.0, upper=0.028),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    tub = object_model.get_part("tub")
    dial = object_model.get_part("cycle_dial")
    plunger = object_model.get_part("lock_plunger")
    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    plunger_slide = object_model.get_articulation("lid_to_plunger")

    ctx.allow_overlap(
        cabinet,
        lid,
        elem_a="hinge_pin",
        elem_b="hinge_barrel_0",
        reason="The cabinet hinge pin is intentionally captured inside the first lid hinge barrel.",
    )
    ctx.allow_overlap(
        cabinet,
        lid,
        elem_a="hinge_pin",
        elem_b="hinge_barrel_1",
        reason="The cabinet hinge pin is intentionally captured inside the second lid hinge barrel.",
    )
    ctx.allow_overlap(
        cabinet,
        tub,
        elem_a="drive_spindle",
        elem_b="basket_shell",
        reason="The hidden motor spindle intentionally nests into the rotating basket hub.",
    )
    ctx.expect_overlap(
        lid,
        cabinet,
        axes="x",
        elem_a="hinge_barrel_0",
        elem_b="hinge_pin",
        min_overlap=0.080,
        name="first lid hinge barrel is retained on the hinge pin",
    )
    ctx.expect_overlap(
        lid,
        cabinet,
        axes="x",
        elem_a="hinge_barrel_1",
        elem_b="hinge_pin",
        min_overlap=0.080,
        name="second lid hinge barrel is retained on the hinge pin",
    )
    ctx.expect_overlap(
        tub,
        cabinet,
        axes="z",
        elem_a="basket_shell",
        elem_b="drive_spindle",
        min_overlap=0.040,
        name="rotating tub remains seated on the drive spindle",
    )
    ctx.expect_gap(
        lid,
        cabinet,
        axis="z",
        positive_elem="front_rail",
        negative_elem="top_deck",
        min_gap=0.004,
        max_gap=0.030,
        name="closed lid sits just above the top deck",
    )
    ctx.expect_within(
        tub,
        cabinet,
        axes="xy",
        inner_elem="basket_shell",
        outer_elem="top_deck",
        margin=0.010,
        name="rotating basket is centered inside the deck opening footprint",
    )
    ctx.expect_overlap(
        tub,
        cabinet,
        axes="z",
        elem_a="basket_shell",
        elem_b="top_deck",
        min_overlap=0.010,
        name="deep basket reaches up under the top deck opening",
    )
    ctx.expect_contact(
        dial,
        cabinet,
        elem_a="dial_cap",
        elem_b="control_face",
        contact_tol=0.004,
        name="cycle dial is mounted on the console face",
    )

    rest_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.35}):
        raised_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid rotates upward on the rear hinge",
        rest_lid_aabb is not None
        and raised_lid_aabb is not None
        and raised_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.30,
        details=f"rest={rest_lid_aabb}, raised={raised_lid_aabb}",
    )

    for i in range(4):
        button = object_model.get_part(f"button_{i}")
        joint = object_model.get_articulation(f"cabinet_to_button_{i}")
        ctx.expect_contact(
            button,
            cabinet,
            elem_a="button_cap",
            elem_b="control_face",
            contact_tol=0.004,
            name=f"button {i} starts proud of the console",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.012}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button {i} depresses independently inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.010,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    ctx.allow_overlap(
        plunger,
        cabinet,
        elem_a="lock_pin",
        elem_b="top_deck",
        reason="At full stroke the lid-lock pin intentionally enters the top-deck latch socket.",
    )
    rest_pin_pos = ctx.part_world_position(plunger)
    with ctx.pose({plunger_slide: 0.028}):
        inserted_pin_pos = ctx.part_world_position(plunger)
        ctx.expect_overlap(
            plunger,
            cabinet,
            axes="xy",
            elem_a="lock_pin",
            elem_b="lock_socket",
            min_overlap=0.010,
            name="lid-lock plunger aligns with the front socket",
        )
        ctx.expect_gap(
            plunger,
            cabinet,
            axis="z",
            positive_elem="lock_pin",
            negative_elem="top_deck",
            max_penetration=0.035,
            name="lid-lock plunger travels into the top deck",
        )
    ctx.check(
        "lid-lock plunger moves downward",
        rest_pin_pos is not None
        and inserted_pin_pos is not None
        and inserted_pin_pos[2] < rest_pin_pos[2] - 0.020,
        details=f"rest={rest_pin_pos}, inserted={inserted_pin_pos}",
    )

    return ctx.report()


object_model = build_object_model()
