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


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _cabinet_shell():
    width = 0.76
    depth = 0.82
    wall = 0.035
    body_h = 0.91
    deck_thick = 0.055
    deck_z = body_h + deck_thick / 2.0

    shell = _cq_box((width, depth, 0.045), (0.0, 0.0, 0.0225))
    shell = shell.union(_cq_box((wall, depth, body_h), (-width / 2.0 + wall / 2.0, 0.0, body_h / 2.0)))
    shell = shell.union(_cq_box((wall, depth, body_h), (width / 2.0 - wall / 2.0, 0.0, body_h / 2.0)))
    shell = shell.union(_cq_box((width, wall, body_h), (0.0, -depth / 2.0 + wall / 2.0, body_h / 2.0)))
    shell = shell.union(_cq_box((width, wall, body_h), (0.0, depth / 2.0 - wall / 2.0, body_h / 2.0)))

    deck = _cq_box((width, depth, deck_thick), (0.0, 0.0, deck_z))
    deck_cutter = cq.Workplane("XY").circle(0.305).extrude(0.20).translate((0.0, 0.0, body_h - 0.04))
    deck = deck.cut(deck_cutter)
    shell = shell.union(deck)

    console = _cq_box((0.78, 0.12, 0.245), (0.0, 0.395, 1.055))
    shell = shell.union(console)
    return shell


def _wash_tub():
    outer_r = 0.285
    inner_r = 0.252
    bottom_z = 0.215
    height = 0.56

    outer = cq.Workplane("XY").circle(outer_r).extrude(height).translate((0.0, 0.0, bottom_z))
    inner_cut = cq.Workplane("XY").circle(inner_r).extrude(height + 0.08).translate((0.0, 0.0, bottom_z + 0.045))
    tub = outer.cut(inner_cut)

    rim_outer = cq.Workplane("XY").circle(outer_r + 0.018).extrude(0.030).translate((0.0, 0.0, bottom_z + height - 0.005))
    rim_inner = cq.Workplane("XY").circle(inner_r - 0.002).extrude(0.050).translate((0.0, 0.0, bottom_z + height - 0.015))
    tub = tub.union(rim_outer.cut(rim_inner))

    agitator = cq.Workplane("XY").circle(0.055).extrude(0.43).translate((0.0, 0.0, bottom_z + 0.045))
    skirt = cq.Workplane("XY").circle(0.135).extrude(0.055).translate((0.0, 0.0, bottom_z + 0.025))
    tub = tub.union(agitator).union(skirt)

    for angle in (0.0, 120.0, 240.0):
        fin = (
            cq.Workplane("XY")
            .box(0.135, 0.018, 0.285)
            .translate((0.090, 0.0, bottom_z + 0.235))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        tub = tub.union(fin)

    cup_outer = cq.Workplane("XY").circle(0.079).extrude(0.055).translate((0.0, 0.0, bottom_z + 0.445))
    cup_inner = cq.Workplane("XY").circle(0.052).extrude(0.075).translate((0.0, 0.0, bottom_z + 0.460))
    tub = tub.union(cup_outer.cut(cup_inner))
    return tub


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="large_top_load_washer")

    porcelain = model.material("white_porcelain", rgba=(0.92, 0.94, 0.93, 1.0))
    shadow = model.material("deep_shadow", rgba=(0.03, 0.035, 0.04, 1.0))
    smoked = model.material("smoked_window", rgba=(0.18, 0.25, 0.30, 0.38))
    stainless = model.material("brushed_stainless", rgba=(0.66, 0.68, 0.69, 1.0))
    dark_panel = model.material("black_console", rgba=(0.015, 0.018, 0.020, 1.0))
    rubber = model.material("soft_black", rgba=(0.02, 0.02, 0.018, 1.0))
    blue = model.material("blue_controls", rgba=(0.10, 0.32, 0.65, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shell(), "washer_cabinet"),
        material=porcelain,
        name="cabinet_shell",
    )
    cabinet.visual(
        Box((0.62, 0.008, 0.130)),
        origin=Origin(xyz=(0.0, 0.336, 1.055)),
        material=dark_panel,
        name="console_panel",
    )
    cabinet.visual(
        Cylinder(radius=0.014, length=0.62),
        origin=Origin(xyz=(0.0, 0.330, 0.965), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=porcelain,
        name="rear_hinge_barrel",
    )
    cabinet.visual(
        Box((0.38, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, -0.407, 0.045)),
        material=rubber,
        name="toe_shadow",
    )
    cabinet.visual(
        Cylinder(radius=0.060, length=0.217),
        origin=Origin(xyz=(0.0, 0.0, 0.1085)),
        material=stainless,
        name="bearing_post",
    )
    cabinet.visual(
        Cylinder(radius=0.009, length=0.170),
        origin=Origin(xyz=(-0.270, -0.315, 0.973), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=porcelain,
        name="cap_mount_barrel",
    )

    tub = model.part("wash_tub")
    tub.visual(
        mesh_from_cadquery(_wash_tub(), "hollow_wash_tub", tolerance=0.0015),
        material=stainless,
        name="hollow_basket",
    )
    tub.visual(
        Cylinder(radius=0.245, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.218)),
        material=shadow,
        name="cavity_shadow",
    )
    model.articulation(
        "tub_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=14.0),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.62, 0.045, 0.040)),
        origin=Origin(xyz=(0.0, -0.580, 0.018)),
        material=porcelain,
        name="front_frame",
    )
    lid.visual(
        Box((0.62, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, -0.025, 0.018)),
        material=porcelain,
        name="rear_frame",
    )
    lid.visual(
        Box((0.050, 0.555, 0.040)),
        origin=Origin(xyz=(-0.285, -0.300, 0.018)),
        material=porcelain,
        name="side_frame_0",
    )
    lid.visual(
        Box((0.050, 0.555, 0.040)),
        origin=Origin(xyz=(0.285, -0.300, 0.018)),
        material=porcelain,
        name="side_frame_1",
    )
    lid.visual(
        Box((0.540, 0.510, 0.010)),
        origin=Origin(xyz=(0.0, -0.305, 0.018)),
        material=smoked,
        name="window_panel",
    )
    lid.visual(
        Box((0.310, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, -0.615, 0.047)),
        material=porcelain,
        name="front_handle",
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, 0.3162, 0.968)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=2.0, lower=0.0, upper=1.45),
    )

    dial = model.part("selector_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.085,
                0.038,
                body_style="skirted",
                top_diameter=0.060,
                skirt=KnobSkirt(0.104, 0.010, flare=0.05, chamfer=0.0015),
                grip=KnobGrip(style="fluted", count=24, depth=0.0014),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
                center=False,
            ),
            "selector_dial",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=porcelain,
        name="dial_cap",
    )
    dial.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="dial_shaft",
    )
    model.articulation(
        "dial_turn",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(-0.220, 0.332, 1.070)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=7.0),
    )

    button_xs = (-0.080, 0.000, 0.080, 0.160, 0.240)
    for index, x in enumerate(button_xs):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.052, 0.026, 0.028)),
            origin=Origin(xyz=(0.0, -0.013, 0.0)),
            material=blue if index == 0 else porcelain,
            name="button_cap",
        )
        button.visual(
            Box((0.034, 0.010, 0.018)),
            origin=Origin(xyz=(0.0, -0.030, 0.0)),
            material=shadow,
            name="button_face",
        )
        model.articulation(
            f"button_push_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, 0.332, 1.035)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.009),
        )

    detergent_cap = model.part("detergent_cap")
    detergent_cap.visual(
        Box((0.150, 0.080, 0.014)),
        origin=Origin(xyz=(0.0, -0.049, 0.0)),
        material=porcelain,
        name="cap_panel",
    )
    detergent_cap.visual(
        Cylinder(radius=0.010, length=0.150),
        origin=Origin(xyz=(0.0, 0.001, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=porcelain,
        name="cap_hinge_barrel",
    )
    model.articulation(
        "detergent_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=detergent_cap,
        origin=Origin(xyz=(-0.270, -0.315, 0.977)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.4, velocity=1.8, lower=0.0, upper=1.25),
    )

    softener_lid = model.part("softener_lid")
    softener_lid.visual(
        Cylinder(radius=0.061, length=0.012),
        origin=Origin(xyz=(0.0, -0.055, 0.0)),
        material=porcelain,
        name="cup_lid_disk",
    )
    softener_lid.visual(
        Cylinder(radius=0.006, length=0.090),
        origin=Origin(xyz=(0.0, 0.000, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=porcelain,
        name="cup_lid_hinge",
    )
    model.articulation(
        "softener_hinge",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=softener_lid,
        origin=Origin(xyz=(0.0, 0.055, 0.721)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=1.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    tub = object_model.get_part("wash_tub")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("selector_dial")
    detergent_cap = object_model.get_part("detergent_cap")
    softener_lid = object_model.get_part("softener_lid")

    ctx.allow_overlap(
        cabinet,
        detergent_cap,
        elem_a="cap_mount_barrel",
        elem_b="cap_hinge_barrel",
        reason="The fold-flat detergent cap has a captured hinge barrel seated through the deck-side barrel.",
    )
    ctx.expect_overlap(
        cabinet,
        detergent_cap,
        axes="x",
        min_overlap=0.12,
        elem_a="cap_mount_barrel",
        elem_b="cap_hinge_barrel",
        name="detergent cap hinge barrels are captured",
    )
    ctx.allow_overlap(
        cabinet,
        dial,
        elem_a="cabinet_shell",
        elem_b="dial_shaft",
        reason="The selector dial shaft intentionally enters the rear console bearing hole.",
    )
    ctx.expect_overlap(
        cabinet,
        dial,
        axes="xyz",
        min_overlap=0.020,
        elem_a="cabinet_shell",
        elem_b="dial_shaft",
        name="selector dial shaft is retained in console",
    )
    ctx.allow_overlap(
        cabinet,
        dial,
        elem_a="console_panel",
        elem_b="dial_shaft",
        reason="The dial shaft passes through the dark console faceplate at its bearing opening.",
    )
    ctx.expect_overlap(
        cabinet,
        dial,
        axes="yz",
        min_overlap=0.008,
        elem_a="console_panel",
        elem_b="dial_shaft",
        name="selector dial shaft passes through faceplate",
    )
    ctx.allow_overlap(
        softener_lid,
        tub,
        elem_a="cup_lid_disk",
        elem_b="hollow_basket",
        reason="The softener cup lid is seated on the simplified agitator cup rim as a flush captured cap.",
    )
    ctx.expect_overlap(
        softener_lid,
        tub,
        axes="xy",
        min_overlap=0.060,
        elem_a="cup_lid_disk",
        elem_b="hollow_basket",
        name="softener lid stays centered on cup rim",
    )

    ctx.check(
        "primary rotary joints are continuous",
        object_model.get_articulation("tub_spin").articulation_type == ArticulationType.CONTINUOUS
        and object_model.get_articulation("dial_turn").articulation_type == ArticulationType.CONTINUOUS,
        details="The wash tub and rear selector dial should be unlimited rotary controls.",
    )

    button_joints = [object_model.get_articulation(f"button_push_{i}") for i in range(5)]
    ctx.check(
        "five separate prismatic mode buttons",
        len(button_joints) == 5
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in button_joints)
        and all(j.motion_limits is not None and j.motion_limits.upper == 0.009 for j in button_joints),
        details="Each mode button must be its own short-travel push control.",
    )

    ctx.expect_within(
        tub,
        cabinet,
        axes="xy",
        margin=0.0,
        inner_elem="hollow_basket",
        outer_elem="cabinet_shell",
        name="wash basket sits inside cabinet footprint",
    )
    tub_aabb = ctx.part_element_world_aabb(tub, elem="hollow_basket")
    ctx.check(
        "deep hollow laundry cavity scale",
        tub_aabb is not None
        and (tub_aabb[1][2] - tub_aabb[0][2]) > 0.52
        and (tub_aabb[1][0] - tub_aabb[0][0]) > 0.55,
        details=f"tub_aabb={tub_aabb}",
    )

    lid_front_closed = ctx.part_element_world_aabb(lid, elem="front_frame")
    with ctx.pose({"lid_hinge": 1.20}):
        lid_front_open = ctx.part_element_world_aabb(lid, elem="front_frame")
    ctx.check(
        "lid front edge lifts on rear hinge",
        lid_front_closed is not None
        and lid_front_open is not None
        and lid_front_open[0][2] > lid_front_closed[0][2] + 0.25,
        details=f"closed={lid_front_closed}, open={lid_front_open}",
    )

    button0 = object_model.get_part("mode_button_0")
    button_rest = ctx.part_world_position(button0)
    with ctx.pose({"button_push_0": 0.009}):
        button_pushed = ctx.part_world_position(button0)
    ctx.check(
        "mode button pushes into console",
        button_rest is not None and button_pushed is not None and button_pushed[1] > button_rest[1] + 0.006,
        details=f"rest={button_rest}, pushed={button_pushed}",
    )

    cap_rest = ctx.part_element_world_aabb(detergent_cap, elem="cap_panel")
    with ctx.pose({"detergent_hinge": 0.95}):
        cap_open = ctx.part_element_world_aabb(detergent_cap, elem="cap_panel")
    ctx.check(
        "detergent cap folds upward from deck hinge",
        cap_rest is not None and cap_open is not None and cap_open[1][2] > cap_rest[1][2] + 0.055,
        details=f"rest={cap_rest}, open={cap_open}",
    )

    softener_rest = ctx.part_element_world_aabb(softener_lid, elem="cup_lid_disk")
    with ctx.pose({"softener_hinge": 1.05}):
        softener_open = ctx.part_element_world_aabb(softener_lid, elem="cup_lid_disk")
    ctx.check(
        "softener cup lid flips at agitator top",
        softener_rest is not None
        and softener_open is not None
        and softener_open[1][2] > softener_rest[1][2] + 0.055,
        details=f"rest={softener_rest}, open={softener_open}",
    )

    dial_pos = ctx.part_world_position(dial)
    ctx.check(
        "selector dial mounted on rear console",
        dial_pos is not None and dial_pos[1] > 0.30 and 1.0 < dial_pos[2] < 1.12,
        details=f"dial_pos={dial_pos}",
    )

    return ctx.report()


object_model = build_object_model()
