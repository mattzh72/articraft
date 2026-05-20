from __future__ import annotations

import math

import cadquery as cq

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
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BOX_X = 0.340
BOX_Y = 0.260
BOX_Z = 0.155
WALL = 0.018

HINGE_X = -BOX_X / 2.0 - 0.006
HINGE_Z = BOX_Z + 0.014
LID_X = BOX_X + 0.006
LID_Y = BOX_Y + 0.004
LID_FRAME_X = LID_X - 0.016
LID_FRAME_ORIGIN_X = 0.016 + LID_FRAME_X / 2.0

WINDER_AXIS = (0.030, 0.0, 0.094)


def _hollow_rounded_box(length: float, width: float, height: float, wall: float) -> cq.Workplane:
    """Open-topped presentation-box shell, modeled as a hollow body."""
    outer = cq.Workplane("XY").box(length, width, height).translate((0.0, 0.0, height / 2.0))
    outer = outer.edges("|Z").fillet(0.011)
    cutter = (
        cq.Workplane("XY")
        .box(length - 2.0 * wall, width - 2.0 * wall, height + 0.030)
        .translate((0.0, 0.0, wall + (height + 0.030) / 2.0))
    )
    return outer.cut(cutter)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(*size)
    return shape.edges().fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_watch_winder_box")

    matte_ebony = model.material("matte_ebony", rgba=(0.025, 0.023, 0.021, 1.0))
    satin_black = model.material("satin_black", rgba=(0.065, 0.062, 0.058, 1.0))
    charcoal_velvet = model.material("charcoal_velvet", rgba=(0.012, 0.017, 0.021, 1.0))
    warm_leather = model.material("warm_leather", rgba=(0.28, 0.16, 0.075, 1.0))
    champagne = model.material("satin_champagne", rgba=(0.78, 0.66, 0.43, 1.0))
    glass = model.material("smoked_glass", rgba=(0.38, 0.52, 0.58, 0.34))
    gasket = model.material("soft_black_gasket", rgba=(0.006, 0.006, 0.005, 1.0))
    steel = model.material("polished_steel", rgba=(0.76, 0.75, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_hollow_rounded_box(BOX_X, BOX_Y, BOX_Z, WALL), "hollow_box_shell"),
        material=matte_ebony,
        name="hollow_shell",
    )

    inner_x = BOX_X - 2.0 * WALL - 0.010
    inner_y = BOX_Y - 2.0 * WALL - 0.010
    body.visual(
        Box((inner_x, inner_y, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, WALL + 0.002)),
        material=charcoal_velvet,
        name="velvet_floor",
    )
    body.visual(
        Box((0.004, inner_y, 0.098)),
        origin=Origin(xyz=(-BOX_X / 2.0 + WALL + 0.002, 0.0, WALL + 0.049)),
        material=charcoal_velvet,
        name="rear_liner",
    )
    body.visual(
        Box((0.004, inner_y, 0.080)),
        origin=Origin(xyz=(BOX_X / 2.0 - WALL - 0.002, 0.0, WALL + 0.040)),
        material=charcoal_velvet,
        name="front_liner",
    )
    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (BOX_X - 2.0 * WALL - 0.012, BOX_Y - 2.0 * WALL - 0.012),
                (BOX_X - 0.014, BOX_Y - 0.014),
                0.003,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.012,
                outer_corner_radius=0.018,
                face=BezelFace(style="flat", front_lip=0.0, fillet=0.001),
                center=False,
            ),
            "top_gasket",
        ),
        origin=Origin(xyz=(0.0, 0.0, BOX_Z)),
        material=gasket,
        name="top_gasket",
    )

    # Restrained front hardware and control interface.
    body.visual(
        Box((0.006, 0.130, 0.014)),
        origin=Origin(xyz=(BOX_X / 2.0 + 0.003, 0.0, 0.066)),
        material=champagne,
        name="front_inlay",
    )
    body.visual(
        Box((0.004, 0.040, 0.040)),
        origin=Origin(xyz=(BOX_X / 2.0 + 0.002, -0.075, 0.066)),
        material=satin_black,
        name="control_plate",
    )
    body.visual(
        Box((0.004, 0.052, 0.012)),
        origin=Origin(xyz=(BOX_X / 2.0 + 0.002, 0.072, 0.070)),
        material=champagne,
        name="latch_keeper",
    )

    # Fixed winder surround: a framed circular opening with real support struts.
    body.visual(
        Box((0.018, 0.012, 0.095)),
        origin=Origin(xyz=(WINDER_AXIS[0] - 0.010, -0.072, WALL + 0.047)),
        material=satin_black,
        name="winder_strut_0",
    )
    body.visual(
        Box((0.018, 0.012, 0.095)),
        origin=Origin(xyz=(WINDER_AXIS[0] - 0.010, 0.072, WALL + 0.047)),
        material=satin_black,
        name="winder_strut_1",
    )
    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.112, 0.112),
                (0.145, 0.145),
                0.012,
                opening_shape="circle",
                outer_shape="circle",
                face=BezelFace(style="radiused_step", front_lip=0.002, fillet=0.001),
            ),
            "winder_bezel",
        ),
        origin=Origin(xyz=(WINDER_AXIS[0] - 0.004, WINDER_AXIS[1], WINDER_AXIS[2]), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=champagne,
        name="winder_bezel",
    )
    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.014, 0.014),
                (0.032, 0.032),
                0.008,
                opening_shape="circle",
                outer_shape="circle",
                face=BezelFace(style="radiused_step", front_lip=0.001, fillet=0.0005),
            ),
            "cradle_bearing",
        ),
        origin=Origin(xyz=(WINDER_AXIS[0] - 0.038, WINDER_AXIS[1], WINDER_AXIS[2]), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="cradle_bearing",
    )
    body.visual(
        Box((0.012, 0.010, 0.058)),
        origin=Origin(xyz=(WINDER_AXIS[0] - 0.038, WINDER_AXIS[1], 0.049)),
        material=satin_black,
        name="bearing_web",
    )

    # Exposed hinge hardware: separated knuckles plus one captured pin.
    for index, y in enumerate((-0.084, 0.084)):
        body.visual(
            Box((0.004, 0.064, 0.042)),
            origin=Origin(xyz=(-BOX_X / 2.0 - 0.002, y, BOX_Z - 0.002)),
            material=champagne,
            name=f"body_hinge_leaf_{index}",
        )
        body.visual(
            Cylinder(radius=0.0062, length=0.064),
            origin=Origin(xyz=(HINGE_X, y, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=champagne,
            name=f"body_hinge_barrel_{index}",
        )
    body.visual(
        Cylinder(radius=0.0024, length=0.252),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(
            BezelGeometry(
                (LID_FRAME_X - 0.060, LID_Y - 0.058),
                (LID_FRAME_X, LID_Y),
                0.016,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.018,
                outer_corner_radius=0.020,
                face=BezelFace(style="radiused_step", front_lip=0.0015, fillet=0.0012),
            ),
            "lid_frame",
        ),
        origin=Origin(xyz=(LID_FRAME_ORIGIN_X, 0.0, -0.003)),
        material=satin_black,
        name="lid_frame",
    )
    lid.visual(
        Box((LID_FRAME_X - 0.052, LID_Y - 0.050, 0.004)),
        origin=Origin(xyz=(LID_FRAME_ORIGIN_X, 0.0, -0.004)),
        material=glass,
        name="glass_pane",
    )
    lid.visual(
        mesh_from_geometry(
            BezelGeometry(
                (LID_FRAME_X - 0.064, LID_Y - 0.062),
                (LID_FRAME_X - 0.036, LID_Y - 0.034),
                0.003,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.016,
                outer_corner_radius=0.018,
            ),
            "lid_gasket",
        ),
        origin=Origin(xyz=(LID_FRAME_ORIGIN_X, 0.0, -0.012)),
        material=gasket,
        name="lid_gasket",
    )
    lid.visual(
        Box((0.020, 0.086, 0.004)),
        origin=Origin(xyz=(0.012, 0.0, -0.004)),
        material=champagne,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.0064, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=champagne,
        name="lid_barrel",
    )
    lid.visual(
        Box((0.004, 0.050, 0.010)),
        origin=Origin(xyz=(LID_X - 0.014, 0.072, -0.013)),
        material=champagne,
        name="latch_striker",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.0074, length=0.030),
        origin=Origin(xyz=(-0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="drive_shaft",
    )
    cradle.visual(
        Cylinder(radius=0.052, length=0.010),
        origin=Origin(xyz=(-0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="rotor_backplate",
    )
    cradle.visual(
        mesh_from_cadquery(_rounded_box((0.045, 0.075, 0.055), 0.008), "pillow"),
        origin=Origin(),
        material=warm_leather,
        name="pillow",
    )
    cradle.visual(
        Box((0.008, 0.016, 0.090)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material=satin_black,
        name="watch_band",
    )
    cradle.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.030, 0.030),
                (0.042, 0.042),
                0.004,
                opening_shape="circle",
                outer_shape="circle",
                face=BezelFace(style="flat", front_lip=0.0, fillet=0.0005),
            ),
            "watch_case",
        ),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="watch_case",
    )
    cradle.visual(
        Cylinder(radius=0.0155, length=0.0030),
        origin=Origin(xyz=(0.034, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="watch_crystal",
    )

    control_knob = model.part("control_knob")
    control_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.028,
                0.014,
                body_style="faceted",
                grip=KnobGrip(style="ribbed", count=16, depth=0.0008),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "control_knob",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=champagne,
        name="knob_cap",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.28),
    )
    model.articulation(
        "cradle_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=WINDER_AXIS),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.4),
    )
    model.articulation(
        "control_knob_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=control_knob,
        origin=Origin(xyz=(BOX_X / 2.0 + 0.0045, -0.075, 0.066)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=3.0, lower=-0.75, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    knob = object_model.get_part("control_knob")
    lid_hinge = object_model.get_articulation("lid_hinge")
    cradle_spin = object_model.get_articulation("cradle_spin")
    knob_turn = object_model.get_articulation("control_knob_turn")

    ctx.allow_overlap(
        body,
        lid,
        elem_a="hinge_pin",
        elem_b="lid_barrel",
        reason="The polished hinge pin is intentionally captured inside the lid-side barrel.",
    )
    ctx.expect_within(
        body,
        lid,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="lid_barrel",
        margin=0.0008,
        name="hinge pin is centered in the lid barrel",
    )
    ctx.expect_overlap(
        body,
        lid,
        axes="y",
        elem_a="hinge_pin",
        elem_b="lid_barrel",
        min_overlap=0.070,
        name="hinge pin spans the lid knuckle",
    )
    ctx.allow_overlap(
        body,
        cradle,
        elem_a="cradle_bearing",
        elem_b="drive_shaft",
        reason="The winder shaft is intentionally captured in a close bearing sleeve.",
    )
    ctx.expect_within(
        cradle,
        body,
        axes="yz",
        inner_elem="drive_shaft",
        outer_elem="cradle_bearing",
        margin=0.001,
        name="drive shaft is centered in the bearing sleeve",
    )
    ctx.expect_overlap(
        cradle,
        body,
        axes="x",
        elem_a="drive_shaft",
        elem_b="cradle_bearing",
        min_overlap=0.006,
        name="drive shaft remains inserted in the fixed bearing",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_frame",
            negative_elem="top_gasket",
            max_gap=0.0015,
            max_penetration=0.0,
            name="closed lid lands on the top gasket seam",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_frame",
            elem_b="top_gasket",
            min_overlap=0.090,
            name="lid frame tracks the box rim footprint",
        )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.10}):
        opened_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward about rear hinge",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.12,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    rest_pillow = ctx.part_element_world_aabb(cradle, elem="pillow")
    with ctx.pose({cradle_spin: math.pi / 2.0}):
        spun_pillow = ctx.part_element_world_aabb(cradle, elem="pillow")
    if rest_pillow is not None and spun_pillow is not None:
        rest_y = rest_pillow[1][1] - rest_pillow[0][1]
        rest_z = rest_pillow[1][2] - rest_pillow[0][2]
        spun_y = spun_pillow[1][1] - spun_pillow[0][1]
        spun_z = spun_pillow[1][2] - spun_pillow[0][2]
        cradle_moves = spun_z > rest_z + 0.012 and spun_y < rest_y - 0.012
    else:
        cradle_moves = False
        rest_y = rest_z = spun_y = spun_z = None
    ctx.check(
        "winder cradle visibly rotates about its motor axis",
        cradle_moves,
        details=f"rest_y={rest_y}, rest_z={rest_z}, spun_y={spun_y}, spun_z={spun_z}",
    )

    ctx.expect_contact(
        knob,
        body,
        elem_a="knob_cap",
        elem_b="control_plate",
        contact_tol=0.001,
        name="control knob seats on its front plate",
    )
    with ctx.pose({knob_turn: 0.55}):
        ctx.expect_contact(
            knob,
            body,
            elem_a="knob_cap",
            elem_b="control_plate",
            contact_tol=0.001,
            name="control knob remains seated while turned",
        )

    return ctx.report()


object_model = build_object_model()
