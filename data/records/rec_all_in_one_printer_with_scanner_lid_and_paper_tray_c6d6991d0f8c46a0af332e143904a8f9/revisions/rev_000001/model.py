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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    place_on_surface,
)


BODY_W = 0.44
BODY_D = 0.36
BODY_H = 0.18
DECK_Z = 0.162
FRONT_TOP_Z = 0.112
CONTROL_Y_REAR = 0.070
CONTROL_Y_FRONT = BODY_D / 2
CONTROL_SLOPE = math.atan2(DECK_Z - FRONT_TOP_Z, CONTROL_Y_FRONT - CONTROL_Y_REAR)

SCANNER_W = 0.322
SCANNER_D = 0.226
SCANNER_CAVITY_Y = -0.052
SCANNER_FLOOR_Z = 0.084

LID_W = 0.342
LID_D = 0.246
LID_T = 0.024
LID_HINGE_Y = -BODY_D / 2 + 0.018
LID_HINGE_Z = DECK_Z - 0.004

TRAY_W = 0.310
TRAY_D = 0.192
TRAY_FLOOR_T = 0.0035
TRAY_WALL_T = 0.0035
TRAY_WALL_H = 0.034
TRAY_FRONT_H = 0.048
TRAY_FRONT_LIP = 0.010
TRAY_ORIGIN_Y = BODY_D / 2 - 0.013
TRAY_ORIGIN_Z = 0.012
TRAY_TRAVEL = 0.092

SHELF_W = 0.286
SHELF_D = 0.094
SHELF_T = 0.0035
SHELF_RAIL_H = 0.015
SHELF_RAIL_T = 0.0035
SHELF_FRONT_LIP_H = 0.012
SHELF_HINGE_Y = BODY_D / 2 - 0.004
SHELF_HINGE_Z = 0.075

KEY_W = 0.012
KEY_D = 0.008
KEY_H = 0.0034
KEY_TRAVEL = 0.0024
KEY_POSITIONS = (
    (-0.062, 0.140),
    (-0.062, 0.126),
    (-0.062, 0.112),
    (0.066, 0.140),
    (0.066, 0.126),
    (0.066, 0.112),
)

DIAL_X = 0.122
DIAL_Y = 0.126
DIAL_D = 0.022
DIAL_H = 0.010


def strip_surface_point(x: float, y: float, proud: float = 0.0) -> tuple[float, float, float]:
    span = (y - CONTROL_Y_REAR) / (CONTROL_Y_FRONT - CONTROL_Y_REAR)
    z = DECK_Z + span * (FRONT_TOP_Z - DECK_Z)
    return (
        x,
        y + proud * math.sin(CONTROL_SLOPE),
        z + proud * math.cos(CONTROL_SLOPE),
    )


def strip_surface_origin(x: float, y: float, proud: float = 0.0) -> Origin:
    return Origin(
        xyz=strip_surface_point(x, y, proud=proud),
        rpy=(-CONTROL_SLOPE, 0.0, 0.0),
    )


def make_body_shape() -> cq.Workplane:
    side_profile = [
        (-BODY_D / 2, 0.0),
        (BODY_D / 2, 0.0),
        (BODY_D / 2, FRONT_TOP_Z),
        (CONTROL_Y_REAR, DECK_Z),
        (-BODY_D / 2, DECK_Z),
    ]
    outer = (
        cq.Workplane("YZ")
        .polyline(side_profile)
        .close()
        .extrude(BODY_W)
        .translate((-BODY_W / 2, 0.0, 0.0))
    )

    scanner_cut = (
        cq.Workplane("XY")
        .box(SCANNER_W, SCANNER_D, BODY_H, centered=(True, True, False))
        .translate((0.0, SCANNER_CAVITY_Y, SCANNER_FLOOR_Z))
    )
    tray_cut = (
        cq.Workplane("XY")
        .box(0.332, 0.190, 0.058, centered=(True, True, False))
        .translate((0.0, BODY_D / 2 - 0.087, 0.010))
    )
    output_cut = (
        cq.Workplane("XY")
        .box(0.300, 0.090, 0.022, centered=(True, True, False))
        .translate((0.0, BODY_D / 2 - 0.039, 0.080))
    )
    underside_relief = (
        cq.Workplane("XY")
        .box(0.220, 0.160, 0.020, centered=(True, True, False))
        .translate((0.0, 0.005, 0.0))
    )
    return outer.cut(scanner_cut).cut(tray_cut).cut(output_cut).cut(underside_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_all_in_one_printer")

    body_white = model.material("body_white", rgba=(0.92, 0.93, 0.94, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.18, 0.20, 0.22, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.18, 0.24, 0.30, 0.85))
    paper_gray = model.material("paper_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    button_gray = model.material("button_gray", rgba=(0.86, 0.87, 0.88, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(make_body_shape(), "body_shell"),
        material=body_white,
        name="body_shell",
    )
    body.visual(
        Box((SCANNER_W - 0.020, SCANNER_D - 0.020, 0.003)),
        origin=Origin(xyz=(0.0, SCANNER_CAVITY_Y, SCANNER_FLOOR_Z + 0.0015)),
        material=glass_dark,
        name="scanner_glass",
    )
    body.visual(
        Box((0.054, 0.030, 0.0025)),
        origin=strip_surface_origin(0.018, 0.126, proud=0.00125),
        material=glass_dark,
        name="display",
    )
    body.visual(
        Cylinder(radius=0.0045, length=BODY_W - 0.110),
        origin=Origin(
            xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z - 0.001),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_dark,
        name="lid_hinge_bar",
    )
    lid = model.part("lid")
    lid.visual(
        Box((LID_W, LID_D - 0.008, 0.004)),
        origin=Origin(xyz=(0.0, LID_D / 2 + 0.004, LID_T - 0.002)),
        material=body_white,
        name="lid_top",
    )
    lid.visual(
        Box((LID_W - 0.020, 0.010, LID_T - 0.004)),
        origin=Origin(xyz=(0.0, LID_D - 0.005, (LID_T - 0.004) / 2)),
        material=body_white,
        name="lid_front",
    )
    lid.visual(
        Box((0.010, LID_D - 0.020, LID_T - 0.004)),
        origin=Origin(
            xyz=(LID_W / 2 - 0.005, LID_D / 2, (LID_T - 0.004) / 2),
        ),
        material=body_white,
        name="lid_side_0",
    )
    lid.visual(
        Box((0.010, LID_D - 0.020, LID_T - 0.004)),
        origin=Origin(
            xyz=(-LID_W / 2 + 0.005, LID_D / 2, (LID_T - 0.004) / 2),
        ),
        material=body_white,
        name="lid_side_1",
    )
    lid.visual(
        Box((LID_W, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.008, 0.005)),
        material=body_white,
        name="lid_rear",
    )
    lid.visual(
        Cylinder(radius=0.0065, length=LID_W - 0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="lid_barrel",
    )

    tray = model.part("tray")
    tray.visual(
        Box((TRAY_W, TRAY_D, TRAY_FLOOR_T)),
        origin=Origin(xyz=(0.0, -TRAY_D / 2 + TRAY_FRONT_LIP, TRAY_FLOOR_T / 2)),
        material=paper_gray,
        name="tray_floor",
    )
    tray.visual(
        Box((TRAY_W, TRAY_WALL_T, TRAY_FRONT_H)),
        origin=Origin(xyz=(0.0, -TRAY_WALL_T / 2, TRAY_FRONT_H / 2)),
        material=trim_dark,
        name="tray_front",
    )
    tray.visual(
        Box((TRAY_W, TRAY_WALL_T, TRAY_WALL_H)),
        origin=Origin(
            xyz=(0.0, -TRAY_D + TRAY_WALL_T / 2 + TRAY_FRONT_LIP, TRAY_WALL_H / 2),
        ),
        material=paper_gray,
        name="tray_rear",
    )
    tray.visual(
        Box((TRAY_WALL_T, TRAY_D - TRAY_WALL_T, TRAY_WALL_H)),
        origin=Origin(
            xyz=(
                TRAY_W / 2 - TRAY_WALL_T / 2,
                -TRAY_D / 2 - TRAY_WALL_T / 2 + TRAY_FRONT_LIP / 2,
                TRAY_WALL_H / 2,
            ),
        ),
        material=paper_gray,
        name="tray_side_0",
    )
    tray.visual(
        Box((TRAY_WALL_T, TRAY_D - TRAY_WALL_T, TRAY_WALL_H)),
        origin=Origin(
            xyz=(
                -TRAY_W / 2 + TRAY_WALL_T / 2,
                -TRAY_D / 2 - TRAY_WALL_T / 2 + TRAY_FRONT_LIP / 2,
                TRAY_WALL_H / 2,
            ),
        ),
        material=paper_gray,
        name="tray_side_1",
    )

    shelf = model.part("shelf")
    shelf.visual(
        Box((SHELF_W, SHELF_T, SHELF_D)),
        origin=Origin(xyz=(0.0, SHELF_T / 2, -SHELF_D / 2)),
        material=paper_gray,
        name="shelf_panel",
    )
    shelf.visual(
        Box((SHELF_RAIL_T, SHELF_RAIL_H, SHELF_D - SHELF_T)),
        origin=Origin(
            xyz=(
                SHELF_W / 2 - SHELF_RAIL_T / 2,
                SHELF_RAIL_H / 2,
                -(SHELF_D - SHELF_T) / 2,
            ),
        ),
        material=paper_gray,
        name="shelf_side_0",
    )
    shelf.visual(
        Box((SHELF_RAIL_T, SHELF_RAIL_H, SHELF_D - SHELF_T)),
        origin=Origin(
            xyz=(
                -SHELF_W / 2 + SHELF_RAIL_T / 2,
                SHELF_RAIL_H / 2,
                -(SHELF_D - SHELF_T) / 2,
            ),
        ),
        material=paper_gray,
        name="shelf_side_1",
    )
    shelf.visual(
        Box((SHELF_W, SHELF_FRONT_LIP_H, SHELF_T)),
        origin=Origin(
            xyz=(0.0, SHELF_FRONT_LIP_H / 2, -SHELF_D + SHELF_T / 2),
        ),
        material=paper_gray,
        name="shelf_front_lip",
    )
    shelf.visual(
        Cylinder(radius=0.003, length=SHELF_W - 0.030),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="shelf_barrel",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.2, lower=0.0, upper=1.42),
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, TRAY_ORIGIN_Y, TRAY_ORIGIN_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.25, lower=0.0, upper=TRAY_TRAVEL),
    )
    model.articulation(
        "body_to_shelf",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shelf,
        origin=Origin(xyz=(0.0, SHELF_HINGE_Y, SHELF_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=0.0, upper=1.48),
    )

    for index, (button_x, button_y) in enumerate(KEY_POSITIONS):
        key = model.part(f"key_{index}")
        key.visual(
            Box((KEY_W, KEY_D, KEY_H)),
            origin=Origin(xyz=(0.0, 0.0, KEY_H / 2)),
            material=button_gray,
            name="key_cap",
        )
        model.articulation(
            f"body_to_key_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=place_on_surface(
                key,
                body,
                point_hint=strip_surface_point(button_x, button_y, proud=0.010),
                child_axis="+z",
                clearance=0.0,
                up_hint=(1.0, 0.0, 0.0),
            ),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=0.08,
                lower=0.0,
                upper=KEY_TRAVEL,
            ),
        )

    dial = model.part("menu_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                DIAL_D,
                DIAL_H,
                body_style="cylindrical",
                edge_radius=0.0012,
                grip=KnobGrip(style="fluted", count=16, depth=0.0007),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "menu_dial",
        ),
        material=trim_dark,
        name="dial_cap",
    )
    model.articulation(
        "body_to_menu_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=place_on_surface(
            dial,
            body,
            point_hint=strip_surface_point(DIAL_X, DIAL_Y, proud=0.012),
            child_axis="+z",
            clearance=0.0,
            up_hint=(1.0, 0.0, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("tray")
    shelf = object_model.get_part("shelf")
    key_0 = object_model.get_part("key_0")
    key_5 = object_model.get_part("key_5")

    lid_hinge = object_model.get_articulation("body_to_lid")
    tray_slide = object_model.get_articulation("body_to_tray")
    shelf_hinge = object_model.get_articulation("body_to_shelf")
    key_0_joint = object_model.get_articulation("body_to_key_0")
    dial_joint = object_model.get_articulation("body_to_menu_dial")

    ctx.expect_overlap(
        lid,
        body,
        axes="x",
        elem_a="lid_top",
        elem_b="scanner_glass",
        min_overlap=0.250,
        name="lid spans the scanner width",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="xz",
        min_overlap=0.030,
        name="tray stays aligned with the lower body opening at rest",
    )
    ctx.expect_gap(
        body,
        tray,
        axis="z",
        positive_elem="scanner_glass",
        negative_elem="tray_front",
        min_gap=0.020,
        name="scanner deck sits above the lower tray front",
    )

    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: TRAY_TRAVEL}):
        ctx.expect_overlap(
            tray,
            body,
            axes="xz",
            min_overlap=0.030,
            name="extended tray remains laterally retained by the body",
        )
        tray_extended = ctx.part_world_position(tray)
    ctx.check(
        "paper tray extends forward",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[1] > tray_rest[1] + 0.080,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    lid_closed = ctx.part_element_world_aabb(lid, elem="lid_front")
    with ctx.pose({lid_hinge: 1.15}):
        lid_open = ctx.part_element_world_aabb(lid, elem="lid_front")
    ctx.check(
        "scanner lid opens upward",
        lid_closed is not None
        and lid_open is not None
        and lid_open[1][2] > lid_closed[1][2] + 0.120,
        details=f"closed={lid_closed}, open={lid_open}",
    )

    shelf_closed = ctx.part_element_world_aabb(shelf, elem="shelf_front_lip")
    with ctx.pose({shelf_hinge: 1.40}):
        shelf_open = ctx.part_element_world_aabb(shelf, elem="shelf_front_lip")
    ctx.check(
        "output shelf folds outward",
        shelf_closed is not None
        and shelf_open is not None
        and shelf_open[0][1] > shelf_closed[0][1] + 0.070,
        details=f"closed={shelf_closed}, open={shelf_open}",
    )

    key_names = [f"key_{index}" for index in range(6)]
    key_joint_names = [f"body_to_key_{index}" for index in range(6)]
    for key_name in key_names:
        ctx.allow_overlap(
            body,
            key_name,
            elem_a="body_shell",
            elem_b="key_cap",
            reason="The sloped control strip is modeled without explicit switch recesses, so each button cap is allowed a shallow panel embedding at rest.",
        )
    ctx.allow_overlap(
        body,
        "menu_dial",
        elem_a="body_shell",
        elem_b="dial_cap",
        reason="The dial is represented as a surface control mounted into the simplified sloped panel without a separate bezel recess.",
    )
    ctx.check(
        "six independent keys are present",
        all(object_model.get_part(name).name == name for name in key_names)
        and all(
            object_model.get_articulation(name).articulation_type == ArticulationType.PRISMATIC
            for name in key_joint_names
        ),
        details=f"keys={key_names}, joints={key_joint_names}",
    )
    ctx.expect_overlap(
        key_0,
        body,
        axes="xy",
        min_overlap=0.006,
        name="left key sits within the control strip footprint",
    )
    ctx.expect_overlap(
        key_5,
        body,
        axes="xy",
        min_overlap=0.006,
        name="right key sits within the control strip footprint",
    )

    key_rest = ctx.part_world_position(key_0)
    with ctx.pose({key_0_joint: KEY_TRAVEL}):
        key_pressed = ctx.part_world_position(key_0)
    ctx.check(
        "a key presses inward along the sloped strip",
        key_rest is not None
        and key_pressed is not None
        and key_pressed[1] < key_rest[1] - 0.0006
        and key_pressed[2] < key_rest[2] - 0.0015,
        details=f"rest={key_rest}, pressed={key_pressed}",
    )
    ctx.check(
        "menu dial uses continuous rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
