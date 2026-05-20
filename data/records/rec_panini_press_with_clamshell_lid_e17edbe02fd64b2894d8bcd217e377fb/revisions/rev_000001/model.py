from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_L = 0.42
BODY_W = 0.36
BODY_H = 0.10
FOOT_H = 0.012
SIDE_PANEL_T = 0.018

HINGE_X = -0.175
HINGE_Z = 0.180
PLATEN_OPEN = math.radians(100.0)

LOWER_GRILL_L = 0.30
LOWER_GRILL_W = 0.25
LOWER_GRILL_T = 0.022
LOWER_GRILL_CENTER_X = 0.010
LOWER_GRILL_BOTTOM_Z = 0.102

UPPER_COVER_L = 0.34
UPPER_COVER_W = 0.28
UPPER_GRILL_L = 0.31
UPPER_GRILL_W = 0.25
UPPER_GRILL_T = 0.024
UPPER_GRILL_CENTER_X = 0.175
UPPER_GRILL_BOTTOM_Z = -0.054

DRIP_TRAY_TRAVEL = 0.150


def _box_at(
    length: float,
    width: float,
    height: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .translate((x, y, z))
    )


def _side_panel(sign: float) -> cq.Workplane:
    y_center = sign * (BODY_W / 2.0 - SIDE_PANEL_T / 2.0)
    panel = _box_at(BODY_L - 0.010, SIDE_PANEL_T, 0.175, y=y_center, z=FOOT_H)
    return panel.edges(">X and >Z").chamfer(0.055)


def _body_housing_shape() -> cq.Workplane:
    housing = _box_at(BODY_L, BODY_W, BODY_H, z=FOOT_H).edges("|Z").chamfer(0.008)

    grill_recess = _box_at(
        LOWER_GRILL_L + 0.030,
        LOWER_GRILL_W + 0.030,
        0.014,
        x=LOWER_GRILL_CENTER_X,
        z=FOOT_H + BODY_H - 0.010,
    )
    housing = housing.cut(grill_recess)

    drip_slot = _box_at(
        0.050,
        0.250,
        0.024,
        x=(BODY_L / 2.0) - 0.025,
        z=0.018,
    )
    housing = housing.cut(drip_slot)

    drip_channel = _box_at(
        0.308,
        0.242,
        0.026,
        x=0.056,
        z=0.017,
    )
    housing = housing.cut(drip_channel)

    housing = housing.union(_side_panel(1.0))
    housing = housing.union(_side_panel(-1.0))
    for y_pos in (-0.153, 0.153):
        housing = housing.union(_box_at(0.026, 0.036, 0.012, x=-0.162, y=y_pos, z=0.138))
    return housing


def _grill_plate(
    *,
    length: float,
    width: float,
    thickness: float,
    center_x: float,
    bottom_z: float,
    top_grooves: bool,
) -> cq.Workplane:
    plate = _box_at(length, width, thickness, x=center_x, z=bottom_z)
    spacing = 0.040
    groove_count = 7
    groove_width = 0.010
    groove_depth = 0.006
    first_x = center_x - spacing * (groove_count - 1) / 2.0

    for idx in range(groove_count):
        groove_x = first_x + (spacing * idx)
        groove_z = bottom_z + thickness - groove_depth if top_grooves else bottom_z
        groove = _box_at(groove_width, width * 0.92, groove_depth, x=groove_x, z=groove_z)
        plate = plate.cut(groove)

    return plate


def _upper_cover_shape() -> cq.Workplane:
    shoulder = _box_at(UPPER_COVER_L, UPPER_COVER_W, 0.020, x=UPPER_COVER_L / 2.0, z=-0.030)
    crown = _box_at(
        UPPER_COVER_L * 0.78,
        UPPER_COVER_W * 0.90,
        0.060,
        x=(UPPER_COVER_L / 2.0) + 0.010,
        z=-0.006,
    ).edges("|Z").fillet(0.020)
    rear_block = _box_at(0.052, UPPER_COVER_W, 0.050, x=0.026, z=-0.030)

    handle_bar = _box_at(0.030, 0.150, 0.022, x=0.236, z=0.052).edges("|Z").fillet(0.008)
    handle_post_0 = _box_at(0.028, 0.024, 0.048, x=0.224, y=-0.055, z=0.012)
    handle_post_1 = _box_at(0.028, 0.024, 0.048, x=0.224, y=0.055, z=0.012)

    return (
        shoulder.union(crown)
        .union(rear_block)
        .union(handle_bar)
        .union(handle_post_0)
        .union(handle_post_1)
    )


def _drip_tray_shape() -> cq.Workplane:
    tray_body = (
        cq.Workplane("XY")
        .box(0.275, 0.230, 0.018, centered=(True, True, False))
        .translate((-0.1375, 0.0, 0.0))
        .faces(">Z")
        .shell(-0.0025)
    )
    front_lip = _box_at(0.020, 0.242, 0.016, x=0.010, z=0.0)
    finger_pull = _box_at(0.010, 0.100, 0.006, x=0.018, z=0.004)
    side_rail_0 = _box_at(0.220, 0.006, 0.010, x=-0.125, y=-0.118, z=0.004)
    side_rail_1 = _box_at(0.220, 0.006, 0.010, x=-0.125, y=0.118, z=0.004)
    return tray_body.union(front_lip).union(finger_pull).union(side_rail_0).union(side_rail_1)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_panini_press")

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    grill_iron = model.material("grill_iron", rgba=(0.17, 0.17, 0.18, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_housing_shape(), "body_housing"),
        material=stainless,
        name="housing",
    )
    body.visual(
        mesh_from_cadquery(
            _grill_plate(
                length=LOWER_GRILL_L,
                width=LOWER_GRILL_W,
                thickness=LOWER_GRILL_T,
                center_x=LOWER_GRILL_CENTER_X,
                bottom_z=LOWER_GRILL_BOTTOM_Z,
                top_grooves=True,
            ),
            "lower_grill",
        ),
        material=grill_iron,
        name="lower_grill",
    )
    foot_positions = (
        (-(BODY_L / 2.0) + 0.050, -(BODY_W / 2.0) + 0.050),
        (-(BODY_L / 2.0) + 0.050, (BODY_W / 2.0) - 0.050),
        ((BODY_L / 2.0) - 0.050, -(BODY_W / 2.0) + 0.050),
        ((BODY_L / 2.0) - 0.050, (BODY_W / 2.0) - 0.050),
    )
    for idx, (x_pos, y_pos) in enumerate(foot_positions):
        body.visual(
            Cylinder(radius=0.016, length=FOOT_H),
            origin=Origin(xyz=(x_pos, y_pos, FOOT_H / 2.0)),
            material=rubber,
            name=f"foot_{idx}",
        )

    platen = model.part("platen")
    platen.visual(
        mesh_from_cadquery(_upper_cover_shape(), "upper_cover"),
        material=stainless,
        name="upper_cover",
    )
    platen.visual(
        mesh_from_cadquery(
            _grill_plate(
                length=UPPER_GRILL_L,
                width=UPPER_GRILL_W,
                thickness=UPPER_GRILL_T,
                center_x=UPPER_GRILL_CENTER_X,
                bottom_z=UPPER_GRILL_BOTTOM_Z,
                top_grooves=False,
            ),
            "upper_grill",
        ),
        material=grill_iron,
        name="upper_grill",
    )

    temp_dial = model.part("temp_dial")
    temp_dial.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="dial_stem",
    )
    temp_dial.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="dial_knob",
    )
    temp_dial.visual(
        Box((0.010, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, -0.024, 0.020)),
        material=stainless,
        name="dial_marker",
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        mesh_from_cadquery(_drip_tray_shape(), "drip_tray"),
        material=stainless,
        name="tray_pan",
    )

    model.articulation(
        "body_to_platen",
        ArticulationType.REVOLUTE,
        parent=body,
        child=platen,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=PLATEN_OPEN,
        ),
    )
    model.articulation(
        "body_to_temp_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=temp_dial,
        origin=Origin(xyz=(0.082, -(BODY_W / 2.0), 0.094)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0),
    )
    model.articulation(
        "body_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=((BODY_L / 2.0) - 0.023, 0.0, 0.019)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.25,
            lower=0.0,
            upper=DRIP_TRAY_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    platen = object_model.get_part("platen")
    temp_dial = object_model.get_part("temp_dial")
    drip_tray = object_model.get_part("drip_tray")

    platen_hinge = object_model.get_articulation("body_to_platen")
    tray_slide = object_model.get_articulation("body_to_drip_tray")

    ctx.allow_overlap(
        body,
        drip_tray,
        elem_a="housing",
        elem_b="tray_pan",
        reason="The removable drip tray is represented as sliding within a simplified body housing proxy around the slot channel.",
    )
    ctx.allow_overlap(
        body,
        platen,
        elem_a="housing",
        elem_b="upper_cover",
        reason="The domed lid cover intentionally nests over the open cooking cavity and side-cheek proxy while the separate grill elements carry the true cooking-surface clearance.",
    )

    with ctx.pose({platen_hinge: 0.0}):
        ctx.expect_gap(
            platen,
            body,
            axis="z",
            positive_elem="upper_grill",
            negative_elem="lower_grill",
            max_gap=0.004,
            max_penetration=0.0,
            name="closed platen hovers just above lower grill",
        )
        ctx.expect_overlap(
            platen,
            body,
            axes="xy",
            elem_a="upper_grill",
            elem_b="lower_grill",
            min_overlap=0.220,
            name="closed grill plates cover the same cooking area",
        )

    with ctx.pose({platen_hinge: PLATEN_OPEN}):
        ctx.expect_gap(
            platen,
            body,
            axis="z",
            positive_elem="upper_grill",
            negative_elem="lower_grill",
            min_gap=0.075,
            name="opened platen lifts clear of the lower grill",
        )

    ctx.expect_contact(
        temp_dial,
        body,
        elem_a="dial_stem",
        elem_b="housing",
        name="temperature dial seats on the right side panel",
    )

    with ctx.pose({tray_slide: 0.0}):
        ctx.expect_within(
            drip_tray,
            body,
            axes="yz",
            margin=0.003,
            name="collapsed drip tray stays centered in the front slot",
        )
        ctx.expect_overlap(
            drip_tray,
            body,
            axes="x",
            min_overlap=0.180,
            name="collapsed drip tray remains deeply inserted",
        )
        tray_rest = ctx.part_world_position(drip_tray)

    with ctx.pose({tray_slide: DRIP_TRAY_TRAVEL}):
        ctx.expect_within(
            drip_tray,
            body,
            axes="yz",
            margin=0.003,
            name="extended drip tray stays guided by the slot",
        )
        ctx.expect_overlap(
            drip_tray,
            body,
            axes="x",
            min_overlap=0.045,
            name="extended drip tray still retains insertion",
        )
        tray_extended = ctx.part_world_position(drip_tray)

    ctx.check(
        "drip tray pulls forward from the front apron",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[0] > tray_rest[0] + 0.100,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    return ctx.report()


object_model = build_object_model()
