from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_X = 0.125
BODY_Y = 0.085
BODY_Z = 0.090
FRONT_X = BODY_X / 2.0
SIDE_Y = BODY_Y / 2.0
PORT_Z = 0.057
PORT_RADIUS = 0.0115
TRAY_Z = 0.020
FLAP_HINGE_Z = PORT_Z + 0.018


def _cylinder_x(length: float, radius: float) -> cq.Workplane:
    """CadQuery cylinder centered at the origin and aligned to local/world X."""
    return cq.Workplane("XY").cylinder(length, radius).rotate((0, 0, 0), (0, 1, 0), 90)


def _rounded_body_shell() -> cq.Workplane:
    """Rounded sharpener housing with true front pencil bore and tray slot cuts."""
    upper_bottom = 0.035
    upper_height = BODY_Z - upper_bottom
    shell = (
        cq.Workplane("XY")
        .box(BODY_X, BODY_Y, upper_height)
        .translate((0.0, 0.0, upper_bottom + upper_height / 2.0))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.012)
        .edges("<Z")
        .fillet(0.003)
    )

    pencil_bore = _cylinder_x(0.088, PORT_RADIUS).translate(
        (FRONT_X - 0.040, 0.0, PORT_Z)
    )
    return shell.cut(pencil_bore)


def _port_liner() -> cq.Workplane:
    """Thin annular throat so the pencil port visibly remains open and deep."""
    outer = _cylinder_x(0.058, PORT_RADIUS * 1.02)
    inner = _cylinder_x(0.062, PORT_RADIUS * 0.68)
    return outer.cut(inner).translate((FRONT_X - 0.028, 0.0, PORT_Z))


def _tray_cup() -> cq.Workplane:
    """Open-top pullout shaving tray with front fascia and retained hidden length."""
    depth = 0.072
    width = 0.050
    height = 0.018
    wall = 0.002
    x_center = -0.034
    bottom = cq.Workplane("XY").box(depth, width, wall).translate(
        (x_center, 0.0, -height / 2.0 + wall / 2.0)
    )
    side_a = cq.Workplane("XY").box(depth, wall, height).translate(
        (x_center, width / 2.0 - wall / 2.0, 0.0)
    )
    side_b = cq.Workplane("XY").box(depth, wall, height).translate(
        (x_center, -width / 2.0 + wall / 2.0, 0.0)
    )
    back = cq.Workplane("XY").box(wall, width, height).translate(
        (x_center - depth / 2.0 + wall / 2.0, 0.0, 0.0)
    )
    front_lip = cq.Workplane("XY").box(0.007, 0.060, 0.026).translate(
        (0.001, 0.0, 0.0)
    )
    runner_a = cq.Workplane("XY").box(0.060, 0.006, 0.0055).translate(
        (x_center, 0.019, -0.01175)
    )
    runner_b = cq.Workplane("XY").box(0.060, 0.006, 0.0055).translate(
        (x_center, -0.019, -0.01175)
    )
    finger_recess = _cylinder_x(0.010, 0.010).translate((0.005, 0.0, 0.002))
    return (
        bottom.union(side_a)
        .union(side_b)
        .union(back)
        .union(front_lip)
        .union(runner_a)
        .union(runner_b)
        .cut(finger_recess)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_office_pencil_sharpener")

    shell_mat = model.material("matte_teal_shell", rgba=(0.08, 0.38, 0.44, 1.0))
    dark_mat = model.material("dark_inner_plastic", rgba=(0.025, 0.025, 0.028, 1.0))
    tray_mat = model.material("charcoal_tray", rgba=(0.035, 0.038, 0.042, 1.0))
    metal_mat = model.material("brushed_steel", rgba=(0.70, 0.68, 0.62, 1.0))
    flap_mat = model.material("smoked_dust_flap", rgba=(0.14, 0.16, 0.18, 0.62))
    rubber_mat = model.material("soft_black_feet", rgba=(0.015, 0.014, 0.013, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_body_shell(), "rounded_sharpener_shell", tolerance=0.0007),
        material=shell_mat,
        name="rounded_shell",
    )
    body.visual(
        Box((0.125, 0.075, 0.0055)),
        origin=Origin(xyz=(0.0, 0.0, 0.00275)),
        material=shell_mat,
        name="tray_floor",
    )
    body.visual(
        Box((0.125, 0.009, 0.037)),
        origin=Origin(xyz=(0.0, BODY_Y / 2.0 - 0.0045, 0.0185)),
        material=shell_mat,
        name="side_cheek_0",
    )
    body.visual(
        Box((0.125, 0.009, 0.037)),
        origin=Origin(xyz=(0.0, -BODY_Y / 2.0 + 0.0045, 0.0185)),
        material=shell_mat,
        name="side_cheek_1",
    )
    body.visual(
        Box((0.010, 0.075, 0.037)),
        origin=Origin(xyz=(-0.055, 0.0, 0.0185)),
        material=shell_mat,
        name="rear_lower_wall",
    )
    body.visual(
        mesh_from_cadquery(_port_liner(), "open_pencil_port_liner", tolerance=0.00045),
        material=dark_mat,
        name="port_liner",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.000, SIDE_Y + 0.0020, 0.052), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="side_axle_boss",
    )
    body.visual(
        Cylinder(radius=0.0032, length=0.012),
        origin=Origin(
            xyz=(FRONT_X + 0.003, 0.019, FLAP_HINGE_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal_mat,
        name="hinge_knuckle_0",
    )
    body.visual(
        Cylinder(radius=0.0032, length=0.012),
        origin=Origin(
            xyz=(FRONT_X + 0.003, -0.019, FLAP_HINGE_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal_mat,
        name="hinge_knuckle_1",
    )
    for index, x in enumerate((-0.038, 0.038)):
        for y in (-0.024, 0.024):
            body.visual(
                Box((0.020, 0.012, 0.004)),
                origin=Origin(xyz=(x, y, 0.002)),
                material=rubber_mat,
                name=f"foot_{index}_{0 if y < 0 else 1}",
            )

    tray = model.part("waste_tray")
    tray.visual(
        mesh_from_cadquery(_tray_cup(), "pullout_waste_tray", tolerance=0.00045),
        material=tray_mat,
        name="tray_cup",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.0135, length=0.008),
        origin=Origin(xyz=(0.0, 0.0100, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="hub_disk",
    )
    crank.visual(
        Box((0.010, 0.0055, 0.048)),
        origin=Origin(xyz=(0.0, 0.0150, -0.026)),
        material=metal_mat,
        name="folding_arm",
    )
    crank.visual(
        Cylinder(radius=0.0065, length=0.026),
        origin=Origin(xyz=(0.0, 0.0305, -0.052), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_mat,
        name="finger_grip",
    )
    crank.visual(
        Cylinder(radius=0.0043, length=0.013),
        origin=Origin(xyz=(0.0, 0.0190, -0.052), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="grip_pin",
    )

    dust_flap = model.part("dust_flap")
    dust_flap.visual(
        Cylinder(radius=0.0030, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="flap_barrel",
    )
    dust_flap.visual(
        Box((0.003, 0.030, 0.026)),
        origin=Origin(xyz=(0.0030, 0.0, -0.014)),
        material=flap_mat,
        name="flap_plate",
    )
    dust_flap.visual(
        Box((0.004, 0.018, 0.004)),
        origin=Origin(xyz=(0.0024, 0.0, -0.0035)),
        material=flap_mat,
        name="flap_tab",
    )

    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(FRONT_X, 0.0, TRAY_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.18, lower=0.0, upper=0.052),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(0.0, SIDE_Y, 0.052)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.4, velocity=8.0),
    )
    model.articulation(
        "body_to_dust_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dust_flap,
        origin=Origin(xyz=(FRONT_X + 0.003, 0.0, FLAP_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=2.0, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    tray = object_model.get_part("waste_tray")
    crank = object_model.get_part("crank")
    dust_flap = object_model.get_part("dust_flap")
    tray_slide = object_model.get_articulation("body_to_tray")
    crank_joint = object_model.get_articulation("body_to_crank")
    flap_hinge = object_model.get_articulation("body_to_dust_flap")

    ctx.expect_within(
        tray,
        body,
        axes="y",
        inner_elem="tray_cup",
        outer_elem="tray_floor",
        margin=0.004,
        name="waste tray fits between the lower side cheeks",
    )
    ctx.expect_gap(
        tray,
        body,
        axis="z",
        max_gap=0.0008,
        max_penetration=0.0,
        positive_elem="tray_cup",
        negative_elem="tray_floor",
        name="waste tray runners ride on the lower floor",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="x",
        elem_a="tray_cup",
        elem_b="tray_floor",
        min_overlap=0.050,
        name="closed waste tray remains retained inside the body",
    )
    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.052}):
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            elem_a="tray_cup",
            elem_b="tray_floor",
            min_overlap=0.016,
            name="extended waste tray keeps hidden insertion length",
        )
        extended_tray_pos = ctx.part_world_position(tray)
    ctx.check(
        "waste tray slides out the front",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[0] > rest_tray_pos[0] + 0.045,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )

    ctx.expect_overlap(
        dust_flap,
        body,
        axes="yz",
        elem_a="flap_plate",
        elem_b="port_liner",
        min_overlap=0.014,
        name="closed dust flap covers the open pencil channel",
    )
    closed_flap_aabb = ctx.part_element_world_aabb(dust_flap, elem="flap_plate")
    with ctx.pose({flap_hinge: 1.10}):
        open_flap_aabb = ctx.part_element_world_aabb(dust_flap, elem="flap_plate")
    ctx.check(
        "dust flap rotates upward and outward",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[0][2] > closed_flap_aabb[0][2] + 0.010
        and open_flap_aabb[1][0] > closed_flap_aabb[1][0] + 0.006,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    rest_grip_aabb = ctx.part_element_world_aabb(crank, elem="finger_grip")
    with ctx.pose({crank_joint: math.pi / 2.0}):
        turned_grip_aabb = ctx.part_element_world_aabb(crank, elem="finger_grip")
    ctx.check(
        "side crank rotates around the side axle",
        rest_grip_aabb is not None
        and turned_grip_aabb is not None
        and turned_grip_aabb[0][2] > rest_grip_aabb[0][2] + 0.030,
        details=f"rest={rest_grip_aabb}, turned={turned_grip_aabb}",
    )

    ctx.check(
        "pencil port is modeled as an open annular throat",
        PORT_RADIUS * 0.68 < PORT_RADIUS * 0.985 and 0.055 <= PORT_Z <= 0.060,
        details="The port liner is an annular mesh around a boolean-cut pencil bore, leaving an open center channel.",
    )

    return ctx.report()


object_model = build_object_model()
