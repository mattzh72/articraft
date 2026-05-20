from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.140
BODY_D = 0.105
BODY_H = 0.096
PORT_Z = 0.014
PORT_R = 0.008
DRAWER_Z = -0.029
CRANK_Y = 0.010
CRANK_Z = 0.010


def _body_shell():
    body = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    body = body.edges().fillet(0.006)

    cutter_cavity = (
        cq.Workplane("XY")
        .box(0.074, 0.064, 0.056)
        .translate((0.0, -0.002, PORT_Z - 0.004))
    )
    body = body.cut(cutter_cavity)

    pencil_throat = (
        cq.Workplane("XZ")
        .center(0.0, PORT_Z)
        .circle(PORT_R)
        .extrude(BODY_D + 0.024)
        .translate((0.0, -BODY_D / 2.0 - 0.012, 0.0))
    )
    body = body.cut(pencil_throat)

    drawer_slot = (
        cq.Workplane("XY")
        .box(0.098, 0.088, 0.034)
        .translate((0.0, -BODY_D / 2.0 + 0.040, DRAWER_Z))
    )
    body = body.cut(drawer_slot)

    crank_axle_bore = (
        cq.Workplane("YZ")
        .center(CRANK_Y, CRANK_Z)
        .circle(0.0082)
        .extrude(BODY_W + 0.040)
        .translate((-BODY_W / 2.0 - 0.020, 0.0, 0.0))
    )
    body = body.cut(crank_axle_bore)

    return body


def _ring_y(outer_radius: float, inner_radius: float, depth: float):
    return (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(depth)
        .translate((0.0, depth / 2.0, 0.0))
    )


def _ring_x(outer_radius: float, inner_radius: float, depth: float):
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(depth)
        .translate((-depth / 2.0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_metal_classroom_pencil_sharpener")

    cast_metal = model.material("cast_metal", rgba=(0.48, 0.50, 0.50, 1.0))
    worn_edge = model.material("worn_edge", rgba=(0.66, 0.68, 0.66, 1.0))
    drawer_metal = model.material("drawer_metal", rgba=(0.38, 0.39, 0.38, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.105, 0.105, 1.0))
    black_dial = model.material("black_dial", rgba=(0.04, 0.045, 0.045, 1.0))
    ivory_mark = model.material("ivory_mark", rgba=(0.88, 0.82, 0.66, 1.0))
    wood_bakelite = model.material("wood_bakelite", rgba=(0.29, 0.14, 0.055, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_body_shell(), "cast_body_shell", tolerance=0.0006),
        material=cast_metal,
        name="body_shell",
    )
    housing.visual(
        Box((0.080, 0.036, 0.004)),
        origin=Origin(xyz=(0.0, -0.012, BODY_H / 2.0 + 0.0015)),
        material=worn_edge,
        name="top_label_plate",
    )
    housing.visual(
        Box((0.022, 0.020, 0.009)),
        origin=Origin(xyz=(-0.050, -0.032, -BODY_H / 2.0 - 0.0035)),
        material=cast_metal,
        name="foot_0",
    )
    housing.visual(
        Box((0.022, 0.020, 0.009)),
        origin=Origin(xyz=(0.050, -0.032, -BODY_H / 2.0 - 0.0035)),
        material=cast_metal,
        name="foot_1",
    )
    housing.visual(
        Box((0.022, 0.020, 0.009)),
        origin=Origin(xyz=(-0.050, 0.032, -BODY_H / 2.0 - 0.0035)),
        material=cast_metal,
        name="foot_2",
    )
    housing.visual(
        Box((0.022, 0.020, 0.009)),
        origin=Origin(xyz=(0.050, 0.032, -BODY_H / 2.0 - 0.0035)),
        material=cast_metal,
        name="foot_3",
    )
    housing.visual(
        mesh_from_cadquery(_ring_x(0.020, 0.0085, 0.017), "side_bearing_collar", tolerance=0.0005),
        origin=Origin(xyz=(BODY_W / 2.0 + 0.006, CRANK_Y, CRANK_Z)),
        material=cast_metal,
        name="side_bearing",
    )
    for idx, (sx, sz) in enumerate(
        (
            (-0.046, PORT_Z + 0.028),
            (0.046, PORT_Z + 0.028),
            (-0.046, PORT_Z - 0.038),
            (0.046, PORT_Z - 0.038),
        )
    ):
        housing.visual(
            Cylinder(radius=0.0036, length=0.0025),
            origin=Origin(xyz=(sx, -BODY_D / 2.0 - 0.0011, sz), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=worn_edge,
            name=f"front_screw_{idx}",
        )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.102, 0.007, 0.034)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=drawer_metal,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.084, 0.076, 0.003)),
        origin=Origin(xyz=(0.0, 0.028, -0.009)),
        material=drawer_metal,
        name="tray_floor",
    )
    drawer.visual(
        Box((0.004, 0.076, 0.017)),
        origin=Origin(xyz=(-0.042, 0.028, -0.001)),
        material=drawer_metal,
        name="tray_side_0",
    )
    drawer.visual(
        Box((0.004, 0.076, 0.017)),
        origin=Origin(xyz=(0.042, 0.028, -0.001)),
        material=drawer_metal,
        name="tray_side_1",
    )
    drawer.visual(
        Box((0.084, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.066, -0.001)),
        material=drawer_metal,
        name="tray_back",
    )
    drawer.visual(
        Box((0.007, 0.010, 0.010)),
        origin=Origin(xyz=(-0.020, -0.014, 0.004)),
        material=worn_edge,
        name="pull_stem_0",
    )
    drawer.visual(
        Box((0.007, 0.010, 0.010)),
        origin=Origin(xyz=(0.020, -0.014, 0.004)),
        material=worn_edge,
        name="pull_stem_1",
    )
    drawer.visual(
        Cylinder(radius=0.004, length=0.046),
        origin=Origin(xyz=(0.0, -0.020, 0.008), rpy=(0.0, pi / 2.0, 0.0)),
        material=worn_edge,
        name="drawer_pull",
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        mesh_from_cadquery(_ring_y(0.034, 0.013, 0.010), "selector_ring", tolerance=0.0004),
        material=black_dial,
        name="selector_ring",
    )
    for idx in range(6):
        angle = idx * 2.0 * pi / 6.0
        selector_dial.visual(
            Cylinder(radius=0.0024, length=0.0016),
            origin=Origin(
                xyz=(0.024 * cos(angle), -0.0048, 0.024 * sin(angle)),
                rpy=(-pi / 2.0, 0.0, 0.0),
            ),
            material=ivory_mark,
            name=f"size_mark_{idx}",
        )
    selector_dial.visual(
        Box((0.006, 0.002, 0.016)),
        origin=Origin(xyz=(0.0, -0.0048, 0.025)),
        material=ivory_mark,
        name="pointer_mark",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.0048, length=0.040),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="shaft",
    )
    crank.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.027, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub",
    )
    crank.visual(
        Box((0.011, 0.014, 0.062)),
        origin=Origin(xyz=(0.032, 0.0, -0.030)),
        material=dark_steel,
        name="handle_arm",
    )
    crank.visual(
        Cylinder(radius=0.0048, length=0.022),
        origin=Origin(xyz=(0.035, 0.0, -0.057), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="tip_pin",
    )
    crank.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.044, 0.0, -0.057), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="tip_washer",
    )

    crank_knob = model.part("crank_knob")
    crank_knob.visual(
        Cylinder(radius=0.0105, length=0.034),
        origin=Origin(xyz=(0.017, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=wood_bakelite,
        name="grip",
    )
    crank_knob.visual(
        Sphere(radius=0.0105),
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        material=wood_bakelite,
        name="rounded_end",
    )

    model.articulation(
        "housing_to_drawer",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=drawer,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 - 0.004, DRAWER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.22, lower=0.0, upper=0.055),
    )
    model.articulation(
        "housing_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=selector_dial,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 - 0.006, PORT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0),
    )
    model.articulation(
        "housing_to_crank",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=crank,
        origin=Origin(xyz=(BODY_W / 2.0 + 0.007, CRANK_Y, CRANK_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=9.0),
    )
    model.articulation(
        "crank_to_crank_knob",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=crank_knob,
        origin=Origin(xyz=(0.046, 0.0, -0.057)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=16.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drawer = object_model.get_part("drawer")
    housing = object_model.get_part("housing")
    crank = object_model.get_part("crank")
    crank_knob = object_model.get_part("crank_knob")
    selector_dial = object_model.get_part("selector_dial")
    drawer_slide = object_model.get_articulation("housing_to_drawer")
    selector_spin = object_model.get_articulation("housing_to_selector_dial")
    crank_spin = object_model.get_articulation("housing_to_crank")
    knob_spin = object_model.get_articulation("crank_to_crank_knob")

    ctx.check(
        "primary mechanisms are articulated",
        selector_spin.articulation_type == ArticulationType.CONTINUOUS
        and crank_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and drawer_slide.articulation_type == ArticulationType.PRISMATIC,
    )

    ctx.expect_within(
        drawer,
        housing,
        axes="xz",
        inner_elem="tray_floor",
        outer_elem="body_shell",
        margin=0.004,
        name="closed drawer tray fits in the front slot",
    )
    ctx.expect_overlap(
        drawer,
        housing,
        axes="y",
        elem_a="tray_floor",
        elem_b="body_shell",
        min_overlap=0.030,
        name="closed drawer remains inserted in the housing",
    )
    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.055}):
        ctx.expect_overlap(
            drawer,
            housing,
            axes="y",
            elem_a="tray_floor",
            elem_b="body_shell",
            min_overlap=0.006,
            name="opened drawer keeps a retained slide engagement",
        )
        open_drawer_pos = ctx.part_world_position(drawer)
    ctx.check(
        "drawer slides out toward the front",
        rest_drawer_pos is not None
        and open_drawer_pos is not None
        and open_drawer_pos[1] < rest_drawer_pos[1] - 0.045,
        details=f"rest={rest_drawer_pos}, open={open_drawer_pos}",
    )

    ctx.expect_gap(
        crank_knob,
        crank,
        axis="x",
        positive_elem="grip",
        negative_elem="tip_washer",
        min_gap=0.0,
        max_gap=0.001,
        name="crank knob is seated against the handle washer",
    )
    ctx.expect_overlap(
        selector_dial,
        housing,
        axes="xz",
        elem_a="selector_ring",
        elem_b="body_shell",
        min_overlap=0.040,
        name="selector dial surrounds the pencil-entry throat",
    )
    ctx.expect_gap(
        housing,
        selector_dial,
        axis="y",
        positive_elem="body_shell",
        negative_elem="selector_ring",
        min_gap=0.0,
        max_gap=0.003,
        name="selector ring sits just proud of the front face",
    )

    return ctx.report()


object_model = build_object_model()
