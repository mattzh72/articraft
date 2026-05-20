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
)


def _turntable_glass_shape() -> cq.Workplane:
    """A shallow glass disk with a raised outer lip, centered on the spin axis."""
    disk = cq.Workplane("XY").circle(0.168).extrude(0.006).translate((0.0, 0.0, -0.003))
    raised_rim = (
        cq.Workplane("XY")
        .circle(0.168)
        .circle(0.154)
        .extrude(0.008)
        .translate((0.0, 0.0, 0.0))
    )
    return disk.union(raised_rim)


def _turntable_support_shape() -> cq.Workplane:
    """Low hub, three spokes, and an annular roller support under the glass tray."""
    hub = cq.Workplane("XY").circle(0.030).extrude(0.020).translate((0.0, 0.0, -0.023))
    ring = (
        cq.Workplane("XY")
        .circle(0.110)
        .circle(0.098)
        .extrude(0.006)
        .translate((0.0, 0.0, -0.024))
    )
    support = hub.union(ring)
    for angle in (0.0, 120.0, 240.0):
        spoke = (
            cq.Workplane("XY")
            .box(0.072, 0.010, 0.006)
            .translate((0.064, 0.0, -0.021))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        support = support.union(spoke)
    return support


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drop_down_microwave")

    stainless = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.70, 1.0))
    dark = model.material("charcoal_enamel", rgba=(0.025, 0.027, 0.030, 1.0))
    black = model.material("black_trim", rgba=(0.005, 0.006, 0.007, 1.0))
    glass = model.material("smoked_glass", rgba=(0.09, 0.13, 0.16, 0.42))
    tray_glass = model.material("clear_glass", rgba=(0.74, 0.92, 1.00, 0.34))
    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    white = model.material("white_print", rgba=(0.92, 0.92, 0.86, 1.0))

    body = model.part("housing")

    # Boxy commercial shell, open at the front cooking cavity and closed around it.
    body.visual(Box((0.560, 0.440, 0.030)), origin=Origin(xyz=(0.0, 0.0, 0.015)), material=stainless, name="bottom_panel")
    body.visual(Box((0.560, 0.440, 0.030)), origin=Origin(xyz=(0.0, 0.0, 0.315)), material=stainless, name="top_panel")
    body.visual(Box((0.030, 0.440, 0.330)), origin=Origin(xyz=(-0.265, 0.0, 0.165)), material=stainless, name="side_panel_0")
    body.visual(Box((0.030, 0.440, 0.330)), origin=Origin(xyz=(0.265, 0.0, 0.165)), material=stainless, name="side_panel_1")
    body.visual(Box((0.560, 0.030, 0.330)), origin=Origin(xyz=(0.0, 0.205, 0.165)), material=stainless, name="rear_panel")
    body.visual(Box((0.026, 0.440, 0.300)), origin=Origin(xyz=(0.118, 0.0, 0.165)), material=stainless, name="cavity_divider")

    # Dark interior liner panels visible through the window.
    body.visual(Box((0.338, 0.354, 0.004)), origin=Origin(xyz=(-0.075, -0.010, 0.032)), material=dark, name="cavity_floor_liner")
    body.visual(Box((0.340, 0.004, 0.236)), origin=Origin(xyz=(-0.075, 0.188, 0.156)), material=dark, name="cavity_back_liner")
    body.visual(Box((0.004, 0.354, 0.236)), origin=Origin(xyz=(-0.248, -0.010, 0.156)), material=dark, name="cavity_side_liner_0")
    body.visual(Box((0.004, 0.354, 0.236)), origin=Origin(xyz=(0.103, -0.010, 0.156)), material=dark, name="cavity_side_liner_1")

    # Front frame around the cooking cavity and the lower hinge landing.
    body.visual(Box((0.380, 0.030, 0.032)), origin=Origin(xyz=(-0.075, -0.235, 0.046)), material=black, name="lower_sill")
    body.visual(Box((0.380, 0.030, 0.034)), origin=Origin(xyz=(-0.075, -0.235, 0.292)), material=black, name="upper_sill")
    body.visual(Box((0.030, 0.030, 0.246)), origin=Origin(xyz=(-0.250, -0.235, 0.169)), material=black, name="front_jamb_0")
    body.visual(Box((0.030, 0.030, 0.246)), origin=Origin(xyz=(0.100, -0.235, 0.169)), material=black, name="front_jamb_1")
    body.visual(Box((0.072, 0.020, 0.016)), origin=Origin(xyz=(-0.178, -0.238, 0.055)), material=black, name="hinge_leaf_0")
    body.visual(Box((0.072, 0.020, 0.016)), origin=Origin(xyz=(0.028, -0.238, 0.055)), material=black, name="hinge_leaf_1")

    # Right-side control panel, with a printed dial scale and a single rotary control.
    body.visual(Box((0.130, 0.030, 0.300)), origin=Origin(xyz=(0.190, -0.235, 0.165)), material=dark, name="control_panel")
    body.visual(
        Cylinder(radius=0.049, length=0.0015),
        origin=Origin(xyz=(0.190, -0.25065, 0.214), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="dial_bezel",
    )
    for i, angle in enumerate((-60.0, -30.0, 0.0, 30.0, 60.0)):
        rad = math.radians(angle)
        x = 0.190 + 0.060 * math.sin(rad)
        z = 0.214 + 0.060 * math.cos(rad)
        body.visual(
            Box((0.004, 0.0012, 0.014 if angle == 0.0 else 0.010)),
            origin=Origin(xyz=(x, -0.2505, z), rpy=(0.0, 0.0, -rad)),
            material=white,
            name=f"timer_mark_{i}",
        )
    body.visual(Box((0.090, 0.0012, 0.012)), origin=Origin(xyz=(0.190, -0.2505, 0.116)), material=white, name="timer_label")

    door = model.part("door")
    door.visual(Box((0.382, 0.032, 0.038)), origin=Origin(xyz=(0.0, 0.0, 0.019)), material=black, name="lower_rail")
    door.visual(Box((0.382, 0.032, 0.038)), origin=Origin(xyz=(0.0, 0.0, 0.219)), material=black, name="upper_rail")
    door.visual(Box((0.040, 0.032, 0.236)), origin=Origin(xyz=(-0.171, 0.0, 0.118)), material=black, name="side_rail_0")
    door.visual(Box((0.040, 0.032, 0.236)), origin=Origin(xyz=(0.171, 0.0, 0.118)), material=black, name="side_rail_1")
    door.visual(Box((0.316, 0.009, 0.164)), origin=Origin(xyz=(0.0, -0.016, 0.128)), material=glass, name="window_glass")
    door.visual(Box((0.270, 0.018, 0.020)), origin=Origin(xyz=(0.0, -0.032, 0.238)), material=stainless, name="pull_handle")
    door.visual(Box((0.026, 0.030, 0.050)), origin=Origin(xyz=(-0.110, -0.018, 0.212)), material=stainless, name="handle_post_0")
    door.visual(Box((0.026, 0.030, 0.050)), origin=Origin(xyz=(0.110, -0.018, 0.212)), material=stainless, name="handle_post_1")
    for i, x in enumerate((-0.115, 0.115)):
        door.visual(
            Cylinder(radius=0.011, length=0.096),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"hinge_knuckle_{i}",
        )

    dial = model.part("timer_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.057,
            0.030,
            body_style="skirted",
            top_diameter=0.046,
            edge_radius=0.0012,
            grip=KnobGrip(style="fluted", count=24, depth=0.0014),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
            center=False,
        ),
        "timer_dial",
    )
    dial.visual(dial_mesh, origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=stainless, name="dial_cap")

    turntable = model.part("turntable")
    turntable.visual(mesh_from_cadquery(_turntable_glass_shape(), "glass_tray"), material=tray_glass, name="glass_tray")
    turntable.visual(mesh_from_cadquery(_turntable_support_shape(), "tray_support"), material=rubber, name="tray_support")

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.075, -0.266, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "housing_to_timer_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.190, -0.251, 0.214)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=5.0, lower=0.0, upper=5.5),
    )
    model.articulation(
        "housing_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=turntable,
        origin=Origin(xyz=(-0.075, -0.010, 0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=2.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    dial = object_model.get_part("timer_dial")
    turntable = object_model.get_part("turntable")
    door_hinge = object_model.get_articulation("housing_to_door")
    timer_axis = object_model.get_articulation("housing_to_timer_dial")
    tray_axis = object_model.get_articulation("housing_to_turntable")

    ctx.expect_within(turntable, housing, axes="xy", inner_elem="glass_tray", outer_elem="cavity_floor_liner", margin=0.004, name="glass tray fits inside cavity footprint")
    ctx.expect_overlap(door, housing, axes="x", elem_a="lower_rail", elem_b="lower_sill", min_overlap=0.30, name="drop-down door spans the lower sill")
    ctx.expect_overlap(dial, housing, axes="xz", elem_a="dial_cap", elem_b="dial_bezel", min_overlap=0.030, name="timer dial sits in its front bezel")

    ctx.check("door hinge is lower horizontal", tuple(round(v, 3) for v in door_hinge.axis) == (1.0, 0.0, 0.0), details=f"axis={door_hinge.axis}")
    ctx.check("timer dial rotates front to back", tuple(round(v, 3) for v in timer_axis.axis) == (0.0, 1.0, 0.0), details=f"axis={timer_axis.axis}")
    ctx.check("turntable rotates vertically", tuple(round(v, 3) for v in tray_axis.axis) == (0.0, 0.0, 1.0), details=f"axis={tray_axis.axis}")

    closed_top = ctx.part_element_world_aabb(door, elem="upper_rail")
    with ctx.pose({door_hinge: 1.20}):
        open_top = ctx.part_element_world_aabb(door, elem="upper_rail")
    ctx.check(
        "drop-down door opens forward and downward",
        closed_top is not None
        and open_top is not None
        and open_top[1][1] < closed_top[1][1] - 0.10
        and open_top[0][2] < closed_top[0][2] - 0.05,
        details=f"closed={closed_top}, open={open_top}",
    )

    return ctx.report()


object_model = build_object_model()
