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


def _handwheel_shape() -> cq.Workplane:
    """A cast utility handwheel: thick rim, four spokes, central hub and grip."""

    rim_radius = 0.20
    rim_inner_radius = 0.158
    rim_thickness = 0.035

    rim = (
        cq.Workplane("XY")
        .circle(rim_radius)
        .circle(rim_inner_radius)
        .extrude(rim_thickness)
        .translate((0.0, 0.0, -rim_thickness / 2.0))
    )
    hub = (
        cq.Workplane("XY")
        .circle(0.055)
        .extrude(0.070)
        .translate((0.0, 0.0, -0.035))
    )
    spoke_a = (
        cq.Workplane("XY")
        .rect(0.310, 0.026)
        .extrude(0.028)
        .translate((0.0, 0.0, -0.014))
    )
    spoke_b = (
        cq.Workplane("XY")
        .rect(0.026, 0.310)
        .extrude(0.028)
        .translate((0.0, 0.0, -0.014))
    )
    grip_boss = (
        cq.Workplane("XY")
        .center(0.0, 0.178)
        .circle(0.030)
        .extrude(0.040)
        .translate((0.0, 0.0, -0.005))
    )
    spinner = (
        cq.Workplane("XY")
        .center(0.0, 0.178)
        .circle(0.022)
        .extrude(0.105)
        .translate((0.0, 0.0, 0.025))
    )
    return rim.union(hub).union(spoke_a).union(spoke_b).union(grip_boss).union(spinner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="framed_sluice_gate")

    galvanized = Material("galvanized_steel", rgba=(0.45, 0.48, 0.48, 1.0))
    dark_steel = Material("dark_blued_steel", rgba=(0.08, 0.10, 0.11, 1.0))
    gate_blue = Material("painted_gate_leaf", rgba=(0.05, 0.20, 0.34, 1.0))
    safety_yellow = Material("safety_yellow", rgba=(0.95, 0.68, 0.08, 1.0))
    black = Material("black_rubber", rgba=(0.01, 0.012, 0.012, 1.0))
    oil_cover = Material("gearbox_cover_gray", rgba=(0.30, 0.33, 0.34, 1.0))

    frame = model.part("frame")
    # Upright C-channel guide assemblies.  The front/back lips leave a real
    # clearance gap around the sliding leaf instead of solidly intersecting it.
    for sx in (-1.0, 1.0):
        frame.visual(
            Box((0.085, 0.180, 2.20)),
            origin=Origin(xyz=(sx * 0.640, 0.0, 1.15)),
            material=galvanized,
            name=f"side_web_{'neg' if sx < 0 else 'pos'}",
        )
        frame.visual(
            Box((0.105, 0.035, 2.00)),
            origin=Origin(xyz=(sx * 0.555, -0.065, 1.10)),
            material=galvanized,
            name="front_guide_lip_neg" if sx < 0 else "front_guide_lip_pos",
        )
        frame.visual(
            Box((0.105, 0.035, 2.00)),
            origin=Origin(xyz=(sx * 0.555, 0.065, 1.10)),
            material=galvanized,
            name="rear_guide_lip_neg" if sx < 0 else "rear_guide_lip_pos",
        )
        frame.visual(
            Box((0.045, 0.008, 1.75)),
            origin=Origin(xyz=(sx * 0.510, -0.050, 0.98)),
            material=black,
            name=f"front_wear_strip_{'neg' if sx < 0 else 'pos'}",
        )
        frame.visual(
            Box((0.045, 0.008, 1.75)),
            origin=Origin(xyz=(sx * 0.510, 0.050, 0.98)),
            material=black,
            name=f"rear_wear_strip_{'neg' if sx < 0 else 'pos'}",
        )

    frame.visual(
        Box((1.52, 0.240, 0.160)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=galvanized,
        name="bottom_sill",
    )
    frame.visual(
        Box((1.62, 0.240, 0.200)),
        origin=Origin(xyz=(0.0, 0.0, 2.25)),
        material=galvanized,
        name="top_crossbeam",
    )
    frame.visual(
        Box((1.14, 0.030, 0.070)),
        origin=Origin(xyz=(0.0, 0.085, 1.88)),
        material=dark_steel,
        name="rear_lintel_shadow",
    )

    # Central handwheel support and bearing on the crossbeam.
    frame.visual(
        Box((0.230, 0.110, 0.360)),
        origin=Origin(xyz=(0.0, -0.075, 2.45)),
        material=galvanized,
        name="wheel_pedestal",
    )
    frame.visual(
        Box((0.260, 0.050, 0.135)),
        origin=Origin(xyz=(0.0, -0.145, 2.48)),
        material=galvanized,
        name="bearing_block",
    )
    frame.visual(
        Cylinder(radius=0.060, length=0.080),
        origin=Origin(xyz=(0.0, -0.190, 2.48), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bearing_cap",
    )

    # Gearbox housing fixed to the top beam, with a separate hinged cover.
    frame.visual(
        Box((0.360, 0.100, 0.260)),
        origin=Origin(xyz=(0.350, -0.170, 2.37)),
        material=dark_steel,
        name="gearbox_body",
    )
    frame.visual(
        Box((0.035, 0.030, 0.275)),
        origin=Origin(xyz=(0.170, -0.215, 2.37)),
        material=galvanized,
        name="cover_hinge_mount",
    )

    # Bolt heads on the beam and channel flanges reinforce the utility scale.
    bolt_positions = [
        (-0.58, -0.124, 2.28),
        (-0.38, -0.124, 2.28),
        (0.58, -0.124, 2.28),
        (-0.63, -0.088, 1.70),
        (-0.63, -0.088, 1.15),
        (-0.63, -0.088, 0.60),
        (0.63, -0.088, 1.70),
        (0.63, -0.088, 1.15),
        (0.63, -0.088, 0.60),
    ]
    for i, xyz in enumerate(bolt_positions):
        frame.visual(
            Cylinder(radius=0.023, length=0.014),
            origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"frame_bolt_{i}",
        )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((1.080, 0.070, 1.340)),
        origin=Origin(),
        material=gate_blue,
        name="leaf_plate",
    )
    gate_leaf.visual(
        Box((0.900, 0.095, 0.070)),
        origin=Origin(xyz=(0.0, -0.005, 0.420)),
        material=dark_steel,
        name="upper_stiffener",
    )
    gate_leaf.visual(
        Box((0.900, 0.095, 0.070)),
        origin=Origin(xyz=(0.0, -0.005, -0.420)),
        material=dark_steel,
        name="lower_stiffener",
    )
    for sx in (-1.0, 1.0):
        gate_leaf.visual(
            Box((0.055, 0.090, 1.340)),
            origin=Origin(xyz=(sx * 0.515, 0.0, 0.0)),
            material=dark_steel,
            name=f"edge_shoe_{'neg' if sx < 0 else 'pos'}",
        )
    gate_leaf.visual(
        Box((0.220, 0.130, 0.050)),
        origin=Origin(xyz=(0.0, -0.095, 0.680)),
        material=dark_steel,
        name="stem_yoke",
    )
    gate_leaf.visual(
        Cylinder(radius=0.018, length=0.520),
        origin=Origin(xyz=(0.0, -0.155, 0.930)),
        material=galvanized,
        name="lift_stem",
    )
    for i, (x, z) in enumerate(
        [
            (-0.38, 0.47),
            (0.0, 0.47),
            (0.38, 0.47),
            (-0.38, -0.47),
            (0.0, -0.47),
            (0.38, -0.47),
        ]
    ):
        gate_leaf.visual(
            Cylinder(radius=0.025, length=0.014),
            origin=Origin(xyz=(x, -0.041, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"leaf_bolt_{i}",
        )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(_handwheel_shape(), "cast_handwheel", tolerance=0.002),
        origin=Origin(xyz=(0.0, -0.180, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety_yellow,
        name="wheel_rim",
    )
    handwheel.visual(
        Cylinder(radius=0.026, length=0.180),
        origin=Origin(xyz=(0.0, -0.080, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="wheel_axle",
    )

    gearbox_cover = model.part("gearbox_cover")
    gearbox_cover.visual(
        Box((0.320, 0.026, 0.240)),
        origin=Origin(xyz=(0.179, 0.0, 0.0)),
        material=oil_cover,
        name="cover_panel",
    )
    gearbox_cover.visual(
        Box((0.032, 0.012, 0.205)),
        origin=Origin(xyz=(0.035, -0.007, 0.0)),
        material=galvanized,
        name="hinge_leaf",
    )
    gearbox_cover.visual(
        Cylinder(radius=0.019, length=0.270),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=galvanized,
        name="hinge_barrel",
    )
    gearbox_cover.visual(
        Box((0.180, 0.010, 0.030)),
        origin=Origin(xyz=(0.160, -0.018, 0.040)),
        material=galvanized,
        name="cover_latch",
    )

    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.83)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.20, lower=0.0, upper=0.65),
    )
    model.articulation(
        "wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=handwheel,
        origin=Origin(xyz=(0.0, -0.210, 2.48)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=5.0),
    )
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=gearbox_cover,
        origin=Origin(xyz=(0.170, -0.235, 2.37)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    gate_leaf = object_model.get_part("gate_leaf")
    handwheel = object_model.get_part("handwheel")
    gearbox_cover = object_model.get_part("gearbox_cover")
    gate_slide = object_model.get_articulation("gate_slide")
    cover_hinge = object_model.get_articulation("cover_hinge")

    ctx.allow_overlap(
        frame,
        handwheel,
        elem_a="bearing_cap",
        elem_b="wheel_axle",
        reason="The rotating handwheel shaft is intentionally captured inside the fixed bearing cap.",
    )
    ctx.expect_overlap(
        frame,
        handwheel,
        axes="y",
        elem_a="bearing_cap",
        elem_b="wheel_axle",
        min_overlap=0.015,
        name="handwheel axle remains seated in bearing",
    )
    ctx.allow_overlap(
        frame,
        gearbox_cover,
        elem_a="cover_hinge_mount",
        elem_b="hinge_barrel",
        reason="The cover hinge barrel wraps around the fixed hinge mount/pin at the side of the gearbox.",
    )
    ctx.expect_overlap(
        frame,
        gearbox_cover,
        axes="z",
        elem_a="cover_hinge_mount",
        elem_b="hinge_barrel",
        min_overlap=0.20,
        name="gearbox hinge has full vertical engagement",
    )

    ctx.expect_within(
        gate_leaf,
        frame,
        axes="x",
        inner_elem="leaf_plate",
        outer_elem="top_crossbeam",
        margin=0.01,
        name="gate leaf fits between side channels",
    )
    ctx.expect_overlap(
        gate_leaf,
        frame,
        axes="z",
        elem_a="leaf_plate",
        elem_b="front_guide_lip_pos",
        min_overlap=1.20,
        name="closed gate is guided along most of its height",
    )
    closed_pos = ctx.part_world_position(gate_leaf)
    with ctx.pose({gate_slide: 0.65}):
        ctx.expect_overlap(
            gate_leaf,
            frame,
            axes="z",
            elem_a="leaf_plate",
            elem_b="front_guide_lip_pos",
            min_overlap=0.65,
            name="raised gate remains captured in guide channels",
        )
        raised_pos = ctx.part_world_position(gate_leaf)
    ctx.check(
        "gate leaf slides upward",
        closed_pos is not None and raised_pos is not None and raised_pos[2] > closed_pos[2] + 0.60,
        details=f"closed={closed_pos}, raised={raised_pos}",
    )

    closed_cover = ctx.part_element_world_aabb(gearbox_cover, elem="cover_panel")
    with ctx.pose({cover_hinge: 1.0}):
        opened_cover = ctx.part_element_world_aabb(gearbox_cover, elem="cover_panel")
    closed_y = None if closed_cover is None else (closed_cover[0][1] + closed_cover[1][1]) / 2.0
    opened_y = None if opened_cover is None else (opened_cover[0][1] + opened_cover[1][1]) / 2.0
    ctx.check(
        "gearbox cover swings outward",
        closed_y is not None and opened_y is not None and opened_y < closed_y - 0.10,
        details=f"closed_y={closed_y}, opened_y={opened_y}",
    )

    return ctx.report()


object_model = build_object_model()
