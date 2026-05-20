from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _tapered_body_mesh() -> object:
    """A compact camcorder shell: larger at the hand/rear end, slimmer at the lens."""
    length = 0.180
    rear_width, rear_height = 0.078, 0.084
    front_width, front_height = 0.066, 0.070

    # A loft on the YZ workplane gives a real tapered body along +X.
    body = (
        cq.Workplane("YZ")
        .rect(rear_width, rear_height)
        .workplane(offset=length)
        .rect(front_width, front_height)
        .loft(ruled=True)
        .translate((-length / 2.0, 0.0, 0.0))
    )
    return body


def _rounded_panel_mesh(width: float, thickness: float, height: float) -> object:
    panel = cq.Workplane("XY").box(width, thickness, height)
    try:
        return panel.edges("|Y").fillet(0.004)
    except Exception:
        return panel


def _tube_z_mesh(outer_radius: float, inner_radius: float, length: float) -> object:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_home_video_camcorder")

    graphite = model.material("graphite", rgba=(0.055, 0.060, 0.064, 1.0))
    satin_black = model.material("satin_black", rgba=(0.004, 0.004, 0.005, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.012, 0.012, 0.013, 1.0))
    dark_glass = model.material("dark_blue_glass", rgba=(0.010, 0.028, 0.055, 0.82))
    display_glass = model.material("display_glass", rgba=(0.020, 0.055, 0.090, 0.92))
    hinge_metal = model.material("brushed_hinge_metal", rgba=(0.45, 0.47, 0.48, 1.0))
    label_white = model.material("white_markings", rgba=(0.86, 0.88, 0.84, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_tapered_body_mesh(), "tapered_body", tolerance=0.0008),
        material=graphite,
        name="tapered_body",
    )
    # Molded right-hand grip/strap pad on the side opposite the display door.
    body.visual(
        Box((0.095, 0.009, 0.050)),
        origin=Origin(xyz=(-0.015, -0.041, -0.002)),
        material=rubber,
        name="hand_grip",
    )
    # Rear/top viewfinder housing and soft eyecup.
    body.visual(
        Box((0.076, 0.036, 0.026)),
        origin=Origin(xyz=(-0.055, 0.0, 0.049)),
        material=graphite,
        name="viewfinder_body",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(-0.100, 0.0, 0.049), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="eyecup",
    )

    # Lens stack: a front objective barrel with a dark inner optical tube and glass.
    body.visual(
        Cylinder(radius=0.033, length=0.024),
        origin=Origin(xyz=(0.090, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="lens_mount",
    )
    body.visual(
        Cylinder(radius=0.0208, length=0.066),
        origin=Origin(xyz=(0.126, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="lens_core",
    )
    body.visual(
        Cylinder(radius=0.027, length=0.020),
        origin=Origin(xyz=(0.150, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="objective_barrel",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.002),
        origin=Origin(xyz=(0.161, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_glass,
        name="objective_glass",
    )
    # Exposed fixed pins/shafts that carry the moving door and diopter wheel.
    body.visual(
        Cylinder(radius=0.0029, length=0.082),
        origin=Origin(xyz=(-0.066, 0.0445, 0.000)),
        material=hinge_metal,
        name="display_hinge_pin",
    )
    for z_boss, boss_name in ((0.039, "upper_hinge_boss"), (-0.039, "lower_hinge_boss")):
        body.visual(
            Box((0.013, 0.009, 0.006)),
            origin=Origin(xyz=(-0.066, 0.0405, z_boss)),
            material=graphite,
            name=boss_name,
        )
    body.visual(
        Cylinder(radius=0.0044, length=0.025),
        origin=Origin(xyz=(-0.095, -0.023, 0.056), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="diopter_shaft",
    )

    # Continuous focus ring around the lens barrel; a tiny hidden interference keeps it retained.
    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.060,
                0.024,
                body_style="cylindrical",
                edge_radius=0.0009,
                grip=KnobGrip(style="ribbed", count=36, depth=0.0010, width=0.0012),
                bore=KnobBore(style="round", diameter=0.041),
            ),
            "focus_ring_grip",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="focus_grip",
    )
    focus_ring.visual(
        Box((0.0020, 0.0030, 0.010)),
        origin=Origin(xyz=(0.0126, -0.0305, 0.000)),
        material=label_white,
        name="focus_index",
    )

    # Flip-out display door: child frame sits on the vertical hinge axis.
    display_door = model.part("display_door")
    display_door.visual(
        mesh_from_cadquery(_rounded_panel_mesh(0.116, 0.007, 0.067), "display_panel"),
        origin=Origin(xyz=(0.066, 0.0046, 0.000)),
        material=graphite,
        name="panel_shell",
    )
    display_door.visual(
        Box((0.090, 0.0012, 0.050)),
        origin=Origin(xyz=(0.066, 0.0006, 0.000)),
        material=display_glass,
        name="lcd_screen",
    )
    display_door.visual(
        mesh_from_cadquery(_tube_z_mesh(0.0045, 0.0028, 0.066), "display_hinge_sleeve"),
        material=graphite,
        name="hinge_sleeve",
    )
    display_door.visual(
        Box((0.012, 0.004, 0.060)),
        origin=Origin(xyz=(0.006, 0.0045, 0.000)),
        material=graphite,
        name="hinge_leaf",
    )

    # Diopter wheel at the rear viewfinder on a small transverse shaft.
    diopter_wheel = model.part("diopter_wheel")
    diopter_wheel.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.023,
                0.008,
                body_style="cylindrical",
                grip=KnobGrip(style="ribbed", count=18, depth=0.0007, width=0.0010),
                bore=KnobBore(style="round", diameter=0.0085),
            ),
            "diopter_wheel_grip",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="diopter_grip",
    )

    model.articulation(
        "display_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=display_door,
        origin=Origin(xyz=(-0.066, 0.0445, 0.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=3.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "focus_rotation",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(0.118, 0.0, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )
    model.articulation(
        "diopter_rotation",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=diopter_wheel,
        origin=Origin(xyz=(-0.095, -0.027, 0.056)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    display_door = object_model.get_part("display_door")
    focus_ring = object_model.get_part("focus_ring")
    diopter_wheel = object_model.get_part("diopter_wheel")
    display_hinge = object_model.get_articulation("display_hinge")
    focus_rotation = object_model.get_articulation("focus_rotation")
    diopter_rotation = object_model.get_articulation("diopter_rotation")

    ctx.check(
        "display door has a vertical limited hinge",
        display_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(display_hinge.axis) == (0.0, 0.0, 1.0)
        and display_hinge.motion_limits is not None
        and display_hinge.motion_limits.upper >= 1.5,
        details=f"type={display_hinge.articulation_type}, axis={display_hinge.axis}, limits={display_hinge.motion_limits}",
    )
    ctx.check(
        "focus ring rotates continuously on lens axis",
        focus_rotation.articulation_type == ArticulationType.CONTINUOUS
        and tuple(focus_rotation.axis) == (1.0, 0.0, 0.0),
        details=f"type={focus_rotation.articulation_type}, axis={focus_rotation.axis}",
    )
    ctx.check(
        "diopter wheel rotates on transverse shaft",
        diopter_rotation.articulation_type == ArticulationType.CONTINUOUS
        and tuple(diopter_rotation.axis) == (0.0, 1.0, 0.0),
        details=f"type={diopter_rotation.articulation_type}, axis={diopter_rotation.axis}",
    )

    ctx.allow_overlap(
        body,
        display_door,
        elem_a="display_hinge_pin",
        elem_b="hinge_sleeve",
        reason="The moving hinge sleeve is intentionally captured on the fixed metal hinge pin with a tiny modeled interference fit.",
    )
    ctx.allow_overlap(
        body,
        focus_ring,
        elem_a="lens_core",
        elem_b="focus_grip",
        reason="The focus grip is modeled as a captured rotating sleeve around the lens core with slight hidden bearing interference.",
    )
    ctx.allow_overlap(
        body,
        diopter_wheel,
        elem_a="diopter_shaft",
        elem_b="diopter_grip",
        reason="The diopter wheel is intentionally retained on its small transverse shaft with a tiny hidden interference fit.",
    )

    ctx.expect_gap(
        display_door,
        body,
        axis="y",
        min_gap=0.002,
        max_gap=0.014,
        positive_elem="panel_shell",
        negative_elem="tapered_body",
        name="closed display panel sits just outside the left body side",
    )
    ctx.expect_overlap(
        body,
        focus_ring,
        axes="x",
        min_overlap=0.018,
        elem_a="lens_core",
        elem_b="focus_grip",
        name="focus ring surrounds the lens barrel along its length",
    )
    ctx.expect_within(
        body,
        focus_ring,
        axes="yz",
        inner_elem="lens_core",
        outer_elem="focus_grip",
        margin=0.004,
        name="lens core is centered inside focus-ring footprint",
    )
    ctx.expect_overlap(
        body,
        diopter_wheel,
        axes="y",
        min_overlap=0.006,
        elem_a="diopter_shaft",
        elem_b="diopter_grip",
        name="diopter wheel is carried on the transverse shaft",
    )
    ctx.expect_within(
        body,
        diopter_wheel,
        axes="xz",
        inner_elem="diopter_shaft",
        outer_elem="diopter_grip",
        margin=0.003,
        name="diopter shaft is centered in wheel footprint",
    )
    ctx.expect_overlap(
        body,
        display_door,
        axes="z",
        min_overlap=0.060,
        elem_a="display_hinge_pin",
        elem_b="hinge_sleeve",
        name="display hinge sleeve is retained on the vertical pin",
    )

    closed_panel = ctx.part_element_world_aabb(display_door, elem="panel_shell")
    with ctx.pose({display_hinge: 1.55}):
        opened_panel = ctx.part_element_world_aabb(display_door, elem="panel_shell")
    ctx.check(
        "display door swings outward from side",
        closed_panel is not None
        and opened_panel is not None
        and opened_panel[1][1] > closed_panel[1][1] + 0.060,
        details=f"closed={closed_panel}, opened={opened_panel}",
    )

    return ctx.report()


object_model = build_object_model()
