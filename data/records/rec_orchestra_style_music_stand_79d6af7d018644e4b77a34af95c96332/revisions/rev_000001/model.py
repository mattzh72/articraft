from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _hollow_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A simple open-ended metal tube with a real bore for the sliding mast."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _score_tray_body() -> cq.Workplane:
    """Connected sheet-metal score tray: broad back panel plus upturned lower lip."""
    body = _box_at((0.580, 0.012, 0.350), (0.0, -0.110, 0.135))
    body = body.union(_box_at((0.540, 0.065, 0.016), (0.0, -0.140, -0.035)))
    body = body.union(_box_at((0.540, 0.014, 0.048), (0.0, -0.173, -0.012)))
    body = body.union(_box_at((0.590, 0.018, 0.016), (0.0, -0.109, 0.316)))
    body = body.union(_box_at((0.014, 0.020, 0.345), (-0.295, -0.107, 0.137)))
    body = body.union(_box_at((0.014, 0.020, 0.345), (0.295, -0.107, 0.137)))
    body = body.union(_box_at((0.082, 0.105, 0.050), (0.0, -0.072, -0.014)))
    body = body.union(_box_at((0.084, 0.022, 0.008), (0.0, -0.020, -0.010)))
    # The desk is normally leaned slightly back before any user hinge adjustment.
    return body.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -12.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weighted_orchestra_music_stand")

    black = model.material("satin_black_metal", rgba=(0.02, 0.022, 0.025, 1.0))
    graphite = model.material("dark_graphite", rgba=(0.18, 0.185, 0.19, 1.0))
    worn_edge = model.material("rubber_black", rgba=(0.006, 0.006, 0.006, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.180, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=black,
        name="weighted_disk",
    )
    base.visual(
        Cylinder(radius=0.174, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=worn_edge,
        name="rubber_foot_ring",
    )
    base.visual(
        Cylinder(radius=0.048, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=black,
        name="base_hub",
    )
    base.visual(
        mesh_from_cadquery(_hollow_cylinder(0.022, 0.0145, 0.630), "lower_mast_tube"),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=black,
        name="lower_mast",
    )
    base.visual(
        mesh_from_cadquery(_hollow_cylinder(0.036, 0.0115, 0.100), "collar_sleeve_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        material=black,
        name="collar_sleeve",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.040),
        origin=Origin(xyz=(0.050, 0.0, 0.745), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="knob_boss",
    )

    upper_column = model.part("upper_column")
    upper_column.visual(
        Cylinder(radius=0.0115, length=1.100),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=graphite,
        name="inner_tube",
    )
    upper_column.visual(
        Cylinder(radius=0.017, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.785)),
        material=black,
        name="top_plug",
    )
    upper_column.visual(
        Box((0.052, 0.035, 0.052)),
        origin=Origin(xyz=(0.0, -0.012, 0.820)),
        material=black,
        name="bracket_neck",
    )
    upper_column.visual(
        Box((0.160, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, -0.030, 0.815)),
        material=black,
        name="hinge_crosshead",
    )
    for x, visual_name in ((-0.060, "hinge_ear_0"), (0.060, "hinge_ear_1")):
        upper_column.visual(
            Box((0.024, 0.040, 0.080)),
            origin=Origin(xyz=(x, -0.055, 0.860)),
            material=black,
            name=visual_name,
        )
    upper_column.visual(
        Cylinder(radius=0.006, length=0.150),
        origin=Origin(xyz=(0.0, -0.055, 0.860), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="hinge_pin",
    )

    model.articulation(
        "base_to_upper_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_column,
        origin=Origin(xyz=(0.0, 0.0, 0.790)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.18, lower=0.0, upper=0.260),
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_score_tray_body(), "score_tray_body"),
        material=graphite,
        name="tray_body",
    )
    tray.visual(
        Cylinder(radius=0.014, length=0.088),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="hinge_barrel",
    )

    model.articulation(
        "upper_column_to_tray",
        ArticulationType.REVOLUTE,
        parent=upper_column,
        child=tray,
        origin=Origin(xyz=(0.0, -0.055, 0.860)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.45, upper=0.55),
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.006, length=0.036),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.048,
                0.024,
                body_style="lobed",
                base_diameter=0.034,
                top_diameter=0.044,
                crown_radius=0.0015,
                edge_radius=0.001,
            ),
            "locking_knob_cap",
        ),
        origin=Origin(xyz=(0.048, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="grip_cap",
    )

    model.articulation(
        "base_to_knob",
        ArticulationType.REVOLUTE,
        parent=base,
        child=knob,
        origin=Origin(xyz=(0.070, 0.0, 0.745)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-2.0 * math.pi, upper=2.0 * math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper_column = object_model.get_part("upper_column")
    tray = object_model.get_part("tray")
    knob = object_model.get_part("knob")
    slide = object_model.get_articulation("base_to_upper_column")
    hinge = object_model.get_articulation("upper_column_to_tray")

    ctx.allow_overlap(
        base,
        upper_column,
        elem_a="collar_sleeve",
        elem_b="inner_tube",
        reason="The upper mast is intentionally modeled as a snug telescoping member captured inside the locking collar sleeve.",
    )
    ctx.allow_overlap(
        upper_column,
        tray,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The tray hinge barrel intentionally captures the bracket pin on the horizontal tilt axis.",
    )
    ctx.expect_within(
        upper_column,
        tray,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="hinge_barrel",
        name="hinge pin is centered inside tray barrel",
    )
    ctx.expect_overlap(
        upper_column,
        tray,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.080,
        name="hinge pin spans the tray barrel",
    )

    ctx.expect_within(
        upper_column,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="collar_sleeve",
        name="upper mast is centered in hollow collar",
    )
    ctx.expect_overlap(
        upper_column,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="collar_sleeve",
        min_overlap=0.085,
        name="collapsed mast remains inserted through collar",
    )

    rest_position = ctx.part_world_position(upper_column)
    with ctx.pose({slide: 0.260}):
        ctx.expect_within(
            upper_column,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="collar_sleeve",
            name="extended mast stays centered in collar",
        )
        ctx.expect_overlap(
            upper_column,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="collar_sleeve",
            min_overlap=0.075,
            name="extended mast retains insertion in collar",
        )
        extended_position = ctx.part_world_position(upper_column)

    ctx.check(
        "upper column translates upward",
        rest_position is not None and extended_position is not None and extended_position[2] > rest_position[2] + 0.240,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    closed_aabb = ctx.part_world_aabb(tray)
    with ctx.pose({hinge: 0.45}):
        tilted_aabb = ctx.part_world_aabb(tray)
    if closed_aabb is not None and tilted_aabb is not None:
        closed_center_y = (closed_aabb[0][1] + closed_aabb[1][1]) * 0.5
        tilted_center_y = (tilted_aabb[0][1] + tilted_aabb[1][1]) * 0.5
        tray_moved = abs(tilted_center_y - closed_center_y) > 0.025
    else:
        tray_moved = False
    ctx.check("score tray rotates about horizontal hinge", tray_moved)

    ctx.expect_gap(
        knob,
        base,
        axis="x",
        positive_elem="shaft",
        negative_elem="knob_boss",
        max_gap=0.002,
        max_penetration=0.0,
        name="locking knob shaft seats at collar boss",
    )

    return ctx.report()


object_model = build_object_model()
