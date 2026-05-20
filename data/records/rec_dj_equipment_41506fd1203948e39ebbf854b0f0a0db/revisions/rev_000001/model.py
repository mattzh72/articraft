from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
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


def _hex_housing_geometry() -> cq.Workplane:
    """Low, wide six-sided DJ controller shell with integral bearing bosses."""
    height = 0.055
    points = [
        (-0.340, -0.220),
        (0.340, -0.220),
        (0.405, 0.000),
        (0.340, 0.220),
        (-0.340, 0.220),
        (-0.405, 0.000),
    ]
    shell = cq.Workplane("XY").polyline(points).close().extrude(height)
    shell = shell.edges("|Z").fillet(0.018).edges(">Z").fillet(0.004)

    platter_bearing = cq.Workplane("XY").circle(0.055).extrude(0.009).translate((0.0, 0.0, height))
    platter_recess_ring = (
        cq.Workplane("XY")
        .circle(0.205)
        .circle(0.182)
        .extrude(0.004)
        .translate((0.0, 0.0, height))
    )

    encoder_sockets = cq.Workplane("XY")
    for x in (-0.265, 0.265):
        socket = cq.Workplane("XY").circle(0.050).extrude(0.009).translate((x, 0.005, height))
        socket_ring = (
            cq.Workplane("XY")
            .circle(0.070)
            .circle(0.056)
            .extrude(0.003)
            .translate((x, 0.005, height))
        )
        encoder_sockets = encoder_sockets.union(socket).union(socket_ring)

    return shell.union(platter_bearing).union(platter_recess_ring).union(encoder_sockets)


def _platter_geometry() -> cq.Workplane:
    """Single connected rotating platter body with a raised finger rim."""
    base = cq.Workplane("XY").circle(0.172).extrude(0.014)
    rim = cq.Workplane("XY").circle(0.176).circle(0.150).extrude(0.004).translate((0.0, 0.0, 0.014))
    hub = cq.Workplane("XY").circle(0.030).extrude(0.003).translate((0.0, 0.0, 0.014))
    return base.union(rim).union(hub).edges(">Z").fillet(0.0015)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_platter_performance_controller")

    housing_mat = Material("matte_charcoal", rgba=(0.035, 0.038, 0.043, 1.0))
    platter_mat = Material("brushed_dark_aluminum", rgba=(0.24, 0.25, 0.26, 1.0))
    touch_mat = Material("gloss_black_touch_glass", rgba=(0.002, 0.002, 0.003, 1.0))
    knob_mat = Material("black_rubber_knurl", rgba=(0.008, 0.008, 0.009, 1.0))
    accent_mat = Material("cool_white_position_marks", rgba=(0.85, 0.92, 0.96, 1.0))
    cyan_mat = Material("cyan_status_light", rgba=(0.0, 0.65, 0.95, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_hex_housing_geometry(), "hex_housing"),
        material=housing_mat,
        name="hex_housing",
    )
    # Thin illuminated strips are flush details on the fixed top deck, not buttons.
    for i, y in enumerate((-0.128, 0.128)):
        housing.visual(
            Cylinder(radius=0.006, length=0.070),
            origin=Origin(xyz=(0.0, y, 0.060), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=cyan_mat,
            name=f"status_light_{i}",
        )

    platter = model.part("platter")
    platter.visual(
        mesh_from_cadquery(_platter_geometry(), "platter_body"),
        material=platter_mat,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=0.143, length=0.002),
        # Slightly seated into the platter mesh so the overlay is a supported skin.
        origin=Origin(xyz=(0.0, 0.0, 0.0150)),
        material=touch_mat,
        name="touch_surface",
    )
    platter.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0178)),
        material=platter_mat,
        name="center_cap",
    )
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        radius = 0.161
        platter.visual(
            Cylinder(radius=0.0035, length=0.020),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.020),
                rpy=(math.pi / 2.0, 0.0, angle),
            ),
            material=accent_mat,
            name=f"rim_mark_{i}",
        )

    encoder_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.032,
            body_style="cylindrical",
            edge_radius=0.0012,
            grip=KnobGrip(style="knurled", count=40, depth=0.0010, helix_angle_deg=18.0),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "encoder_knob",
    )
    encoder_positions = [(-0.265, 0.005), (0.265, 0.005)]
    for i, (x, y) in enumerate(encoder_positions):
        knob = model.part(f"encoder_{i}")
        knob.visual(encoder_mesh, material=knob_mat, name="knurled_cap")
        knob.visual(
            Cylinder(radius=0.0045, length=0.0015),
            origin=Origin(xyz=(0.015, 0.0, 0.0315)),
            material=accent_mat,
            name="position_dot",
        )
        model.articulation(
            f"housing_to_encoder_{i}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=knob,
            origin=Origin(xyz=(x, y, 0.064)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.35, velocity=18.0),
        )

    model.articulation(
        "housing_to_platter",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    platter = object_model.get_part("platter")
    platter_joint = object_model.get_articulation("housing_to_platter")

    ctx.check(
        "platter joint is continuous vertical",
        platter_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(platter_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={platter_joint.articulation_type}, axis={platter_joint.axis}",
    )
    ctx.expect_gap(
        platter,
        housing,
        axis="z",
        positive_elem="platter_body",
        negative_elem="hex_housing",
        max_gap=0.001,
        max_penetration=0.0,
        name="platter sits on the central bearing boss",
    )
    ctx.expect_overlap(
        platter,
        housing,
        axes="xy",
        elem_a="platter_body",
        elem_b="hex_housing",
        min_overlap=0.28,
        name="platter is centered over the top deck",
    )

    for i in (0, 1):
        encoder = object_model.get_part(f"encoder_{i}")
        encoder_joint = object_model.get_articulation(f"housing_to_encoder_{i}")
        ctx.check(
            f"encoder {i} joint is continuous vertical",
            encoder_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(encoder_joint.axis) == (0.0, 0.0, 1.0),
            details=f"type={encoder_joint.articulation_type}, axis={encoder_joint.axis}",
        )
        ctx.expect_gap(
            encoder,
            housing,
            axis="z",
            positive_elem="knurled_cap",
            negative_elem="hex_housing",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"encoder {i} cap sits on its socket",
        )

    return ctx.report()


object_model = build_object_model()
