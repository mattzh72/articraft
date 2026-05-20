from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_box_fan")

    shell = model.material("warm_white_plastic", color=(0.86, 0.87, 0.84, 1.0))
    pale = model.material("pale_gray_plastic", color=(0.70, 0.72, 0.71, 1.0))
    dark = model.material("dark_grille_plastic", color=(0.035, 0.038, 0.040, 1.0))
    black = model.material("satin_black", color=(0.008, 0.008, 0.009, 1.0))
    accent = model.material("indicator_white", color=(0.96, 0.96, 0.90, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.190, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=pale,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.145, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=shell,
        name="raised_plinth",
    )
    base.visual(
        mesh_from_geometry(TorusGeometry(radius=0.107, tube=0.0075), "bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=black,
        name="bearing_ring",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.023),
        origin=Origin(xyz=(0.0, 0.0, 0.0835)),
        material=dark,
        name="pivot_post",
    )

    column = model.part("column")
    body_shape = (
        cq.Workplane("XY")
        .box(0.200, 0.155, 1.050)
        .edges("|Z")
        .fillet(0.024)
    )
    column.visual(
        mesh_from_cadquery(body_shape, "rounded_column"),
        origin=Origin(xyz=(0.0, 0.0, 0.605)),
        material=shell,
        name="rounded_column",
    )

    column.visual(
        Cylinder(radius=0.084, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=shell,
        name="pivot_flange",
    )
    column.visual(
        Cylinder(radius=0.058, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=shell,
        name="pivot_collar",
    )
    column.visual(
        Box((0.150, 0.118, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.077)),
        material=shell,
        name="neck_bridge",
    )

    column.visual(
        Box((0.145, 0.004, 0.880)),
        origin=Origin(xyz=(0.0, -0.0788, 0.615)),
        material=dark,
        name="dark_grille_backing",
    )
    front_slots = SlotPatternPanelGeometry(
        (0.158, 0.930),
        0.004,
        slot_size=(0.835, 0.0085),
        pitch=(0.026, 1.000),
        frame=0.012,
        corner_radius=0.018,
        slot_angle_deg=89.0,
    )
    column.visual(
        mesh_from_geometry(front_slots, "front_grille"),
        origin=Origin(xyz=(0.0, -0.0795, 0.615), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pale,
        name="front_grille",
    )

    top_panel = (
        cq.Workplane("XY")
        .box(0.122, 0.082, 0.006)
        .edges("|Z")
        .fillet(0.020)
    )
    column.visual(
        mesh_from_cadquery(top_panel, "top_control_panel"),
        origin=Origin(xyz=(0.0, 0.0, 1.133)),
        material=dark,
        name="top_panel",
    )
    column.visual(
        Cylinder(radius=0.033, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 1.146)),
        material=dark,
        name="speed_boss",
    )

    knob = model.part("knob")
    speed_knob = KnobGeometry(
        0.052,
        0.026,
        body_style="domed",
        edge_radius=0.001,
        grip=KnobGrip(style="fluted", count=22, depth=0.0010),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(speed_knob, "speed_knob"),
        material=black,
        name="knob_cap",
    )
    knob.visual(
        Box((0.005, 0.026, 0.0012)),
        origin=Origin(xyz=(0.0, 0.010, 0.0260)),
        material=accent,
        name="pointer_mark",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.REVOLUTE,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.75, upper=0.75, effort=18.0, velocity=0.8),
    )
    model.articulation(
        "column_to_knob",
        ArticulationType.REVOLUTE,
        parent=column,
        child=knob,
        origin=Origin(xyz=(0.0, 0.0, 1.156)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=4.712, effort=0.4, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    knob = object_model.get_part("knob")
    oscillation = object_model.get_articulation("base_to_column")
    speed = object_model.get_articulation("column_to_knob")

    ctx.expect_gap(
        column,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.006,
        name="column flange seats just above bearing",
    )
    ctx.expect_overlap(
        column,
        base,
        axes="xy",
        min_overlap=0.080,
        name="round base supports centered column",
    )
    ctx.expect_gap(
        knob,
        column,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="knob_cap",
        negative_elem="speed_boss",
        name="speed knob sits on boss stub",
    )
    ctx.expect_overlap(
        knob,
        column,
        axes="xy",
        min_overlap=0.025,
        elem_a="knob_cap",
        elem_b="speed_boss",
        name="knob is concentric with round boss",
    )

    rest_grille = ctx.part_element_world_aabb(column, elem="front_grille")
    with ctx.pose({oscillation: 0.75}):
        swung_grille = ctx.part_element_world_aabb(column, elem="front_grille")
    ctx.check(
        "oscillation swings front grille sideways",
        rest_grille is not None
        and swung_grille is not None
        and abs(
            ((swung_grille[0][0] + swung_grille[1][0]) * 0.5)
            - ((rest_grille[0][0] + rest_grille[1][0]) * 0.5)
        )
        > 0.030,
        details=f"rest={rest_grille}, swung={swung_grille}",
    )

    with ctx.pose({speed: 4.0}):
        ctx.expect_gap(
            knob,
            column,
            axis="z",
            min_gap=0.0,
            max_gap=0.002,
            positive_elem="knob_cap",
            negative_elem="speed_boss",
            name="rotated speed knob remains seated",
        )

    return ctx.report()


object_model = build_object_model()
