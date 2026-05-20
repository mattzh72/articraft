from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


ALUMINUM = Material("satin_aluminum", color=(0.74, 0.76, 0.76, 1.0))
DARK_ALUMINUM = Material("dark_anodized_aluminum", color=(0.08, 0.09, 0.095, 1.0))
BLACK_RUBBER = Material("black_rubber", color=(0.01, 0.01, 0.012, 1.0))


def _tube_mesh(outer_radius: float, inner_radius: float, height: float, name: str):
    """CadQuery annular tube, open through the center for the telescoping mast."""
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    cutter = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(height + 0.006)
        .translate((0.0, 0.0, -0.003))
    )
    return mesh_from_cadquery(outer.cut(cutter), name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_laptop_stand")

    base = model.part("base")
    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                superellipse_profile(0.46, 0.31, exponent=2.0, segments=96),
                0.032,
            ),
            "oval_base",
        ),
        material=ALUMINUM,
        name="oval_base",
    )
    base.visual(
        Cylinder(radius=0.066, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=ALUMINUM,
        name="socket_boss",
    )
    base.visual(
        _tube_mesh(0.040, 0.026, 0.300, "outer_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=DARK_ALUMINUM,
        name="outer_sleeve",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.0205, length=0.500),
        # The mast stays inserted below the sleeve lip even at full extension.
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=ALUMINUM,
        name="inner_mast",
    )
    pedestal.visual(
        Cylinder(radius=0.037, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.2325)),
        material=ALUMINUM,
        name="top_collar",
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=DARK_ALUMINUM,
        name="swivel_disk",
    )
    head.visual(
        Box((0.055, 0.040, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=DARK_ALUMINUM,
        name="neck_block",
    )
    head.visual(
        Box((0.170, 0.042, 0.030)),
        origin=Origin(xyz=(0.0, -0.004, 0.075)),
        material=DARK_ALUMINUM,
        name="yoke_bridge",
    )
    head.visual(
        Box((0.018, 0.060, 0.092)),
        origin=Origin(xyz=(-0.075, 0.0, 0.114)),
        material=DARK_ALUMINUM,
        name="yoke_cheek_0",
    )
    head.visual(
        Box((0.018, 0.060, 0.092)),
        origin=Origin(xyz=(0.075, 0.0, 0.114)),
        material=DARK_ALUMINUM,
        name="yoke_cheek_1",
    )

    tray = model.part("tray")
    tray.visual(
        Cylinder(radius=0.013, length=0.1322),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=ALUMINUM,
        name="hinge_pin",
    )
    tray.visual(
        Box((0.118, 0.065, 0.018)),
        origin=Origin(xyz=(0.0, 0.030, 0.009)),
        material=ALUMINUM,
        name="rear_saddle",
    )
    tray.visual(
        mesh_from_geometry(
            ExtrudeGeometry.centered(
                rounded_rect_profile(0.360, 0.250, 0.025, corner_segments=10),
                0.014,
            ),
            "tray_plate",
        ),
        origin=Origin(xyz=(0.0, 0.180, 0.017)),
        material=ALUMINUM,
        name="tray_plate",
    )
    for i, x in enumerate((-0.095, 0.095)):
        tray.visual(
            Box((0.108, 0.020, 0.046)),
            origin=Origin(xyz=(x, 0.300, 0.045)),
            material=ALUMINUM,
            name=f"front_lip_{i}",
        )
        tray.visual(
            Box((0.120, 0.020, 0.004)),
            origin=Origin(xyz=(x, 0.178, 0.0255)),
            material=BLACK_RUBBER,
            name=f"rubber_pad_{i}",
        )

    model.articulation(
        "base_to_pedestal",
        ArticulationType.PRISMATIC,
        parent=base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.160),
    )
    model.articulation(
        "pedestal_to_head",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "head_to_tray",
        ArticulationType.REVOLUTE,
        parent=head,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=1.4, lower=-0.45, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    pedestal = object_model.get_part("pedestal")
    head = object_model.get_part("head")
    tray = object_model.get_part("tray")
    slide = object_model.get_articulation("base_to_pedestal")
    swivel = object_model.get_articulation("pedestal_to_head")
    tilt = object_model.get_articulation("head_to_tray")

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    ctx.expect_within(
        pedestal,
        base,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="inner mast is centered inside the outer sleeve",
    )
    ctx.expect_overlap(
        pedestal,
        base,
        axes="z",
        elem_a="inner_mast",
        elem_b="outer_sleeve",
        min_overlap=0.25,
        name="collapsed pedestal retains deep sleeve insertion",
    )

    rest_pedestal_position = ctx.part_world_position(pedestal)
    with ctx.pose({slide: 0.160}):
        ctx.expect_within(
            pedestal,
            base,
            axes="xy",
            inner_elem="inner_mast",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="extended mast remains centered in the sleeve",
        )
        ctx.expect_overlap(
            pedestal,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="outer_sleeve",
            min_overlap=0.10,
            name="extended pedestal still has retained insertion",
        )
        extended_pedestal_position = ctx.part_world_position(pedestal)
    ctx.check(
        "pedestal prismatic joint raises the stand vertically",
        rest_pedestal_position is not None
        and extended_pedestal_position is not None
        and extended_pedestal_position[2] > rest_pedestal_position[2] + 0.150,
        details=f"rest={rest_pedestal_position}, extended={extended_pedestal_position}",
    )

    ctx.expect_gap(
        head,
        pedestal,
        axis="z",
        positive_elem="swivel_disk",
        negative_elem="top_collar",
        max_gap=0.001,
        max_penetration=0.0005,
        name="swivel head sits on top of the telescoping pedestal",
    )
    ctx.expect_gap(
        tray,
        head,
        axis="x",
        positive_elem="hinge_pin",
        negative_elem="yoke_cheek_0",
        max_gap=0.001,
        max_penetration=0.001,
        name="tilt pin is captured by one yoke cheek",
    )
    ctx.expect_gap(
        head,
        tray,
        axis="x",
        positive_elem="yoke_cheek_1",
        negative_elem="hinge_pin",
        max_gap=0.001,
        max_penetration=0.001,
        name="tilt pin is captured by the opposite yoke cheek",
    )

    rest_lip = _center_from_aabb(ctx.part_element_world_aabb(tray, elem="front_lip_0"))
    with ctx.pose({tilt: 0.55}):
        raised_lip = _center_from_aabb(ctx.part_element_world_aabb(tray, elem="front_lip_0"))
    with ctx.pose({tilt: -0.35}):
        lowered_lip = _center_from_aabb(ctx.part_element_world_aabb(tray, elem="front_lip_0"))
    ctx.check(
        "horizontal tilt hinge moves the front retaining lips up and down",
        rest_lip is not None
        and raised_lip is not None
        and lowered_lip is not None
        and raised_lip[2] > rest_lip[2] + 0.10
        and lowered_lip[2] < rest_lip[2] - 0.06,
        details=f"rest={rest_lip}, raised={raised_lip}, lowered={lowered_lip}",
    )

    rest_lip_xy = _center_from_aabb(ctx.part_element_world_aabb(tray, elem="front_lip_1"))
    with ctx.pose({swivel: math.pi / 2.0}):
        swiveled_lip_xy = _center_from_aabb(ctx.part_element_world_aabb(tray, elem="front_lip_1"))
    ctx.check(
        "vertical swivel joint rotates the tray around the pedestal axis",
        rest_lip_xy is not None
        and swiveled_lip_xy is not None
        and math.hypot(swiveled_lip_xy[0] - rest_lip_xy[0], swiveled_lip_xy[1] - rest_lip_xy[1]) > 0.25,
        details=f"rest={rest_lip_xy}, swiveled={swiveled_lip_xy}",
    )

    return ctx.report()


object_model = build_object_model()
