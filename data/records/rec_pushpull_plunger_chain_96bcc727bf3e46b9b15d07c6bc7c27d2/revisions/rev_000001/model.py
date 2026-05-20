from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _annular_tube_x(
    start_x: float,
    end_x: float,
    outer_radius: float,
    inner_radius: float,
    *,
    segments: int = 64,
) -> MeshGeometry:
    """Open tubular sleeve running along +X, with visible annular end faces."""
    length = end_x - start_x
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=segments),
        [_circle_profile(inner_radius, segments=segments)],
        length,
        center=True,
    )
    geom.rotate_y(math.pi / 2.0)
    geom.translate((start_x + end_x) * 0.5, 0.0, 0.0)
    return geom


def _cone_tip_x(radius: float, length: float) -> MeshGeometry:
    geom = ConeGeometry(radius, length, radial_segments=48)
    geom.rotate_y(math.pi / 2.0)
    geom.translate(length * 0.5, 0.0, 0.0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ejector_plunger_stack")

    painted_iron = model.material("painted_iron", rgba=(0.12, 0.16, 0.18, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    polished_rod = model.material("polished_rod", rgba=(0.82, 0.84, 0.86, 1.0))
    blued_steel = model.material("blued_steel", rgba=(0.18, 0.23, 0.28, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.24, 0.18, 0.16)),
        origin=Origin(xyz=(-0.13, 0.0, 0.0)),
        material=painted_iron,
        name="base_block",
    )
    base.visual(
        Box((0.20, 0.22, 0.025)),
        origin=Origin(xyz=(-0.13, 0.0, -0.092)),
        material=painted_iron,
        name="mounting_foot",
    )

    guide_geom = MeshGeometry()
    # Three coaxial guide sleeves overlap only through collars, making the
    # serial nesting/step-down construction obvious in side silhouette.
    guide_geom.merge(_annular_tube_x(-0.012, 0.365, 0.055, 0.037))
    guide_geom.merge(_annular_tube_x(0.285, 0.595, 0.038, 0.025))
    guide_geom.merge(_annular_tube_x(0.525, 0.755, 0.025, 0.0175))
    guide_geom.merge(_annular_tube_x(-0.028, 0.022, 0.070, 0.037))
    guide_geom.merge(_annular_tube_x(0.345, 0.385, 0.061, 0.025))
    guide_geom.merge(_annular_tube_x(0.575, 0.612, 0.043, 0.0175))
    base.visual(
        mesh_from_geometry(guide_geom, "nested_guides"),
        material=satin_steel,
        name="nested_guides",
    )

    rear_rod = model.part("rear_rod")
    rear_rod.visual(
        Cylinder(radius=0.0175, length=0.220),
        origin=Origin(xyz=(0.110, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_rod,
        name="rear_rod_body",
    )
    rear_rod.visual(
        Cylinder(radius=0.021, length=0.014),
        origin=Origin(xyz=(0.227, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blued_steel,
        name="rear_shoulder",
    )

    middle_rod = model.part("middle_rod")
    middle_rod.visual(
        Cylinder(radius=0.0115, length=0.180),
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_rod,
        name="middle_rod_body",
    )
    middle_rod.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.186, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blued_steel,
        name="middle_shoulder",
    )

    front_rod = model.part("front_rod")
    front_rod.visual(
        Cylinder(radius=0.0065, length=0.160),
        origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_rod,
        name="front_rod_body",
    )
    front_rod.visual(
        mesh_from_geometry(_cone_tip_x(0.007, 0.042), "plunger_tip"),
        origin=Origin(xyz=(0.160, 0.0, 0.0)),
        material=polished_rod,
        name="plunger_tip",
    )

    model.articulation(
        "rear_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=rear_rod,
        origin=Origin(xyz=(0.755, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.120),
    )
    model.articulation(
        "middle_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=middle_rod,
        origin=Origin(xyz=(0.989, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=95.0, velocity=0.32, lower=0.0, upper=0.105),
    )
    model.articulation(
        "front_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=front_rod,
        origin=Origin(xyz=(1.181, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.30, lower=0.0, upper=0.085),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    rear = object_model.get_part("rear_rod")
    middle = object_model.get_part("middle_rod")
    front = object_model.get_part("front_rod")
    rear_slide = object_model.get_articulation("rear_slide")
    middle_slide = object_model.get_articulation("middle_slide")
    front_slide = object_model.get_articulation("front_slide")

    for joint in (rear_slide, middle_slide, front_slide):
        ctx.check(
            f"{joint.name} is axial prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(round(v, 6) for v in joint.axis) == (1.0, 0.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    ctx.expect_gap(
        rear,
        base,
        axis="x",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="rear_rod_body",
        negative_elem="nested_guides",
        name="rear rod seats at guide mouth",
    )
    ctx.expect_overlap(rear, base, axes="yz", elem_a="rear_rod_body", elem_b="nested_guides", name="rear rod is coaxial with guides")
    ctx.expect_overlap(middle, rear, axes="yz", elem_a="middle_rod_body", elem_b="rear_rod_body", name="middle rod shares plunger axis")
    ctx.expect_overlap(front, middle, axes="yz", elem_a="front_rod_body", elem_b="middle_rod_body", name="front rod shares plunger axis")

    rear_box = ctx.part_element_world_aabb(rear, elem="rear_rod_body")
    middle_box = ctx.part_element_world_aabb(middle, elem="middle_rod_body")
    front_box = ctx.part_element_world_aabb(front, elem="front_rod_body")
    if rear_box and middle_box and front_box:
        rear_d = rear_box[1][1] - rear_box[0][1]
        middle_d = middle_box[1][1] - middle_box[0][1]
        front_d = front_box[1][1] - front_box[0][1]
        ctx.check(
            "rod diameters step down toward tip",
            rear_d > middle_d > front_d,
            details=f"diameters_y={(rear_d, middle_d, front_d)}",
        )
    else:
        ctx.fail("rod diameter measurement available", "could not read rod element AABBs")

    rest_positions = {
        rear_slide: ctx.part_world_position(rear),
        middle_slide: ctx.part_world_position(middle),
        front_slide: ctx.part_world_position(front),
    }
    with ctx.pose({rear_slide: 0.120, middle_slide: 0.105, front_slide: 0.085}):
        extended_positions = {
            rear_slide: ctx.part_world_position(rear),
            middle_slide: ctx.part_world_position(middle),
            front_slide: ctx.part_world_position(front),
        }

    for joint in (rear_slide, middle_slide, front_slide):
        rest = rest_positions[joint]
        extended = extended_positions[joint]
        ctx.check(
            f"{joint.name} moves forward",
            rest is not None and extended is not None and extended[0] > rest[0] + 0.05,
            details=f"rest={rest}, extended={extended}",
        )

    return ctx.report()


object_model = build_object_model()
