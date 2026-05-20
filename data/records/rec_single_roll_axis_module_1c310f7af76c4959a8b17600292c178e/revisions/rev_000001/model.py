from __future__ import annotations

import math

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
import cadquery as cq


BASE_THICKNESS = 0.035
AXIS_Z = 0.155
CHEEK_Y = 0.150
CHEEK_THICKNESS = 0.034


def _cylinder_y(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    # SDK cylinders are local-Z aligned; this rotation lays them along local Y.
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _cheek_plate(y_center: float) -> cq.Workplane:
    """A fixed cheek plate with a real roll-axis bearing hole."""
    height = 0.215
    length = 0.205
    z_center = BASE_THICKNESS + height / 2.0
    plate = (
        cq.Workplane("XY")
        .box(length, CHEEK_THICKNESS, height)
        .translate((0.0, y_center, z_center))
    )
    hole = (
        cq.Workplane("XZ")
        .center(0.0, AXIS_Z)
        .cylinder(CHEEK_THICKNESS * 4.0, 0.048)
        .translate((0.0, y_center, 0.0))
    )
    return plate.cut(hole)


def _bearing_ring(y_center: float, side_sign: float) -> cq.Workplane:
    """A steel annular bearing liner seated on the inner face of a cheek."""
    inner_face_y = y_center - side_sign * (CHEEK_THICKNESS / 2.0)
    ring_length = 0.014
    # Ring protrudes into the fork gap and embeds a hair into the cheek visual.
    ring_center_y = inner_face_y - side_sign * (ring_length / 2.0 - 0.0006)
    outer = (
        cq.Workplane("XZ")
        .center(0.0, AXIS_Z)
        .cylinder(ring_length, 0.072)
        .translate((0.0, ring_center_y, 0.0))
    )
    inner = (
        cq.Workplane("XZ")
        .center(0.0, AXIS_Z)
        .cylinder(ring_length * 1.8, 0.041)
        .translate((0.0, ring_center_y, 0.0))
    )
    return outer.cut(inner)


def _head_block() -> cq.Workplane:
    """Non-axisymmetric compact head so roll motion is visually legible."""
    return (
        cq.Workplane("XY")
        .box(0.140, 0.112, 0.098)
        .edges("|Y")
        .fillet(0.012)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_cheek_roll_spindle")

    base_paint = Material("dark_anodized_base", rgba=(0.08, 0.09, 0.10, 1.0))
    edge_paint = Material("machined_edges", rgba=(0.45, 0.47, 0.48, 1.0))
    bearing_steel = Material("bearing_steel", rgba=(0.72, 0.72, 0.70, 1.0))
    head_finish = Material("blackened_head", rgba=(0.02, 0.025, 0.03, 1.0))
    flange_finish = Material("plain_output_steel", rgba=(0.82, 0.82, 0.78, 1.0))
    shaft_finish = Material("ground_shaft", rgba=(0.64, 0.65, 0.65, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.560, 0.360, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=base_paint,
        name="base_plate",
    )

    base.visual(
        mesh_from_cadquery(_cheek_plate(CHEEK_Y), "cheek_0"),
        material=base_paint,
        name="cheek_0",
    )
    base.visual(
        mesh_from_cadquery(_bearing_ring(CHEEK_Y, 1.0), "cheek_0_bearing"),
        material=bearing_steel,
        name="cheek_0_bearing",
    )
    base.visual(
        mesh_from_cadquery(_cheek_plate(-CHEEK_Y), "cheek_1"),
        material=base_paint,
        name="cheek_1",
    )
    base.visual(
        mesh_from_cadquery(_bearing_ring(-CHEEK_Y, -1.0), "cheek_1_bearing"),
        material=bearing_steel,
        name="cheek_1_bearing",
    )

    # Low gusset blocks make the cheeks read as bolted fixed supports rather
    # than free-standing plates.
    for y in (CHEEK_Y, -CHEEK_Y):
        base.visual(
            Box((0.150, 0.030, 0.040)),
            origin=Origin(xyz=(0.0, y * 0.86, BASE_THICKNESS + 0.020)),
            material=base_paint,
            name=f"cheek_foot_{0 if y > 0 else 1}",
        )

    for i, (x, y) in enumerate(
        ((-0.230, -0.135), (-0.230, 0.135), (0.230, -0.135), (0.230, 0.135))
    ):
        base.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(x, y, BASE_THICKNESS + 0.004)),
            material=edge_paint,
            name=f"mount_bolt_{i}",
        )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_block(), "head_block"),
        material=head_finish,
        name="head_block",
    )
    head.visual(
        _cylinder_y(0.026, 0.225),
        origin=_y_cylinder_origin(0.0, 0.0, 0.0),
        material=shaft_finish,
        name="shaft",
    )
    head.visual(
        _cylinder_y(0.042, 0.020),
        origin=_y_cylinder_origin(0.0, 0.119, 0.0),
        material=shaft_finish,
        name="journal_0",
    )
    head.visual(
        _cylinder_y(0.042, 0.020),
        origin=_y_cylinder_origin(0.0, -0.119, 0.0),
        material=shaft_finish,
        name="journal_1",
    )
    head.visual(
        _cylinder_y(0.083, 0.020),
        origin=_y_cylinder_origin(0.0, 0.071, 0.0),
        material=flange_finish,
        name="output_face",
    )
    head.visual(
        _cylinder_y(0.045, 0.022),
        origin=_y_cylinder_origin(0.0, -0.071, 0.0),
        material=shaft_finish,
        name="rear_journal",
    )
    head.visual(
        Box((0.018, 0.100, 0.010)),
        origin=Origin(xyz=(0.046, 0.0, 0.052)),
        material=edge_paint,
        name="index_rib",
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=4.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    roll = object_model.get_articulation("base_to_head")

    ctx.allow_overlap(
        base,
        head,
        elem_a="cheek_0_bearing",
        elem_b="journal_0",
        reason=(
            "The rotating journal is intentionally captured in the fixed "
            "bearing liner with a tiny modeled interference fit."
        ),
    )
    ctx.allow_overlap(
        base,
        head,
        elem_a="cheek_1_bearing",
        elem_b="journal_1",
        reason=(
            "The rotating journal is intentionally captured in the fixed "
            "bearing liner with a tiny modeled interference fit."
        ),
    )

    ctx.check(
        "single roll revolute joint",
        len(object_model.articulations) == 1
        and roll.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )
    ctx.check(
        "roll axis is longitudinal Y",
        tuple(round(v, 6) for v in roll.axis) == (0.0, 1.0, 0.0),
        details=f"axis={roll.axis}",
    )
    ctx.expect_origin_distance(
        head,
        base,
        axes="xy",
        max_dist=0.001,
        name="head centered between split cheeks",
    )
    ctx.expect_origin_gap(
        head,
        base,
        axis="z",
        min_gap=0.150,
        max_gap=0.160,
        name="roll axis carried above base plate",
    )
    ctx.expect_within(
        head,
        base,
        axes="y",
        inner_elem="shaft",
        outer_elem="base_plate",
        margin=0.0,
        name="shaft remains inside base footprint",
    )
    for bearing_name, journal_name in (
        ("cheek_0_bearing", "journal_0"),
        ("cheek_1_bearing", "journal_1"),
    ):
        ctx.expect_within(
            head,
            base,
            axes="xz",
            inner_elem=journal_name,
            outer_elem=bearing_name,
            margin=0.0,
            name=f"{journal_name} is nested in {bearing_name}",
        )
        ctx.expect_overlap(
            head,
            base,
            axes="y",
            elem_a=journal_name,
            elem_b=bearing_name,
            min_overlap=0.006,
            name=f"{journal_name} engages {bearing_name}",
        )

    rest_aabb = ctx.part_element_world_aabb(head, elem="head_block")
    with ctx.pose({roll: math.pi / 2.0}):
        rolled_aabb = ctx.part_element_world_aabb(head, elem="head_block")

    def _span(aabb, axis_index: int) -> float:
        return float(aabb[1][axis_index] - aabb[0][axis_index]) if aabb else 0.0

    ctx.check(
        "head block visibly rolls about shaft",
        rest_aabb is not None
        and rolled_aabb is not None
        and _span(rest_aabb, 0) > _span(rest_aabb, 2)
        and _span(rolled_aabb, 2) > _span(rolled_aabb, 0),
        details=f"rest={rest_aabb}, rolled={rolled_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
