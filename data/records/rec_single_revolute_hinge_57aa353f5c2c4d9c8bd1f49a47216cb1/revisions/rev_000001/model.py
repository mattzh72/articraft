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


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_z(radius: float, z_min: float, z_max: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(z_max - z_min).translate((0.0, 0.0, z_min))


def _ring_z(inner_radius: float, outer_radius: float, z_min: float, z_max: float) -> cq.Workplane:
    outer = _cylinder_z(outer_radius, z_min, z_max)
    bore = _cylinder_z(inner_radius, z_min - 0.002, z_max + 0.002)
    return outer.cut(bore)


def _fixed_leaf_body() -> cq.Workplane:
    """Short frame-side leaf, offset web, and continuous spacer barrel."""
    leaf = _box((0.072, 0.006, 0.116), (-0.066, 0.0, 0.0))
    offset_web = _box((0.026, 0.012, 0.135), (-0.020, 0.0, 0.0))
    spacer_barrel = _cylinder_z(0.0125, -0.110, 0.110)
    body = leaf.union(offset_web).union(spacer_barrel)
    return body


def _moving_leaf_body() -> cq.Workplane:
    """Taller hatch-side leaf with two hollow knuckles around the barrel axis."""
    leaf = _box((0.118, 0.006, 0.200), (0.085, 0.0, 0.0))
    knuckle_top = _ring_z(0.0155, 0.0220, 0.046, 0.092)
    knuckle_bottom = _ring_z(0.0155, 0.0220, -0.092, -0.046)
    lug_top = _box((0.020, 0.014, 0.046), (0.026, 0.0, 0.069))
    lug_bottom = _box((0.020, 0.014, 0.046), (0.026, 0.0, -0.069))
    raised_spine = _box((0.014, 0.012, 0.168), (0.037, 0.0, 0.0))
    body = leaf.union(raised_spine).union(knuckle_top).union(knuckle_bottom).union(lug_top).union(lug_bottom)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_inspection_hatch_hinge")

    galvanized = Material("galvanized_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    hatch_yellow = Material("painted_hatch_leaf", rgba=(0.95, 0.68, 0.12, 1.0))
    dark_screw = Material("dark_recessed_screw", rgba=(0.06, 0.06, 0.055, 1.0))

    mount_leaf = model.part("mount_leaf")
    mount_leaf.visual(
        Box((0.072, 0.006, 0.116)),
        origin=Origin(xyz=(-0.074, 0.0, 0.0)),
        material=galvanized,
        name="mount_plate",
    )
    mount_leaf.visual(
        Box((0.035, 0.012, 0.060)),
        origin=Origin(xyz=(-0.0245, 0.0, 0.0)),
        material=galvanized,
        name="offset_web",
    )
    mount_leaf.visual(
        Cylinder(radius=0.0175, length=0.220),
        origin=Origin(),
        material=galvanized,
        name="spacer_barrel",
    )
    for index, z in enumerate((-0.036, 0.036)):
        mount_leaf.visual(
            Cylinder(radius=0.0065, length=0.004),
            origin=Origin(xyz=(-0.066, 0.004, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_screw,
            name=f"mount_screw_{index}",
        )

    moving_leaf = model.part("moving_leaf")
    moving_leaf.visual(
        Box((0.118, 0.006, 0.200)),
        origin=Origin(xyz=(0.093, 0.0, 0.0)),
        material=hatch_yellow,
        name="leaf_plate",
    )
    moving_leaf.visual(
        Box((0.014, 0.012, 0.168)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=hatch_yellow,
        name="raised_spine",
    )
    moving_leaf.visual(
        Box((0.022, 0.014, 0.046)),
        origin=Origin(xyz=(0.033, 0.0, 0.069)),
        material=hatch_yellow,
        name="upper_lug",
    )
    moving_leaf.visual(
        Box((0.022, 0.014, 0.046)),
        origin=Origin(xyz=(0.033, 0.0, -0.069)),
        material=hatch_yellow,
        name="lower_lug",
    )
    moving_leaf.visual(
        mesh_from_cadquery(_ring_z(0.0175, 0.0260, 0.046, 0.092), "upper_hinge_collar", tolerance=0.0004),
        material=hatch_yellow,
        name="upper_collar",
    )
    moving_leaf.visual(
        mesh_from_cadquery(_ring_z(0.0175, 0.0260, -0.092, -0.046), "lower_hinge_collar", tolerance=0.0004),
        material=hatch_yellow,
        name="lower_collar",
    )
    for index, (x, z) in enumerate(((0.089, -0.064), (0.112, 0.0), (0.089, 0.064))):
        moving_leaf.visual(
            Cylinder(radius=0.0060, length=0.004),
            origin=Origin(xyz=(x, 0.004, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_screw,
            name=f"leaf_screw_{index}",
        )

    model.articulation(
        "barrel_axis",
        ArticulationType.REVOLUTE,
        parent=mount_leaf,
        child=moving_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount_leaf = object_model.get_part("mount_leaf")
    moving_leaf = object_model.get_part("moving_leaf")
    hinge = object_model.get_articulation("barrel_axis")

    ctx.allow_overlap(
        mount_leaf,
        moving_leaf,
        elem_a="spacer_barrel",
        elem_b="upper_collar",
        reason="The stationary barrel/pin is intentionally captured inside the upper moving hinge collar.",
    )
    ctx.allow_overlap(
        mount_leaf,
        moving_leaf,
        elem_a="spacer_barrel",
        elem_b="lower_collar",
        reason="The stationary barrel/pin is intentionally captured inside the lower moving hinge collar.",
    )

    ctx.check(
        "single revolute barrel joint",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in hinge.axis) == (0.0, 0.0, 1.0),
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_overlap(
        mount_leaf,
        moving_leaf,
        axes="z",
        min_overlap=0.18,
        elem_a="spacer_barrel",
        elem_b="leaf_plate",
        name="moving leaf spans the spacer barrel height",
    )
    ctx.expect_within(
        mount_leaf,
        moving_leaf,
        axes="xy",
        inner_elem="spacer_barrel",
        outer_elem="upper_collar",
        margin=0.0,
        name="upper collar is centered around the barrel",
    )
    ctx.expect_within(
        mount_leaf,
        moving_leaf,
        axes="xy",
        inner_elem="spacer_barrel",
        outer_elem="lower_collar",
        margin=0.0,
        name="lower collar is centered around the barrel",
    )
    ctx.expect_overlap(
        mount_leaf,
        moving_leaf,
        axes="z",
        min_overlap=0.040,
        elem_a="spacer_barrel",
        elem_b="upper_collar",
        name="upper collar captures the barrel length",
    )
    ctx.expect_overlap(
        mount_leaf,
        moving_leaf,
        axes="z",
        min_overlap=0.040,
        elem_a="spacer_barrel",
        elem_b="lower_collar",
        name="lower collar captures the barrel length",
    )
    ctx.expect_gap(
        moving_leaf,
        mount_leaf,
        axis="x",
        min_gap=0.002,
        positive_elem="leaf_plate",
        negative_elem="mount_plate",
        name="opposed leaves are separated by the barrel at rest",
    )

    def _element_center(part, elem: str) -> tuple[float, float, float] | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return (
            0.5 * (float(lo[0]) + float(hi[0])),
            0.5 * (float(lo[1]) + float(hi[1])),
            0.5 * (float(lo[2]) + float(hi[2])),
        )

    rest_center = _element_center(moving_leaf, "leaf_plate")
    with ctx.pose({hinge: 1.45}):
        swept_center = _element_center(moving_leaf, "leaf_plate")
        ctx.expect_gap(
            moving_leaf,
            mount_leaf,
            axis="y",
            min_gap=0.005,
            positive_elem="leaf_plate",
            negative_elem="mount_plate",
            name="opened leaf sweeps clear around the pin",
        )
    ctx.check(
        "moving leaf center follows an arc about the barrel",
        rest_center is not None
        and swept_center is not None
        and rest_center[0] > 0.055
        and abs(rest_center[1]) < 0.010
        and swept_center[1] > 0.055,
        details=f"rest_center={rest_center}, swept_center={swept_center}",
    )

    return ctx.report()


object_model = build_object_model()
