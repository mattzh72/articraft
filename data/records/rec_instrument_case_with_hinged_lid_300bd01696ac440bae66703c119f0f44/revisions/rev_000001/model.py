from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CASE_LENGTH = 0.72
CASE_DEPTH = 0.30
LOWER_HEIGHT = 0.120
LID_HEIGHT = 0.058
SEAM_GAP = 0.004
WALL = 0.015
FLOOR = 0.016
HINGE_BACK_OFFSET = 0.006
HINGE_AXIS_Z = LOWER_HEIGHT + SEAM_GAP
FRONT_Y = -CASE_DEPTH / 2.0
BACK_Y = CASE_DEPTH / 2.0


def _safe_fillet(shape: cq.Workplane, selector: str, radius: float) -> cq.Workplane:
    """Apply a CadQuery fillet when the selected edge set is valid."""
    try:
        return shape.edges(selector).fillet(radius)
    except Exception:
        return shape


def _lower_shell_mesh() -> cq.Workplane:
    """Deeper open tray: rounded hard case shell with a true open instrument well."""
    outer = cq.Workplane("XY").box(
        CASE_LENGTH, CASE_DEPTH, LOWER_HEIGHT, centered=(True, True, False)
    )
    outer = _safe_fillet(outer, "|Z", 0.030)
    outer = _safe_fillet(outer, ">Z", 0.006)

    cutter = cq.Workplane("XY").box(
        CASE_LENGTH - 2.0 * WALL,
        CASE_DEPTH - 2.0 * WALL,
        LOWER_HEIGHT + 0.020,
        centered=(True, True, False),
    )
    cutter = cutter.translate((0.0, 0.0, FLOOR))
    cutter = _safe_fillet(cutter, "|Z", 0.022)
    return outer.cut(cutter)


def _lid_shell_mesh() -> cq.Workplane:
    """Shallow inverted tray whose local origin lies on the rear hinge line."""
    outer = cq.Workplane("XY").box(
        CASE_LENGTH, CASE_DEPTH, LID_HEIGHT, centered=(True, True, False)
    )
    outer = outer.translate((0.0, -CASE_DEPTH / 2.0, 0.0))
    outer = _safe_fillet(outer, "|Z", 0.030)
    outer = _safe_fillet(outer, ">Z", 0.006)

    cutter = cq.Workplane("XY").box(
        CASE_LENGTH - 2.0 * WALL,
        CASE_DEPTH - 2.0 * WALL,
        LID_HEIGHT - 0.010,
        centered=(True, True, False),
    )
    cutter = cutter.translate((0.0, -CASE_DEPTH / 2.0, -0.010))
    cutter = _safe_fillet(cutter, "|Z", 0.022)
    return outer.cut(cutter)


def _capsule_cutter(
    length: float,
    width: float,
    height: float,
    *,
    center: tuple[float, float],
    angle_deg: float = 0.0,
) -> cq.Workplane:
    cx, cy = center
    mid = cq.Workplane("XY").center(cx, cy).rect(length, width).extrude(height)
    cap_a = cq.Workplane("XY").center(cx - length / 2.0, cy).circle(width / 2.0).extrude(height)
    cap_b = cq.Workplane("XY").center(cx + length / 2.0, cy).circle(width / 2.0).extrude(height)
    cutter = mid.union(cap_a).union(cap_b)
    if angle_deg:
        cutter = cutter.rotate((cx, cy, 0.0), (cx, cy, 1.0), angle_deg)
    return cutter


def _foam_insert_mesh() -> cq.Workplane:
    """Velvet-covered insert with trumpet-shaped recesses in the lower shell."""
    foam_thickness = 0.024
    foam = cq.Workplane("XY").box(
        CASE_LENGTH - 2.0 * WALL - 0.010,
        CASE_DEPTH - 2.0 * WALL - 0.010,
        foam_thickness,
        centered=(True, True, False),
    )
    foam = foam.translate((0.0, 0.0, FLOOR))
    foam = _safe_fillet(foam, "|Z", 0.020)

    cut_height = foam_thickness + 0.018
    cutters = [
        cq.Workplane("XY").center(0.230, 0.020).circle(0.070).extrude(cut_height),
        _capsule_cutter(0.390, 0.045, cut_height, center=(-0.105, 0.038), angle_deg=-7.0),
        _capsule_cutter(0.245, 0.036, cut_height, center=(-0.110, -0.054), angle_deg=2.0),
        cq.Workplane("XY").center(0.015, -0.038).rect(0.110, 0.070).extrude(cut_height),
        _capsule_cutter(0.120, 0.026, cut_height, center=(-0.275, -0.006), angle_deg=18.0),
    ]
    for cutter in cutters:
        foam = foam.cut(cutter.translate((0.0, 0.0, FLOOR - 0.004)))
    return foam


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_trumpet_case")

    case_mat = model.material("black_pebbled_case", rgba=(0.015, 0.014, 0.012, 1.0))
    trim_mat = model.material("black_rubber_trim", rgba=(0.003, 0.003, 0.003, 1.0))
    plush_mat = model.material("deep_red_plush", rgba=(0.42, 0.015, 0.025, 1.0))
    metal_mat = model.material("brushed_nickel", rgba=(0.74, 0.70, 0.62, 1.0))
    grip_mat = model.material("black_leather_grip", rgba=(0.020, 0.018, 0.015, 1.0))

    lower = model.part("lower_shell")
    lower.visual(
        mesh_from_cadquery(_lower_shell_mesh(), "lower_shell"),
        material=case_mat,
        name="lower_shell",
    )
    lower.visual(
        mesh_from_cadquery(_foam_insert_mesh(), "trumpet_cavity"),
        material=plush_mat,
        name="trumpet_cavity",
    )
    lower.visual(
        Box((CASE_LENGTH * 0.92, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.004, LOWER_HEIGHT - 0.006)),
        material=trim_mat,
        name="front_seam_trim",
    )
    lower.visual(
        Box((CASE_LENGTH * 0.92, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, BACK_Y + 0.002, LOWER_HEIGHT - 0.012)),
        material=trim_mat,
        name="rear_hinge_leaf",
    )

    # Center carry handle, fixed to the front shell between the two draw latches.
    lower.visual(
        Cylinder(radius=0.014, length=0.170),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.038, 0.064), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_mat,
        name="carry_grip",
    )
    for i, x in enumerate((-0.092, 0.092)):
        lower.visual(
            Box((0.020, 0.040, 0.036)),
            origin=Origin(xyz=(x, FRONT_Y - 0.019, 0.060)),
            material=metal_mat,
            name=f"handle_mount_{i}",
        )

    # Interleaved fixed knuckles on the lower shell side of the long rear hinge.
    for i, (x, length) in enumerate(((-0.270, 0.130), (0.0, 0.150), (0.270, 0.130))):
        lower.visual(
            Cylinder(radius=0.008, length=length),
            origin=Origin(
                xyz=(x, BACK_Y + HINGE_BACK_OFFSET, HINGE_AXIS_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=metal_mat,
            name=f"lower_knuckle_{i}",
        )

    latch_xs = (-0.235, 0.235)
    latch_pivot_z = 0.086
    for i, x in enumerate(latch_xs):
        lower.visual(
            Box((0.072, 0.014, 0.024)),
            origin=Origin(xyz=(x, FRONT_Y - 0.006, latch_pivot_z)),
            material=metal_mat,
            name=f"latch_mount_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_mesh(), "lid_shell"),
        material=case_mat,
        name="lid_shell",
    )
    lid.visual(
        Box((CASE_LENGTH * 0.92, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, -CASE_DEPTH - 0.004, 0.010)),
        material=trim_mat,
        name="lid_front_trim",
    )
    lid.visual(
        Box((CASE_LENGTH * 0.90, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.004, 0.014)),
        material=trim_mat,
        name="lid_hinge_leaf",
    )
    for i, (x, length) in enumerate(((-0.145, 0.105), (0.145, 0.105))):
        lid.visual(
            Cylinder(radius=0.008, length=length),
            origin=Origin(xyz=(x, HINGE_BACK_OFFSET, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"lid_knuckle_{i}",
        )
    for i, x in enumerate(latch_xs):
        lid.visual(
            Box((0.062, 0.010, 0.030)),
            origin=Origin(xyz=(x, -CASE_DEPTH - 0.001, 0.018)),
            material=metal_mat,
            name=f"latch_catch_{i}",
        )

    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=lid,
        origin=Origin(xyz=(0.0, BACK_Y, HINGE_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    for i, x in enumerate(latch_xs):
        latch = model.part(f"latch_{i}")
        latch.visual(
            Cylinder(radius=0.006, length=0.066),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name="pivot_barrel",
        )
        latch.visual(
            Box((0.050, 0.006, 0.078)),
            origin=Origin(xyz=(0.0, -0.004, 0.039)),
            material=metal_mat,
            name="draw_plate",
        )
        latch.visual(
            Box((0.038, 0.010, 0.014)),
            origin=Origin(xyz=(0.0, -0.006, 0.082)),
            material=metal_mat,
            name="pull_tab",
        )
        model.articulation(
            f"latch_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=lower,
            child=latch,
            origin=Origin(xyz=(x, FRONT_Y - 0.014, latch_pivot_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=3.5, lower=0.0, upper=1.25),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    lid_joint = object_model.get_articulation("shell_to_lid")

    ctx.expect_gap(
        lid,
        lower,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="lower_shell",
        min_gap=SEAM_GAP * 0.5,
        max_gap=SEAM_GAP * 1.6,
        name="closed clamshell has a narrow seam gap",
    )
    ctx.expect_overlap(
        lid,
        lower,
        axes="xy",
        elem_a="lid_shell",
        elem_b="lower_shell",
        min_overlap=0.25,
        name="lid footprint covers the lower case",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_joint: 1.25}):
        opened_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid rotates upward about the rear long hinge",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.11
        and opened_aabb[0][1] > closed_aabb[0][1] + 0.08,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    for i in (0, 1):
        latch = object_model.get_part(f"latch_{i}")
        pivot = object_model.get_articulation(f"latch_pivot_{i}")
        ctx.allow_overlap(
            lower,
            latch,
            elem_a=f"latch_mount_{i}",
            elem_b="pivot_barrel",
            reason="The draw latch barrel is intentionally captured in its front hinge mount.",
        )
        ctx.expect_overlap(
            lower,
            latch,
            axes="x",
            elem_a=f"latch_mount_{i}",
            elem_b="pivot_barrel",
            min_overlap=0.040,
            name=f"latch {i} pivot is retained by its mount",
        )
        closed = ctx.part_element_world_aabb(latch, elem="draw_plate")
        with ctx.pose({pivot: 1.0}):
            opened = ctx.part_element_world_aabb(latch, elem="draw_plate")
        ctx.check(
            f"latch {i} flips outward from the front edge",
            closed is not None
            and opened is not None
            and opened[0][1] < closed[0][1] - 0.020,
            details=f"closed={closed}, opened={opened}",
        )

    return ctx.report()


object_model = build_object_model()
